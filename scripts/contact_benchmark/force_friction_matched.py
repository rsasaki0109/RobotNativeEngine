"""Calibrate actual-force friction with unchanged H5 gates, then validate held-out masses."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import statistics
import time

import mujoco
import numpy as np
import force_friction as base
import mujoco_stack
import threshold

PROTOCOL = 'force_friction_matched_v1'
TIMESTEPS = [2_000_000, 250_000, 31_250, 8_000, 5_000]
SOLREFS = [[2e-6, 10], [2e-6, 1]]
CALIBRATION = [(0.3, 0.8, 0.5), (3, 1.2, 0.5)]
VALIDATION = [(1, -0.8, 0.35), (1, -1.2, 0.35), (5, 0.8, 0.35), (5, 1.2, 0.35)]
SOURCES = ['force_friction_matched.py', 'force_friction.py', 'threshold.py', 'mujoco_stack.py', 'box.py']


def case(values, dt):
    mass, ratio, mu = values
    return dict(dt_ticks=dt, mass_kg=mass, force_ratio=ratio, mu=mu)


def model_xml(c, solref):
    return base.model_xml(c).replace('solref="2e-6 10"', f'solref="{solref[0]} {solref[1]}"')


def run_mujoco(c, solref, audit, first_step=False):
    xml = model_xml(c, solref)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    steps = 1 if first_step else 2_000_000_000 // c['dt_ticks']
    dt = c['dt_ticks'] * 1e-9
    force = c['mass_kg'] * c['force_ratio'] * float(np.float32(c['mu'])) * 9.81
    trace = np.empty((steps + 1, 15)) if audit else None
    if audit:
        trace[0] = mujoco_stack.sample(data, 1, 0, [0])[0]
    start = time.perf_counter()
    for tick in range(steps):
        data.xfrc_applied[1, :] = (force, 0, 0, 0, 0, 0)
        mujoco.mj_step(model, data)
        if audit:
            trace[tick + 1] = mujoco_stack.sample(data, 1, (tick + 1) * dt, [0])[0]
    return dict(backend='mujoco', load_mode='external_body_wrench', trace=trace,
                elapsed_s=time.perf_counter()-start,
                final_state=mujoco_stack.sample(data, 1, steps * dt, [0])[0],
                warnings=[int(w.number) for w in data.warning],
                simulation_time_s=float(data.time), model_xml=xml, disableflags=int(model.opt.disableflags))


def quality(c, solref, audit):
    if audit['backend'] == 'rne_native':
        return base.quality(c, audit)
    if audit['load_mode'] != 'external_body_wrench':
        raise ValueError('actual forces required')
    result = threshold.metrics(c, audit['trace'])
    result['gates']['backend_health'] = (not any(audit['warnings']) and audit['disableflags'] == 0
        and abs(audit['simulation_time_s']-2) < 1e-8 and audit['model_xml'] == model_xml(c, solref))
    result['passed'] = all(result['gates'].values())
    return result


def measure(runner, c, solref, timing):
    a = runner(True)
    repeated = runner(True)
    if not np.array_equal(a['trace'], repeated['trace']):
        raise ValueError('exact replay differs')
    del repeated
    result = dict(case=c, solref=solref, audit=a, metrics=quality(c, solref, a), exact_replay=True)
    if timing:
        runner(False)
        costs = []
        for _ in range(5):
            timed = runner(False)
            if a['final_state'][1:14] != timed['final_state'][1:14]:
                raise ValueError('timed state differs')
            costs.append(timed['elapsed_s']/2)
        result.update(seconds_per_simulated_second=costs, median_cost=statistics.median(costs))
    return result


def comparison(rows):
    groups = [[r for r in rows if r['phase'] == 'validation' and r['backend'] == b]
              for b in ['native', 'mujoco']]
    for group in groups:
        if [(r['case']['mass_kg'], r['case']['force_ratio'], r['case']['mu']) for r in group] != VALIDATION:
            return {'eligible': False, 'reason': 'missing validation'}
    if not all(r['metrics']['passed'] for group in groups for r in group):
        return {'eligible': False, 'reason': 'validation quality failure'}
    costs = [statistics.mean(r['median_cost'] for r in group) for group in groups]
    return {'eligible': True, 'native_cost': costs[0], 'mujoco_cost': costs[1],
            'ratio_mujoco_over_native': costs[1]/costs[0]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    if args.directory.exists() or mujoco.__version__ != '3.9.0':
        parser.error('new directory and MuJoCo 3.9.0 required')
    args.directory.mkdir(parents=True)
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = dict(protocol=PROTOCOL, status='running', gates=threshold.GATES,
        timesteps=TIMESTEPS, solrefs=SOLREFS, calibration=CALIBRATION, validation=VALIDATION,
        selection='dt order then solref order; reject first-step static-speed failures; first full calibration pass',
        mujoco_version=mujoco.__version__, cpu_affinity=sorted(os.sched_getaffinity(0)),
        binary_sha256=hashlib.sha256(args.binary.read_bytes()).hexdigest(),
        source_sha256={n:hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in SOURCES},
        screens=[], records=[], selected=None)
    def checkpoint():
        (args.directory/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    def save(r, backend, phase):
        r.update(backend=backend, phase=phase)
        trace = r['audit'].pop('trace')
        path = args.directory/f'trace-{len(report["records"]):02}.npz'
        np.savez_compressed(path, trace=np.asarray(trace))
        r['trace_file'] = path.name
        r['trace_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
        report['records'].append(r)
        checkpoint()
        print(json.dumps({k:r[k] for k in ['backend','phase','case','metrics']}), flush=True)
    checkpoint()
    for dt in TIMESTEPS:
        for solref in SOLREFS:
            c = case(CALIBRATION[0], dt)
            a = run_mujoco(c, solref, True, first_step=True)
            speed = float(np.linalg.norm(a['trace'][1,8:11]))
            rejected = speed > threshold.GATES['static_speed_m_s']
            report['screens'].append(dict(dt_ticks=dt, solref=solref, first_step_speed_m_s=speed,
                                         rejected=rejected, first_step=a['trace'][1].tolist()))
            checkpoint()
            if rejected:
                continue
            rows=[]
            for values in CALIBRATION:
                c=case(values,dt)
                r=measure(lambda audit:run_mujoco(c,solref,audit),c,solref,False)
                rows.append(r)
                save(r,'mujoco','calibration')
            if all(r['metrics']['passed'] for r in rows):
                report['selected']=dict(dt_ticks=dt,solref=solref)
                checkpoint()
                break
        if report['selected']:
            break
    if report['selected']:
        selected=report['selected']
        for i,values in enumerate(VALIDATION):
            for backend in (['native','mujoco'] if i%2==0 else ['mujoco','native']):
                c=case(values,2_000_000 if backend=='native' else selected['dt_ticks'])
                solref=None if backend=='native' else selected['solref']
                runner=(lambda audit:base.run_native(args.binary.resolve(),c,audit)) if backend=='native' else (lambda audit:run_mujoco(c,solref,audit))
                save(measure(runner,c,solref,True),backend,'validation')
    report['comparison']=comparison(report['records'])
    report['status']='complete'
    checkpoint()


if __name__=='__main__':
    main()
