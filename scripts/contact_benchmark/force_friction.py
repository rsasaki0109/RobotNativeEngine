"""Actual COM-force comparison at a frozen common 2 ms timestep (not a tuning scan)."""
import argparse
import hashlib
import itertools
import json
import lzma
import os
from pathlib import Path
import statistics
import subprocess
import time

import mujoco
import numpy as np
import mujoco_stack
import threshold

PROTOCOL = 'force_friction_common_step_v1'
SETTING = {'dt_ticks': 2_000_000, 'native_regularization': 0, 'iterations': 200,
           'mujoco_solref': [2e-6, 10], 'mujoco_solimp': [0.999, 0.999, 0.001, 0.5, 2]}
SOURCES = ('force_friction.py', 'threshold.py', 'box.py', 'mujoco_stack.py')


def cases():
    return [dict(dt_ticks=SETTING['dt_ticks'], mass_kg=m, force_ratio=r, mu=u)
            for m, r, u in itertools.product((0.3, 3), (-1.2, -0.8, 0.8, 1.2), (0.2, 0.5))]


def model_xml(case):
    # Exactly the same f32 material coefficient used by the Native fixture.
    mu = float(np.float32(case['mu']))
    mass = case['mass_kg']
    inertia = mass * 0.1**2 / 6
    return f'''<mujoco><option timestep="{case['dt_ticks'] * 1e-9}" gravity="0 -9.81 0"
      integrator="Euler" solver="Newton" iterations="200" tolerance="1e-10" cone="elliptic"/>
      <default><geom friction="{mu} 0 0" condim="3" margin="0" gap="0"
        solref="2e-6 10" solimp="0.999 0.999 0.001 0.5 2"/></default>
      <worldbody><geom type="box" size="5 0.5 5" pos="0 -0.5 0"/>
        <body pos="0 0.05 0"><freejoint/>
          <inertial pos="0 0 0" mass="{mass}" diaginertia="{inertia} {inertia} {inertia}"/>
          <geom type="box" size="0.05 0.05 0.05"/>
        </body></worldbody></mujoco>'''


def run_mujoco(case, audit):
    xml = model_xml(case)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    steps = 2_000_000_000 // case['dt_ticks']
    dt = case['dt_ticks'] * 1e-9
    force_n = case['mass_kg'] * case['force_ratio'] * float(np.float32(case['mu'])) * 9.81
    trace = [mujoco_stack.sample(data, 1, 0, [0])[0]] if audit else []
    start = time.perf_counter()
    for tick in range(steps):
        # xfrc_applied is a world-frame force/free torque at the body's COM.
        # Reassign every step rather than comparing Native submission to MJ nstep.
        data.xfrc_applied[1, :] = (force_n, 0, 0, 0, 0, 0)
        mujoco.mj_step(model, data)
        if audit:
            trace.append(mujoco_stack.sample(data, 1, (tick + 1) * dt, [0])[0])
    elapsed = time.perf_counter() - start
    return {'backend': 'mujoco', 'load_mode': 'external_body_wrench',
            'trace': trace, 'final_state': mujoco_stack.sample(data, 1, 2, [0])[0],
            'elapsed_s': elapsed, 'warnings': [int(w.number) for w in data.warning],
            'simulation_time_s': float(data.time), 'model_xml': xml,
            'disableflags': int(model.opt.disableflags)}


def run_native(binary, case, audit):
    args = [str(binary), str(case['dt_ticks']), str(case['mass_kg']), str(case['force_ratio']),
            '0', '200', 'audit' if audit else 'timing', str(case['mu']), 'wrench']
    return json.loads(subprocess.run(args, capture_output=True, text=True, check=True, timeout=120).stdout)


def quality(case, audit):
    if audit['load_mode'] != 'external_body_wrench':
        raise ValueError('actual force API required')
    result = threshold.metrics(case, audit['trace'])
    if audit['backend'] == 'mujoco':
        health = (not any(audit['warnings']) and abs(audit['simulation_time_s'] - 2) < 1e-8
                  and audit['disableflags'] == 0 and audit['model_xml'] == model_xml(case))
    elif audit['backend'] == 'rne_native':
        health = not audit['debug_assertions']
    else:
        raise ValueError('unknown backend')
    result['gates']['backend_health'] = health
    result['passed'] = all(result['gates'].values())
    return result


def measure(runner, case):
    audit = runner(case, True)
    if audit['trace'] != runner(case, True)['trace']:
        raise ValueError('exact replay failed')
    runner(case, False)
    costs = []
    for _ in range(5):
        timed = runner(case, False)
        if audit['final_state'][1:14] != timed['final_state'][1:14]:
            raise ValueError('timing state differs')
        costs.append(timed['elapsed_s'] / 2)
    return {'audit': audit, 'metrics': quality(case, audit), 'exact_replay': True,
            'seconds_per_simulated_second': costs, 'median_cost': statistics.median(costs)}


def comparison(rows):
    eligible = all(r[b]['metrics']['passed'] for r in rows for b in ('native', 'mujoco'))
    return {'eligible': eligible, 'mujoco_over_native_cost':
            statistics.mean(r['mujoco']['median_cost'] for r in rows) /
            statistics.mean(r['native']['median_cost'] for r in rows) if eligible else None}


def verify(report):
    if (report['protocol'] != PROTOCOL or report['setting'] != SETTING
            or report['gates'] != threshold.GATES or report['mujoco_version'] != '3.9.0'
            or report['status'] != 'complete'):
        raise ValueError('protocol differs')
    for name in SOURCES:
        if report['source_sha256'][name] != hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest():
            raise ValueError('source differs')
    rows = report['results']
    if [r['case'] for r in rows] != cases():
        raise ValueError('grid differs')
    for row in rows:
        for backend, expected in [('native', 'rne_native'), ('mujoco', 'mujoco')]:
            r = row[backend]
            a = r['audit']
            if (a['backend'] != expected or quality(row['case'], a) != r['metrics']
                    or not r['exact_replay'] or a['trace'][-1][1:14] != a['final_state'][1:14]):
                raise ValueError('audit differs')
            costs = r['seconds_per_simulated_second']
            if len(costs) != 5 or not all(0 < c < float('inf') for c in costs) or statistics.median(costs) != r['median_cost']:
                raise ValueError('cost differs')
    if report['comparison'] != comparison(rows):
        raise ValueError('comparison differs')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    if args.verify:
        with lzma.open(args.output, 'rt') as f:
            verify(json.load(f))
        print('Verified 16 paired actual-force cases')
        return
    if args.output.exists() or mujoco.__version__ != '3.9.0':
        parser.error('new output and MuJoCo 3.9.0 required')
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    binary = args.binary.resolve()
    report = {'protocol': PROTOCOL, 'status': 'running', 'setting': SETTING,
              'gates': threshold.GATES, 'mujoco_version': mujoco.__version__,
              'cpu_affinity': sorted(os.sched_getaffinity(0)),
              'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
              'source_sha256': {n: hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in SOURCES},
              'results': []}
    for case in cases():
        row = {'case': case, 'native': measure(lambda c, a: run_native(binary, c, a), case),
               'mujoco': measure(run_mujoco, case)}
        report['results'].append(row)
        print(json.dumps({'case': case, **{b: row[b]['metrics'] for b in ('native', 'mujoco')}}), flush=True)
    report['status'] = 'complete'
    report['comparison'] = comparison(report['results'])
    verify(report)
    with lzma.open(args.output, 'xt') as f:
        json.dump(report, f, allow_nan=False)


if __name__ == '__main__':
    main()
