"""Frozen wider-time-constant calibration, preserving the earlier hard-contact scan."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import numpy as np
import mujoco
import force_friction_matched as hard
from force_friction_matched import case, model_xml, run_mujoco, quality, comparison, measure
import force_friction as base
import threshold
import verify_force_friction_matched as verifier

PROTOCOL = 'force_friction_time_constant_v1'
TIMESTEPS = [8_000, 5_000]
SOLREFS = [[0.02, 1], [0.01, 1], [0.005, 1]]
CALIBRATION = hard.CALIBRATION
VALIDATION = hard.VALIDATION
SOURCES = hard.SOURCES + ['force_friction_time_constant.py', 'verify_force_friction_matched.py']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    if args.verify:
        report = verifier.verify(args.directory, sys.modules[__name__])
        print(f"Verified {len(report['records'])} records: {report['comparison']}")
        return
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
