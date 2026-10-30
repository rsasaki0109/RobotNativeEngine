"""Recompute quality and verify frozen calibration/validation provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import numpy as np
import force_friction_matched as p


def verify(directory):
    directory = Path(directory)
    report = json.loads((directory/'report.json').read_text())
    if (report['protocol'] != p.PROTOCOL or report['status'] != 'complete'
            or report['gates'] != p.threshold.GATES or report['timesteps'] != p.TIMESTEPS
            or report['solrefs'] != p.SOLREFS or report['mujoco_version'] != '3.9.0'
            or report['calibration'] != [list(c) for c in p.CALIBRATION]
            or report['validation'] != [list(c) for c in p.VALIDATION]):
        raise ValueError('protocol differs')
    for name in p.SOURCES:
        if report['source_sha256'][name] != hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest():
            raise ValueError('source differs')
    records = iter(report['records'])
    used = []
    def consume(backend, phase, c, ref):
        try:
            r = next(records)
        except StopIteration as e:
            raise ValueError('missing record') from e
        if (r['backend'], r['phase'], r['case'], r['solref']) != (backend, phase, c, ref):
            raise ValueError('record order/configuration differs')
        path = directory / r['trace_file']
        if path.parent != directory or path.name != f'trace-{len(used):02}.npz':
            raise ValueError('trace path differs')
        if hashlib.sha256(path.read_bytes()).hexdigest() != r['trace_sha256']:
            raise ValueError('trace digest differs')
        with np.load(path, allow_pickle=False) as data:
            trace = data['trace']
        a = dict(r['audit'], trace=trace)
        if a['backend'] != ('rne_native' if backend == 'native' else 'mujoco'):
            raise ValueError('backend differs')
        if (p.quality(c, ref, a) != r['metrics'] or not r['exact_replay']
                or not np.array_equal(trace[-1,1:14], a['final_state'][1:14])):
            raise ValueError('quality/final state differs')
        if backend == 'native':
            setting = a['settings']
            if (setting['regularization'] != 0 or setting['max_iterations'] != 200
                    or np.float32(setting['friction_coefficient']) != np.float32(c['mu'])):
                raise ValueError('native configuration differs')
        if phase == 'validation':
            costs = r['seconds_per_simulated_second']
            if len(costs) != 5 or not all(0 < t < float('inf') for t in costs) or statistics.median(costs) != r['median_cost']:
                raise ValueError('cost differs')
        used.append(r)
        return r
    screens = iter(report['screens'])
    selected = None
    for dt in p.TIMESTEPS:
        for ref in p.SOLREFS:
            try:
                screen = next(screens)
            except StopIteration as e:
                raise ValueError('missing screen') from e
            if (screen['dt_ticks'], screen['solref']) != (dt, ref):
                raise ValueError('screen order differs')
            a = p.run_mujoco(p.case(p.CALIBRATION[0],dt),ref,True,first_step=True)
            speed = float(np.linalg.norm(a['trace'][1,8:11]))
            rejected = speed > p.threshold.GATES['static_speed_m_s']
            if (screen['rejected'] != rejected or screen['first_step_speed_m_s'] != speed
                    or screen['first_step'] != a['trace'][1].tolist()):
                raise ValueError('first-step screening differs')
            if rejected:
                continue
            rows = [consume('mujoco','calibration',p.case(v,dt),ref) for v in p.CALIBRATION]
            if all(r['metrics']['passed'] for r in rows):
                selected = dict(dt_ticks=dt,solref=ref)
                break
        if selected:
            break
    if next(screens,None) is not None or report['selected'] != selected:
        raise ValueError('selection differs')
    if selected:
        for i,v in enumerate(p.VALIDATION):
            for b in (['native','mujoco'] if i%2==0 else ['mujoco','native']):
                consume(b,'validation',p.case(v,2_000_000 if b=='native' else selected['dt_ticks']),
                        None if b=='native' else selected['solref'])
    if next(records,None) is not None or report['comparison'] != p.comparison(used):
        raise ValueError('comparison/record count differs')
    return report


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    report=verify(parser.parse_args().directory)
    print(f"Verified {len(report['records'])} records; comparison: {report['comparison']}")
