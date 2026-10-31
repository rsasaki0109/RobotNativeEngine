"""H3 MuJoCo calibration scan; reuse frozen stack geometry and quality gates."""
import argparse
import hashlib
import itertools
import json
import lzma
import os
from pathlib import Path
import statistics
import time

import mujoco
import numpy as np
import stack


def model_xml(case, setting):
    bodies = []
    for i in range(case['levels']):
        mass = case['top_mass_kg'] if i + 1 == case['levels'] else 1
        inertia = mass * 0.1**2 / 6
        height = 0.05 + case['gap_m'] + i * (0.1 + case['gap_m'])
        bodies.append(f'''<body pos="0 {height} 0"><freejoint/>
            <inertial pos="0 0 0" mass="{mass}" diaginertia="{inertia} {inertia} {inertia}"/>
            <geom type="box" size="0.05 0.05 0.05"/></body>''')
    ref = setting['solref']
    return f'''<mujoco><option timestep="{case['dt_ticks'] * 1e-9}" gravity="0 -9.81 0"
        integrator="Euler" solver="Newton" iterations="200" tolerance="1e-10" cone="elliptic"/>
        <default><geom friction="0 0 0" condim="1" margin="0" gap="0"
            solref="{ref[0]} {ref[1]}" solimp="0.999 0.999 0.001 0.5 2"/></default>
        <worldbody><geom type="box" size="5 0.5 5" pos="0 -0.5 0"/>
        {''.join(bodies)}</worldbody></mujoco>'''


def sample(data, count, timestamp, impulses):
    result = []
    for i in range(count):
        p = data.qpos[i * 7:(i + 1) * 7]
        v = data.qvel[i * 6:(i + 1) * 6]
        rotation = np.zeros(9)
        mujoco.mju_quat2Mat(rotation, p[3:7])
        angular = rotation.reshape(3, 3) @ v[3:6]
        result.append([timestamp, *p[:3], *p[4:7], p[3], *v[:3], *angular, impulses[i]])
    return np.array(result).tolist()


def run(case, setting, audit):
    xml = model_xml(case, setting)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    n, dt = case['levels'], case['dt_ticks'] * 1e-9
    steps = 2_000_000_000 // case['dt_ticks']
    trace = [sample(data, n, 0, np.zeros(n))] if audit else []
    force = np.zeros(6)
    start = time.perf_counter()
    if audit:
        for tick in range(steps):
            mujoco.mj_step(model, data)
            impulses = np.zeros(n)
            for j in range(data.ncon):
                a, b = sorted((data.contact[j].geom1, data.contact[j].geom2))
                if b == a + 1 and 0 <= a < n:
                    mujoco.mj_contactForce(model, data, j, force)
                    impulses[a] += force[0] * dt
            trace.append(sample(data, n, (tick + 1) * dt, impulses))
    else:
        mujoco.mj_step(model, data, nstep=steps)
    elapsed = time.perf_counter() - start
    return {'trace': trace, 'final_state': sample(data, n, 2, np.zeros(n)), 'elapsed_s': elapsed,
            'warnings': [int(w.number) for w in data.warning], 'simulation_time_s': float(data.time),
            'model_xml': xml, 'disableflags': int(model.opt.disableflags)}


def run_case(case, setting):
    audit = run(case, setting, True)
    repeated = run(case, setting, True)
    if audit['trace'] != repeated['trace']:
        raise ValueError('exact replay failed')
    run(case, setting, False)
    costs = []
    for _ in range(5):
        timed = run(case, setting, False)
        if timed['final_state'] != audit['final_state']:
            raise ValueError('timing final state differs')
        costs.append(timed['elapsed_s'] / 2)
    quality = stack.metrics(case, audit['trace'])
    quality['gates']['backend_health'] = not any(audit['warnings']) and abs(audit['simulation_time_s'] - 2) < 1e-8
    quality['passed'] = all(quality['gates'].values())
    return {'case': case, 'setting': setting, 'audit': audit, 'metrics': quality,
            'exact_replay': True, 'seconds_per_simulated_second': costs, 'median_cost': statistics.median(costs)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    if args.output.exists() or mujoco.__version__ != '3.9.0':
        p.error('new output path and MuJoCo 3.9.0 required')
    os.sched_setaffinity(0, {min(os.sched_getaffinity(0))})
    report = {'protocol': 'mujoco_stack_calibration_v1', 'mujoco': mujoco.__version__, 'gates': stack.GATES,
              'cpu_affinity': sorted(os.sched_getaffinity(0)), 'source_sha256': {
                  name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                  for name in ('mujoco_stack.py', 'stack.py', 'box.py')},
              'status': 'running', 'results': [], 'comparison_eligible': False}
    for dt, levels, mass, ref in itertools.product((2_000_000, 1_000_000, 250_000), (2, 3), (1, 10), ([0.02, 1], [2e-6, 10])):
        row = run_case({'dt_ticks': dt, 'levels': levels, 'top_mass_kg': mass, 'gap_m': 0.005}, {'solref': ref})
        report['results'].append(row)
        print(json.dumps({'case': row['case'], 'setting': row['setting'], 'metrics': row['metrics']}), flush=True)
    report['status'] = 'complete'
    with lzma.open(args.output, 'xt') as f:
        json.dump(report, f, allow_nan=False)


if __name__ == '__main__':
    main()
