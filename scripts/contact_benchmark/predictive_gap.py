"""Audit finite-gap landing against a semi-implicit free-flight oracle."""
import argparse
import hashlib
import itertools
import json
import lzma
from pathlib import Path
import subprocess
import numpy as np
import box

PROTOCOL = 'predictive_gap_v1'
GAPS = [0.00005, 0.0005, 0.005]
SPEEDS = [0.0, 0.2]
MARGINS = [0.0, 0.0001, 0.02]
TIMESTEPS = [2000000, 250000]
SOURCES = ['predictive_gap.py', 'box.py']


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cases():
    return [dict(dt_ticks=dt, initial_gap_m=g, initial_speed_m_s=v, margin_m=m)
            for dt, g, v, m in itertools.product(TIMESTEPS, GAPS, SPEEDS, MARGINS)]


def run(binary, case):
    args = [str(binary.resolve()), str(case['dt_ticks']), '3', '1.2', '0', '200',
            'audit', '.5', 'wrench', 'diagnostics', str(case['margin_m']),
            str(case['initial_gap_m']), str(case['initial_speed_m_s'])]
    audit = json.loads(subprocess.run(args, check=True, capture_output=True,
                                     text=True, timeout=120).stdout)
    del audit['elapsed_s']
    return audit


def summarize(case, audit):
    dt = case['dt_ticks']*1e-9
    count = 50000000//case['dt_ticks']
    trace = np.asarray(audit['trace'], dtype=float)
    rows = audit['contact_diagnostics']
    s = audit['settings']
    if (trace.shape != (count+1, 15) or len(rows) != count
            or not np.isfinite(trace).all() or audit['debug_assertions']
            or audit['load_mode'] != 'external_body_wrench'
            or s['initial_gap_m'] != case['initial_gap_m']
            or s['initial_speed_m_s'] != case['initial_speed_m_s']
            or s['contact_margin_m'] != case['margin_m']
            or s['regularization'] != 0 or s['max_iterations'] != 200
            or abs(s['equivalent_force_n']-17.658) > 1e-12):
        raise ValueError('fixture differs')
    if not np.allclose(trace[:, 0], np.arange(count+1)*dt, rtol=0, atol=1e-12):
        raise ValueError('clock differs')
    initial = np.zeros(15)
    initial[2] = .05+case['initial_gap_m']; initial[7] = 1
    initial[8] = case['initial_speed_m_s']
    if not np.allclose(trace[0], initial, rtol=0, atol=1e-12):
        raise ValueError('initial state differs')
    first = None; first_gap = None
    for i, row in enumerate(rows):
        if row['before'][1:14] != audit['trace'][i][1:14]:
            raise ValueError('pre-step state differs')
        c = row['pipeline']
        if not c['generated'] >= c['deduplicated'] >= c['retained'] == len(row['points']):
            raise ValueError('pipeline differs')
        impulse = 0.0
        loaded = []
        for point in row['points']:
            if point['dt_s'] != dt or point['gap_m'] > case['margin_m']:
                raise ValueError('candidate distance/duration differs')
            j = point['normal_impulse_n_s']
            if not np.isfinite(j) or j < 0:
                raise ValueError('invalid normal impulse')
            impulse += j
            if j > 1e-10: loaded.append(point['gap_m'])
        if first is None and impulse > 1e-10:
            first = i+1
            first_gap = max(loaded, default=None)
    if first is None:
        raise ValueError('expected landing within 50 ms')
    # Gravity-only semi-implicit flight y_n = y_0 - g dt² n(n+1)/2.
    n = np.arange(count+1)
    free_gap = case['initial_gap_m'] - 9.81*dt*dt*n*(n+1)/2
    expected = int(np.flatnonzero(free_gap <= 0)[0])
    before = np.arange(first)
    expected_y = .05+free_gap[:first]
    expected_x = case['initial_speed_m_s']*dt*before + 5.886*dt*dt*before*(before+1)/2
    position_error = max(float(np.abs(trace[:first, 2]-expected_y).max()),
                       float(np.abs(trace[:first, 1]-expected_x).max()))
    velocity_error = max(float(np.abs(trace[:first, 8]-(case['initial_speed_m_s']+5.886*dt*before)).max()),
                       float(np.abs(trace[:first, 9]+9.81*dt*before).max()))
    if position_error > 1e-9 or velocity_error > 1e-9:
        raise ValueError('force before landing changes free flight')
    # Separately expose rather than hide premature or missed admission.
    return dict(first_loaded_step=first, first_loaded_time_s=first*dt,
                free_flight_crossing_step=expected, landing_delay_steps=first-expected,
                first_loaded_gap_m=first_gap,
                free_flight_position_error_m=position_error,
                free_flight_velocity_error_m_s=velocity_error,
                max_penetration_m=max(0.0, float(-box.corners(trace)[:, :, 1].min())),
                max_spin_rad_s=float(np.linalg.norm(trace[:, 11:14], axis=1).max()),
                final_height_m=float(trace[-1, 2]),
                final_horizontal_speed_m_s=float(trace[-1, 8]))


def verify(path, binary):
    with lzma.open(path, 'rt') as stream: report = json.load(stream)
    if (report['protocol'] != PROTOCOL or report['binary_sha256'] != digest(binary)
            or report['source_sha256'] != {name: digest(Path(__file__).with_name(name)) for name in SOURCES}
            or [r['case'] for r in report['results']] != cases()):
        raise ValueError('protocol/provenance differs')
    for row in report['results']:
        if summarize(row['case'], row['audit']) != row['metrics']:
            raise ValueError('metrics differ')
        if run(binary, row['case']) != row['audit']:
            raise ValueError('exact replay differs')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    if args.verify:
        verify(args.output, args.binary); print('Verified 36 finite-gap replays'); return
    if args.output.exists(): parser.error('new output required')
    report = dict(protocol=PROTOCOL, binary_sha256=digest(args.binary),
                  source_sha256={name: digest(Path(__file__).with_name(name)) for name in SOURCES}, results=[])
    for case in cases():
        audit = run(args.binary, case)
        metrics = summarize(case, audit)
        report['results'].append(dict(case=case, audit=audit, metrics=metrics))
        print(json.dumps(dict(case=case, metrics=metrics)), flush=True)
    with lzma.open(args.output, 'xt') as stream:
        json.dump(report, stream, allow_nan=False)
    verify(args.output, args.binary)


if __name__ == '__main__':
    main()
