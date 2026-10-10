#!/usr/bin/env python3
"""Independently check captured motion, clearance, telemetry and replay binding."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from encode import ARTIFACT_ROOT, require, verify_source_provenance


ROOT = Path(__file__).resolve().parent


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace', type=Path, default=ARTIFACT_ROOT / 'trace.json')
    parser.add_argument('--validation', type=Path, default=ARTIFACT_ROOT / 'validation.json')
    parser.add_argument('--independent-trace', type=Path,
                        default=ARTIFACT_ROOT / 'independent-trace.json')
    parser.add_argument('--independent-validation', type=Path,
                        default=ARTIFACT_ROOT / 'independent-validation.json')
    parser.add_argument('--output', type=Path,
                        default=ARTIFACT_ROOT / 'independent-recording-check.json')
    args = parser.parse_args()
    trace = json.loads(args.trace.read_text())
    verify_source_provenance(trace)
    validation = trace['validation']
    require(validation['passed'], "Recording integrity check failed: validation['passed']")
    require(sha(args.trace) == sha(args.independent_trace), "Recording integrity check failed: sha(args.trace) == sha(args.independent_trace)")
    require(sha(args.validation) == sha(args.independent_validation), "Recording integrity check failed: sha(args.validation) == sha(args.independent_validation)")
    frames = trace['frames']
    poses = np.array([[[r['x_m'], r['z_m']] for r in f['robots']] for f in frames])
    require(poses.shape == (len(frames), 40, 2), 'Recording integrity check failed: poses.shape == (len(frames), 40, 2)')
    dt = trace['capture_dt_s']
    radius = trace['robot_radius_m']
    acceleration = 1.8
    single_error_m = acceleration * dt * dt / 8
    before, after = poses[:-1], poses[1:]
    r0 = before[:, :, None, :] - before[:, None, :, :]
    dv = (after - before)[:, :, None, :] - (after - before)[:, None, :, :]
    denominator = np.sum(dv * dv, axis=-1)
    parameter = np.divide(-np.sum(r0 * dv, axis=-1), denominator,
                          out=np.zeros_like(denominator), where=denominator > 0)
    parameter = np.clip(parameter, 0, 1)
    distances = np.linalg.norm(r0 + parameter[..., None] * dv, axis=-1)
    mask = np.triu(np.ones((40, 40), dtype=bool), k=1)
    minimum_linear = float(np.min(distances[:, mask]))
    minimum_conservative = minimum_linear - 2 * single_error_m
    require(minimum_conservative >= 2 * radius - 1e-8, 'Recording integrity check failed: minimum_conservative >= 2 * radius - 1e-08')

    segment_delta = after - before
    rack_conflicts = 0
    for rack in trace['racks']:
        center = np.array([rack['x_m'], rack['z_m']])
        half = np.array([rack['width_m'], rack['depth_m']]) / 2
        low, high = center - half - radius - single_error_m, center + half + radius + single_error_m
        enter = np.zeros(before.shape[:2])
        leave = np.ones(before.shape[:2])
        for axis in range(2):
            moving = np.abs(segment_delta[..., axis]) > 1e-12
            outside = (before[..., axis] < low[axis]) | (before[..., axis] > high[axis])
            t0 = np.divide(low[axis] - before[..., axis], segment_delta[..., axis],
                           out=np.full(before.shape[:2], -np.inf), where=moving)
            t1 = np.divide(high[axis] - before[..., axis], segment_delta[..., axis],
                           out=np.full(before.shape[:2], np.inf), where=moving)
            axis_enter = np.where(moving, np.minimum(t0, t1), np.where(outside, np.inf, -np.inf))
            axis_leave = np.where(moving, np.maximum(t0, t1), np.where(outside, -np.inf, np.inf))
            enter = np.maximum(enter, axis_enter)
            leave = np.minimum(leave, axis_leave)
        rack_conflicts += int(np.count_nonzero(enter <= leave))
    require(rack_conflicts == 0, 'Conservatively inflated rack bounds were entered')

    require(np.all(poses[..., 0] >= radius), 'Recording integrity check failed: np.all(poses[..., 0] >= radius)')
    require(np.all(poses[..., 0] <= trace['extent_m']['width'] - radius), "Recording integrity check failed: np.all(poses[..., 0] <= trace['extent_m']['width'] - radius)")
    require(np.all(poses[..., 1] >= radius), 'Recording integrity check failed: np.all(poses[..., 1] >= radius)')
    require(np.all(poses[..., 1] <= trace['extent_m']['depth'] - radius), "Recording integrity check failed: np.all(poses[..., 1] <= trace['extent_m']['depth'] - radius)")
    maximum_average_speed = float(np.max(np.linalg.norm(after - before, axis=-1) / dt))
    require(maximum_average_speed <= 1.2 + 1e-8, 'Recording integrity check failed: maximum_average_speed <= 1.2 + 1e-08')
    yaw = np.array([[r['yaw_rad'] for r in f['robots']] for f in frames])
    angular = (np.diff(yaw, axis=0) + np.pi) % (2 * np.pi) - np.pi
    maximum_average_yaw_rate = float(np.max(np.abs(angular) / dt))
    require(maximum_average_yaw_rate <= 1 + 1e-8, 'Recording integrity check failed: maximum_average_yaw_rate <= 1 + 1e-08')

    for frame in frames:
        robots = frame['robots']
        require(len({r['id'] for r in robots}) == 40, "Recording integrity check failed: len({r['id'] for r in robots}) == 40")
        require(frame['kpis']['waiting'] == sum(r['state'] == 'waiting' for r in robots), "Recording integrity check failed: frame['kpis']['waiting'] == sum((r['state'] == 'waiting' for r in robots))")
        require(frame['kpis']['charging'] == sum(r['operational_state'] == 'charging' for r in robots), "Recording integrity check failed: frame['kpis']['charging'] == sum((r['operational_state'] == 'charging' for r in robots))")
        require(frame['kpis']['active'] == sum(r['operational_state'] != 'idle' for r in robots), "Recording integrity check failed: frame['kpis']['active'] == sum((r['operational_state'] != 'idle' for r in robots))")
        require(all(0 <= r['battery_percent'] <= 100 for r in robots), "Recording integrity check failed: all((0 <= r['battery_percent'] <= 100 for r in robots))")
        if frame['incident']['active']:
            require(all(not (abs(r['x_m'] - 24) < 1e-8 and 11 <= r['z_m'] <= 16)
                       for r in robots), "Recording integrity check failed: all((not (abs(r['x_m'] - 24) < 1e-08 and 11 <= r['z_m'] <= 16) for r in robots))")
    # Docking is an observed event inside the exported clip, not inferred from an initial state.
    dock_events = [e for e in trace['events'] if e['kind'] == 'charge_started'
                   and frames[0]['time_s'] <= e['time_s'] <= frames[-1]['time_s']]
    require(dock_events, 'Clip does not show actual charger arrival')
    chargers = [station for station in trace['stations'] if station['kind'] == 'charge']
    for event in dock_events:
        visible = [r for f in frames if f['time_s'] >= event['time_s']
                   for r in f['robots'] if r['id'] == event['robot_id']
                   and r['operational_state'] == 'charging']
        require(visible, 'Recorded dock event has no visible charging state')
        require(all(any(np.hypot(r['x_m'] - station['x_m'], r['z_m'] - station['z_m']) < 1e-8
                       for station in chargers) for r in visible), "Recording integrity check failed: all((any((np.hypot(r['x_m'] - station['x_m'], r['z_m'] - station['z_m']) < 1e-08 for station in chargers)) for r in visible))")
    proof = {'passed': True, 'scope': 'Fresh-process byte-identical trace/validation; independent captured-interval conservative geometry and motion checks, plus full-run simulator checks.',
             'trace_sha256': sha(args.trace),
             'captured_frames': len(frames), 'robots_per_frame': 40,
             'capture_interval_s': [frames[0]['time_s'], frames[-1]['time_s']],
             'minimum_captured_linear_swept_spacing_m': minimum_linear,
             'minimum_captured_conservative_spacing_m': minimum_conservative,
             'pair_interpolation_error_bound_m': 2 * single_error_m,
             'inflated_rack_conflicts': rack_conflicts,
             'maximum_capture_average_speed_m_s': maximum_average_speed,
             'maximum_capture_average_yaw_rate_rad_s': maximum_average_yaw_rate,
             'recorded_docking_events': dock_events,
             'same_runtime_only': True}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(proof, indent=2) + '\n')
    print(json.dumps(proof, indent=2))


if __name__ == '__main__':
    main()
