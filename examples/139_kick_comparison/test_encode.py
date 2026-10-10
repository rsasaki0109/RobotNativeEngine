"""Synthetic encoder validation tests; these do not measure physical recovery."""

import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


ENCODER = Path(__file__).resolve().with_name("encode.py")
REPO = ENCODER.parents[2]
SPEC = importlib.util.spec_from_file_location('kick_encode', ENCODER)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
REQUIRED = {'nominal', 'zero', 'reverse_early', 'late', 'rate_500hz', 'balance_off_zero'}
NAMES = sorted(REQUIRED | {'balance_off', 'front_limit', 'oblique_limit'})


def current_manifest():
    names = ['examples/139_kick_comparison/' + name for name in
             ('main.rs', 'physics.rs', 'render.rs', 'model.rs', 'disturbance.rs',
              'go2_controller.rs', 'g1_controller.rs', 'observation.rs', 'diagnostic.rs', 'models.json',
              'go2.rne.scene.toml', 'g1.rne.scene.toml',
              'go2.rne.robot.toml', 'g1.rne.robot.toml')]
    models = json.loads((REPO / 'examples/139_kick_comparison/models.json').read_text())
    names += [model['derived'] for model in models['models']]
    names += ['crates/rne_data/src/frame.rs', 'crates/rne_data/src/bus.rs',
              'crates/rne_data/src/stream.rs', 'crates/rne_core/src/rng.rs',
              'crates/rne_core/src/time.rs', 'crates/rne_world/src/resources.rs',
              'crates/rne_physics_rapier/src/backend.rs',
              'assets/fixtures/kick_human/cc0_sport_human.glb']
    return {name: hashlib.sha256((REPO / name).read_bytes()).hexdigest() for name in names}


def make_trace(robot, strength, name='nominal'):
    dt_s = .002 if name == 'rate_500hz' else .001
    impulse = 0.0 if name in {'zero', 'balance_off_zero'} else strength
    onset = {'reverse_early': 1.5, 'late': 2.25}.get(name, 2.0)
    direction = {'reverse_early': [0, 0, -1], 'front_limit': [1, 0, 0],
                 'oblique_limit': [math.sqrt(.5), 0, math.sqrt(.5)]}.get(name, [0, 0, 1])
    count = round(.08 / dt_s)
    start = round(onset / dt_s) + 1
    weights = [math.sin(math.pi * (n + .5) / count) for n in range(count)]
    scale = impulse / (sum(weights) * dt_s)
    history = []
    for step in range(1, round(6.2 / dt_s) + 1):
        n = step - start
        value = weights[n] * scale if 0 <= n < count else 0.0
        history.append({'step': step, 'time_s': step * dt_s, 'dt_s': dt_s,
                        'force_world_n': [value * axis for axis in direction]})
    frames = [{'time_s': round(index / 30.0 / dt_s) * dt_s} for index in range(187)]
    return {'schema_version': 2, 'robot': robot, 'case': name,
            'dt_s': dt_s, 'impulse_n_s': impulse, 'push_start_s': onset,
            'push_duration_s': .08, 'force_history': history,
            'force_n': max(weights) * scale,
            'integrated_impulse_world_ns': [impulse * axis for axis in direction],
            'frames': frames,
            'summary': {'recovered': name not in {'front_limit', 'oblique_limit'}},
            'controller': {'contact_load_regulation_enabled': robot == 'Go2'},
            'plant': {'mass_kg': 16.087000011 if robot == 'Go2' else 34.133857284},
            'observed_state_hash': robot + '-' + name,
            'observed_state_bits_exact_repeat': True,
            'observation_pipeline': {
                'profile': 'ideal_reference', 'sample_period_ticks': round(dt_s * 1_000_000_000),
                'latency_ticks': 0, 'phase_offset_ticks': 0,
                'noise': {key: 0.0 for key in ('attitude_world_x_z_bound_rad',
                    'angular_velocity_axis_bound_rad_s', 'com_and_foot_position_axis_bound_m',
                    'com_velocity_axis_bound_m_s', 'go2_foot_normal_load_bound_n')}},
            'observation_control_state_hash': hashlib.sha256((robot + name).encode()).hexdigest()[:16],
            'observation_control_state_bits_exact_repeat': True,
            'canonical_render_frames_exact_repeat': True,
            'compiled_source_sha256': {'synthetic_fixture': 'not physical evidence'}}


def summary_for(traces):
    cases = []
    for trace in traces:
        case = {key: trace[key] for key in
                ('robot', 'force_n', 'dt_s', 'impulse_n_s', 'controller', 'plant',
                 'summary', 'observed_state_hash', 'observed_state_bits_exact_repeat',
                 'canonical_render_frames_exact_repeat', 'observation_pipeline',
                 'observation_control_state_hash', 'observation_control_state_bits_exact_repeat')}
        case['duration_s'] = trace['push_duration_s']
        cases.append(case)
    return {'schema_version': 2, 'cases': cases}


def make_report(root, strength):
    cases, nominal = [], []
    for robot in ('Go2', 'G1'):
        for name in NAMES:
            trace = make_trace(robot, strength, name)
            (root / f'{robot.lower()}-{name}.json').write_text(json.dumps(trace))
            case = {key: trace[key] for key in
                    ('robot', 'case', 'dt_s', 'impulse_n_s', 'summary',
                     'observed_state_hash', 'observed_state_bits_exact_repeat',
                     'controller', 'plant', 'push_start_s', 'push_duration_s',
                     'integrated_impulse_world_ns', 'compiled_source_sha256', 'observation_pipeline',
                     'observation_control_state_hash', 'observation_control_state_bits_exact_repeat')}
            case['required_recovery'] = name in REQUIRED
            cases.append(case)
            if name == 'nominal':
                nominal.append(trace)
    report = {'robots': ['Go2', 'G1'], 'cases': cases,
              'required_cases_passed': True, 'trajectory_convergence_claimed': False}
    path = root / 'recovery-validation.json'
    path.write_text(json.dumps(report))
    return path, report, nominal


class EncoderStrongerImpulseTests(unittest.TestCase):
    def setUp(self):
        # Test numeric/report validation separately from repository provenance.
        self.manifest_mock = patch.object(MODULE, 'validate_compiled_sources',
                                          side_effect=lambda trace, repo: trace['compiled_source_sha256'])
        self.manifest_mock.start()
        self.addCleanup(self.manifest_mock.stop)
        self.traces = [make_trace(robot, 48.0) for robot in ('Go2', 'G1')]
        self.summary = summary_for(self.traces)

    def test_accepts_measured_32_and_48_ns(self):
        for strength in (32.0, 48.0):
            traces = [make_trace(robot, strength) for robot in ('Go2', 'G1')]
            measurements = MODULE.validate_recordings(traces, summary_for(traces))
            self.assertEqual([m['robot'] for m in measurements], ['Go2', 'G1'])
            for measured in measurements:
                self.assertAlmostEqual(measured['impulse_n_s'], strength, places=11)
                self.assertAlmostEqual(measured['measured_peak_force_n'],
                                       471.11778910882333 * strength / 24, places=9)

    def test_rejects_invalid_common_impulses(self):
        for value in (0, -48, float('nan'), float('inf'), True):
            with self.subTest(value=value):
                traces = copy.deepcopy(self.traces)
                traces[0]['impulse_n_s'] = value
                with self.assertRaises(ValueError):
                    MODULE.validate_recordings(traces, self.summary)

    def test_nonideal_estimates_and_missing_controller_replay_cannot_qualify_media(self):
        mutations = (
            lambda trace: trace['observation_pipeline'].update(profile='combined'),
            lambda trace: trace['observation_pipeline'].update(sample_period_ticks=4_000_000),
            lambda trace: trace['observation_pipeline'].update(latency_ticks=5_000_000),
            lambda trace: trace['observation_pipeline']['noise'].update(attitude_world_x_z_bound_rad=.01),
            lambda trace: trace.update(observation_control_state_bits_exact_repeat=False),
            lambda trace: trace.update(observation_control_state_hash=''),
        )
        for mutate in mutations:
            traces = copy.deepcopy(self.traces)
            mutate(traces[0])
            with self.assertRaises(ValueError):
                MODULE.validate_recordings(traces, summary_for(traces))

    def test_diagnosis_evidence_cannot_be_relabelled_as_nominal_media(self):
        # The profile, nominal labels, replay evidence, and clocks remain media-valid.
        for robots in (('G1',), ('Go2', 'G1')):
            with self.subTest(capture_robots=robots):
                traces = copy.deepcopy(self.traces)
                for trace in traces:
                    if trace['robot'] in robots:
                        trace['g1_observation_diagnosis'] = True
                with self.assertRaisesRegex(ValueError, 'diagnosis evidence cannot qualify'):
                    MODULE.validate_recordings(traces, summary_for(traces))
        with tempfile.TemporaryDirectory() as directory:
            path, report, nominal = make_report(Path(directory), 48.0)
            self.assertTrue(MODULE.validation_metadata(path, nominal)['required_cases_passed'])
            marked_nominal = copy.deepcopy(nominal)
            marked_nominal[1]['g1_observation_diagnosis'] = True
            with self.assertRaisesRegex(ValueError, 'diagnosis evidence cannot qualify'):
                MODULE.validation_metadata(path, marked_nominal)
            marked_report = dict(report, g1_observation_diagnosis=True)
            path.write_text(json.dumps(marked_report))
            with self.assertRaisesRegex(ValueError, 'diagnosis evidence cannot qualify'):
                MODULE.validation_metadata(path, nominal)
            trace_path = path.parent / 'g1-nominal.json'
            trace = json.loads(trace_path.read_text())
            trace['g1_observation_diagnosis'] = True
            trace_path.write_text(json.dumps(trace))
            case = next(case for case in report['cases']
                        if case['robot'] == 'G1' and case['case'] == 'nominal')
            for marker_in_case in (False, True):
                with self.subTest(marker_in_validation_case=marker_in_case):
                    if marker_in_case:
                        case['g1_observation_diagnosis'] = True
                    path.write_text(json.dumps(report))
                    with self.assertRaisesRegex(ValueError, 'diagnosis evidence cannot qualify'):
                        MODULE.validation_metadata(path, nominal)

    def test_rejects_mismatched_panels(self):
        self.traces[1] = make_trace('G1', 32.0)
        with self.assertRaises(ValueError):
            MODULE.validate_recordings(self.traces, summary_for(self.traces))

    def test_rejects_stale_summary_declaration(self):
        self.summary['cases'][0]['impulse_n_s'] = 24.0
        with self.assertRaises(ValueError):
            MODULE.validate_recordings(self.traces, self.summary)

    def test_rejects_incorrect_integrated_declaration(self):
        self.traces[0]['integrated_impulse_world_ns'][2] = 24.0
        with self.assertRaises(ValueError):
            MODULE.validate_recordings(self.traces, self.summary)

    def test_rejects_force_change_even_when_declarations_match(self):
        self.traces[0]['force_history'][2001]['force_world_n'][2] *= .5
        with self.assertRaises(ValueError):
            MODULE.validate_recordings(self.traces, self.summary)

    def test_rejects_hidden_opposite_axis_force(self):
        # Opposite X samples cancel in the integral but are not a lateral pulse.
        self.traces[0]['force_history'][2001]['force_world_n'][0] = 10
        self.traces[0]['force_history'][2002]['force_world_n'][0] = -10
        with self.assertRaises(ValueError):
            MODULE.validate_recordings(self.traces, self.summary)

    def test_rejects_incomplete_or_nonconsecutive_pulse(self):
        for history in (self.traces[0]['force_history'][:2040],
                        self.traces[0]['force_history'][:2001] + self.traces[0]['force_history'][2002:]):
            trace = dict(self.traces[0], force_history=history)
            with self.assertRaises(ValueError):
                MODULE.validate_recordings([trace, self.traces[1]], self.summary)

    def test_rejects_wrong_clock_and_stale_peak_or_state_hash(self):
        mutations = (
            lambda t, s: t[0]['force_history'][2001].update(dt_s=.002),
            lambda t, s: t[0]['frames'][63].update(time_s=float('nan')),
            lambda t, s: s['cases'][0].update(force_n=471.11778910882333),
            lambda t, s: s['cases'][0].update(observed_state_hash='stale'),
        )
        for mutate in mutations:
            traces, summary = copy.deepcopy(self.traces), copy.deepcopy(self.summary)
            mutate(traces, summary)
            with self.assertRaises(ValueError):
                MODULE.validate_recordings(traces, summary)

    def test_compiled_manifest_requires_render_human_and_diagnosis_sources(self):
        self.manifest_mock.stop()
        trace = {'compiled_source_sha256': current_manifest()}
        self.assertEqual(len(MODULE.validate_compiled_sources(trace, REPO)), 24)
        for name in ('examples/139_kick_comparison/render.rs',
                     'examples/139_kick_comparison/observation.rs',
                     'examples/139_kick_comparison/diagnostic.rs',
                     'crates/rne_data/src/bus.rs', 'crates/rne_core/src/rng.rs',
                     'assets/fixtures/kick_human/cc0_sport_human.glb'):
            with self.subTest(source=name):
                stale = copy.deepcopy(trace)
                stale['compiled_source_sha256'][name] = '0' * 64
                with self.assertRaisesRegex(ValueError, 'Compiled capture source differs'):
                    MODULE.validate_compiled_sources(stale, REPO)
                missing = copy.deepcopy(trace)
                del missing['compiled_source_sha256'][name]
                with self.assertRaisesRegex(ValueError, 'complete compiled source manifest'):
                    MODULE.validate_compiled_sources(missing, REPO)

    def test_full18_validation_for_32_and_48_ns(self):
        for strength in (32.0, 48.0):
            with tempfile.TemporaryDirectory() as directory:
                path, _, nominal = make_report(Path(directory), strength)
                result = MODULE.validation_metadata(path, nominal)
                self.assertEqual(len(result['cases']), 18)
                self.assertTrue(result['required_cases_passed'])
                self.assertEqual(len(result['negative_cases']), 4)
                self.assertEqual(sum(c['required_recovery'] for c in result['cases']), 12)

    def test_rejects_validation_impulse_changes_even_with_consistent_report(self):
        for name, field, value in (('zero', 'impulse_n_s', 48),
                                   ('late', 'impulse_n_s', 32),
                                   ('rate_500hz', 'dt_s', .001),
                                   ('reverse_early', 'push_start_s', 2.0),
                                   ('front_limit', 'integrated_impulse_world_ns', [0, 0, 48])):
            with self.subTest(name=name, field=field), tempfile.TemporaryDirectory() as directory:
                path, report, nominal = make_report(Path(directory), 48.0)
                trace_path = path.parent / f'go2-{name}.json'
                trace = json.loads(trace_path.read_text())
                trace[field] = value
                next(c for c in report['cases'] if c['robot'] == 'Go2' and c['case'] == name)[field] = value
                trace_path.write_text(json.dumps(trace))
                path.write_text(json.dumps(report))
                with self.assertRaises(ValueError):
                    MODULE.validation_metadata(path, nominal)

    def test_rejects_failed_required_recovery_with_consistent_report(self):
        with tempfile.TemporaryDirectory() as directory:
            path, report, nominal = make_report(Path(directory), 48.0)
            trace_path = path.parent / 'go2-late.json'
            trace = json.loads(trace_path.read_text())
            trace['summary']['recovered'] = False
            case = next(c for c in report['cases'] if c['robot'] == 'Go2' and c['case'] == 'late')
            case['summary']['recovered'] = False
            report['required_cases_passed'] = False
            trace_path.write_text(json.dumps(trace))
            path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, 'A required recovery case failed'):
                MODULE.validation_metadata(path, nominal)

    def test_preserves_validation_scope_and_replay_guards(self):
        with tempfile.TemporaryDirectory() as directory:
            path, report, nominal = make_report(Path(directory), 48.0)
            mutations = (
                lambda r: r.update(robots=['Go2']),
                lambda r: r['cases'].append(r['cases'][0]),
                lambda r: r['cases'].pop(),
                lambda r: r['cases'][0].update(required_recovery=True),
                lambda r: r.update(required_cases_passed=False),
                lambda r: r['cases'][0].update(observed_state_bits_exact_repeat=False),
            )
            for mutate in mutations:
                bad = copy.deepcopy(report)
                mutate(bad)
                path.write_text(json.dumps(bad))
                with self.assertRaises(ValueError):
                    MODULE.validation_metadata(path, nominal)
            path.write_text(json.dumps(report))
            bad_nominal = copy.deepcopy(nominal)
            bad_nominal[0]['controller'] = {'contact_load_regulation_enabled': False}
            with self.assertRaises(ValueError):
                MODULE.validation_metadata(path, bad_nominal)
            bad_nominal = copy.deepcopy(nominal)
            bad_nominal[0]['compiled_source_sha256'] = {'synthetic_fixture': 'stale capture'}
            with self.assertRaises(ValueError):
                MODULE.validation_metadata(path, bad_nominal)
            with self.assertRaises(ValueError):
                MODULE.validation_metadata(None, nominal)


if __name__ == '__main__':
    unittest.main(verbosity=2)
