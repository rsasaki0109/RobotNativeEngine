"""Shared prospective fixed18 source/binary admission; no native or data calls."""
from datetime import datetime, timezone
import hashlib
from pathlib import Path
import strict_io
from strict_io import bound, canonical, integer, keys, loads, need

PROFILES = {
    'split_1_2': dict(pgs_passes=1, stabilization_passes=2),
    'early_1_0': dict(pgs_passes=1, stabilization_passes=0),
    'early_3_0': dict(pgs_passes=3, stabilization_passes=0),
}
CASES = [dict(profile=profile, target=target, observer=observer,
              label=profile + '_' + target + '_' + mode)
         for profile in PROFILES
         for target in ('negative', 'positive')
         for mode, observer in [('on', 'on'), ('fresh', 'on'), ('off', 'off')]]
CANDIDATE_KEYS = ('kind schema_version registered_utc collection_worlds_before_registration '
                  'cases binary cwd output_root frozen_inputs registration_artifacts source_scope '
                  'environment timeout_seconds process_failure_policy driver_run_receipt_path '
                  'driver_interpreter checked_driver_source')
REGISTRATION_ROLES = 'protocol collector_source_manifest compiler_feature_receipt producer_review pure_tests_review'


def same(a, b, message):
    need(canonical(a) == canonical(b), message)


def persistent(path, message):
    p = Path(path)
    need(type(path) is str and p.is_absolute() and p.resolve() == p and
         p != Path('/workspace') and Path('/workspace') in p.parents, message)
    return p


def driver_source(spec):
    matches = [item for item in spec['frozen_inputs']
               if Path(item['path']).name == 'cohort_driver.py']
    need(len(matches) == 1, 'exactly one actual driver source descriptor required')
    return matches[0]


def load_spec(path, expected_sha):
    need(type(expected_sha) is str and len(expected_sha) == 64 and
         all(c in '0123456789abcdef' for c in expected_sha), 'invalid external wrapper SHA')
    wrapper_path = persistent(str(Path(path)), 'wrapper must be persistent canonical /workspace path')
    need(hashlib.sha256(wrapper_path.read_bytes()).hexdigest() == expected_sha,
         'external wrapper SHA mismatch')
    wrapper = loads(wrapper_path.read_bytes())
    keys(wrapper, 'kind schema_version candidate admission')
    need(wrapper['kind'] == 'admitted_motor_visit_placement_cohort' and
         type(wrapper['schema_version']) is int and wrapper['schema_version'] == 1,
         'invalid final visit-placement admission wrapper')
    bound(wrapper['candidate']); bound(wrapper['admission'])
    spec = loads(Path(wrapper['candidate']['path']).read_bytes())
    keys(spec, CANDIDATE_KEYS)
    need(spec['kind'] == 'frozen_motor_visit_placement_cohort' and
         type(spec['schema_version']) is int and spec['schema_version'] == 1 and
         type(spec['collection_worlds_before_registration']) is int and
         spec['collection_worlds_before_registration'] == 0, 'not prospectively registered')
    same(spec['cases'], CASES, 'case order must be fixed separate eighteen')
    dt = datetime.fromisoformat(spec['registered_utc'])
    need(dt.tzinfo is not None and dt <= datetime.now(timezone.utc), 'bad registration timestamp')
    for key in ('cwd', 'output_root'):
        persistent(spec[key], 'absolute canonical persistent registered path required')
    need(Path(spec['cwd']).is_dir(), 'registered cwd missing')
    receipt = persistent(spec['driver_run_receipt_path'], 'invalid checked-driver receipt path')
    need(receipt.name == 'actual-command.json', 'checked-driver receipt filename differs')
    need(spec['process_failure_policy'] == 'stop_on_first_failure_no_resume_or_replacement',
         'undeclared process failure policy')
    integer(spec['timeout_seconds'])
    need(1 <= spec['timeout_seconds'] <= 120, 'timeout must be bounded')
    need(type(spec['source_scope']) is str and spec['source_scope'], 'explicit source scope required')
    keys(spec['environment'], 'RUSTFLAGS CARGO_ENCODED_RUSTFLAGS RUSTDOCFLAGS RUST_TEST_THREADS')
    for value in spec['environment'].values():
        need(value is None or type(value) is str, 'invalid non-secret environment binding')
    bound(spec['binary'])
    persistent(spec['binary']['path'], 'binary copy must be persistent /workspace')
    bound(spec['driver_interpreter'])
    need(spec['driver_interpreter']['path'] == '/usr/bin/python3', 'pin /usr/bin/python3')
    need(type(spec['frozen_inputs']) is list and spec['frozen_inputs'], 'missing source/tool closure')
    paths = []
    for item in spec['frozen_inputs']:
        bound(item); paths.append(item['path'])
    need(len(paths) == len(set(paths)), 'duplicate frozen path')
    for module_path in (Path(__file__).resolve(), Path(strict_io.__file__).resolve()):
        need(str(module_path) in paths, 'actually loaded contract/helper source is not frozen')
    driver = driver_source(spec)
    persistent(driver['path'], 'driver must be persistent /workspace')
    bound(spec['checked_driver_source'])
    need(spec['checked_driver_source']['path'] == str(Path(driver['path']).parent / 'checked_driver.py'),
         'checked-driver source must match registered actual driver directory')
    need(spec['checked_driver_source']['path'] in paths, 'checked-driver source not in frozen closure')
    keys(spec['registration_artifacts'], REGISTRATION_ROLES)
    for artifact in spec['registration_artifacts'].values(): bound(artifact)
    review = loads(Path(wrapper['admission']['path']).read_bytes())
    need(review.get('status') == 'PASS_PRE_DATA_SOURCE_PROTOCOL_AND_DRIVER' and
         review.get('reviewed_candidate_sha256') == wrapper['candidate']['sha256'] and
         canonical(review.get('binary')) == canonical(spec['binary']) and
         review.get('frozen_inputs_sha256') == hashlib.sha256(canonical(spec['frozen_inputs'])).hexdigest(),
         'independent admission does not bind candidate/source/binary')
    spec['_authenticated_wrapper_sha256'] = expected_sha
    spec['_authenticated_candidate_sha256'] = wrapper['candidate']['sha256']
    spec['_authenticated_wrapper_path'] = str(wrapper_path)
    spec['_cohort_driver_source_path'] = driver['path']
    return spec


def expected_driver_command(spec):
    return [spec['driver_interpreter']['path'], '-B', driver_source(spec)['path'],
            '--spec', spec['_authenticated_wrapper_path'], '--spec-sha256',
            spec['_authenticated_wrapper_sha256'], '--launch-native']
