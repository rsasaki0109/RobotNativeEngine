"""Final read-only pre-data metadata/source qualification; no native/reader launch."""
from datetime import datetime, timezone
import ast
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

OWN = Path(__file__).resolve().parent
PREP = OWN.parent
COHORT = PREP.parent / 'next-motor-physical-120'
ROOT = Path('/workspace/RobotNativeEngine')
EXPECTED = 'e81d000fecaaa612aed14b72dfe639679da45448b5f4cfc5d62b4532152a40df'


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True).encode()


def pairs(items):
    out = {}
    for key, value in items:
        if key in out: raise ValueError('duplicate JSON key: ' + key)
        out[key] = value
    return out


def load(path):
    return json.loads(Path(path).read_bytes(), object_pairs_hook=pairs,
                      parse_constant=lambda token: (_ for _ in ()).throw(ValueError(token)))


def descriptor(path):
    path = Path(path); raw = path.read_bytes()
    return dict(path=str(path), bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())


def bound(d):
    assert set(d) == {'path', 'bytes', 'sha256'} and type(d['bytes']) is int and d['bytes'] >= 0
    assert Path(d['path']).is_absolute() and descriptor(d['path']) == d


def save(name, value):
    with (OWN / name).open('xb') as f:
        f.write(json.dumps(value, sort_keys=True, indent=2).encode() + b'\n'); f.flush(); os.fsync(f.fileno())
    fd = os.open(OWN, os.O_RDONLY | os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT,
                                  env={**os.environ, 'GIT_OPTIONAL_LOCKS': '0'})


def main():
    result = dict(status='FAILED_PRE_DATA_INDEPENDENT_QUALIFICATION', collection_admitted=False,
                  native_processes_launched=0, scientific_reader_invocations=0,
                  cargo_commands=0, old_numerical_payload_reads=0, checkout_changes=0, github_calls=0)
    try:
        path = COHORT / 'frozen-candidate.json'
        assert descriptor(path)['sha256'] == EXPECTED
        spec = load(path); frozen = spec['frozen_inputs']
        assert len(frozen) == len({d['path'] for d in frozen}) == 536
        assert [d['path'] for d in frozen] == sorted(d['path'] for d in frozen)
        for d in frozen: bound(d)
        before = [descriptor(d['path']) for d in frozen]
        save('frozen-inputs-before.json', before)
        for d in spec['registration_artifacts'].values(): bound(d)
        for key in ('driver_interpreter', 'checked_driver_source', 'scientific_reader_source'): bound(spec[key])
        assert spec['kind'] == 'frozen_motor_physical20_cohort' and type(spec['schema_version']) is int and spec['schema_version'] == 1
        assert type(spec['collection_worlds_before_registration']) is int and spec['collection_worlds_before_registration'] == 0
        assert spec['environment'] == dict(RUSTFLAGS=None, CARGO_ENCODED_RUSTFLAGS=None, RUSTDOCFLAGS=None, RUST_TEST_THREADS=None)
        assert spec['timeout_seconds'] == 60 and spec['reader_timeout_seconds'] == 600
        assert spec['process_failure_policy'] == 'stop_on_first_failure_no_resume_or_replacement'
        for p in ('output_root', 'master_run_root', 'scientific_reader_output_path', 'driver_run_receipt_path'):
            assert Path(spec[p]).is_absolute() and Path(spec[p]).resolve() == Path(spec[p])
            assert not Path(spec[p]).exists(), 'prospective registered output already exists: ' + p
        assert spec['driver_interpreter']['path'] == '/usr/bin/python3'
        assert [d['path'] for d in frozen if Path(d['path']).name == 'cohort_driver.py'] == [str(PREP / 'driver/cohort_driver.py')]
        protocol = load(spec['registration_artifacts']['protocol']['path'])
        assert descriptor(spec['registration_artifacts']['protocol']['path'])['sha256'] == '305725217acb81eb72adf34d56a072fb2792e5cc797ec4f58baaf7a3bacab136'
        assert canonical(spec['cases']) == canonical(protocol['cases']) and canonical(spec['process_order']) == canonical(protocol['process_order'])
        assert len(spec['cases']) == 20 and len(spec['process_order']) == 120
        assert sum(c['target_pos_rad'] == 0.0 for c in spec['cases']) == 4
        assert protocol['budget'] == dict(fresh_native_processes=120, scientific_reader_invocations=1,
            pilot_processes=0, retries=0, replacement_processes=0, old_reader_invocations=0)
        assert protocol['native_processes_before_protocol'] == protocol['scientific_reader_invocations_before_protocol'] == 0
        for k in ('source_case_template', 'operative_diagnostic_declaration', 'independent_physics_review'): bound(protocol[k])
        bound(protocol['precedence']['collector_earlier_proposal'])
        assert 'Historical preparatory suggestion only' in protocol['precedence']['earlier_envelope_request']
        v2 = load(protocol['operative_diagnostic_declaration']['path']); v1 = load(v2['supersedes_before_any_native_worlds']['path'])
        assert canonical(v1['unchanged_literal_existing_tolerances']) == canonical(v2['unchanged_literal_existing_tolerances'])
        assert v2['outputs']['historical_four_primary_cases_reclassified'] is False
        assert v2['outputs']['production_or_global_candidate_adoption'] is False
        assert 'not conservative propagated f32/input/inertia uncertainty envelopes' in v2['nominal_allowance_scope']
        assignments = {n.targets[0].id: ast.literal_eval(n.value) for n in ast.parse((PREP / 'driver/fixed_contract.py').read_bytes()).body if isinstance(n, ast.Assign)}
        assert canonical(assignments['CASES']) == canonical(spec['cases'])
        assert canonical(assignments['PROCESS_ORDER']) == canonical(spec['process_order'])
        for name in ('fixed_contract.py', 'spec_contract.py', 'strict_io.py'):
            assert (PREP / 'driver' / name).read_bytes() == (PREP / 'reader' / name).read_bytes()
        binding = load(PREP / 'actual-build-binding.json')
        assert canonical(spec['binaries']) == canonical(binding['binaries'])
        assert descriptor(PREP / 'actual-build-binding.json')['sha256'] == '89675062b3e245690538e14248defddbb4df018f4cbcadc75042fd29249025c2'
        assert len(binding['source_descriptors']) == 264 and len(binding['receipt_files']) == 86
        assert load(PREP / 'build-qualification-attempt-3/sources-before.json') == load(PREP / 'build-qualification-attempt-3/sources-after.json') == binding['source_descriptors']
        assert all(c['original_exit'] == 0 for c in binding['original_command_receipts']) and len(binding['original_command_receipts']) == 21
        for compiler in binding['compiler_executables'].values():
            for d in compiler.values(): bound(d)
        for variant, d in spec['binaries'].items():
            bound(d); assert Path(d['path']).read_bytes() == (PREP / ('target_' + variant) / 'release/motor_physical').read_bytes()
            md = load(PREP / 'build-qualification-attempt-3' / (variant + '-metadata-stdout.log'))
            rp = next(p for p in md['packages'] if p['name'] == 'rapier3d')
            rn = next(n for n in md['resolve']['nodes'] if n['id'] == rp['id'])
            assert set(rn['features']) == {'default', 'dim3', 'f32', 'serde-serialize'}
            assert rp['manifest_path'] == str(PREP / ('vendor_base' if variant == 'baseline' else 'vendor_candidate') / 'Cargo.toml')
            log = (PREP / 'build-qualification-attempt-3' / (variant + '-pure-tests-stdout.log')).read_text()
            assert '10 passed; 0 failed; 0 ignored;' in log
        reader = load(PREP / 'reader/handoff.json'); driver = load(PREP / 'driver/handoff.json')
        for d in reader['runtime_closure'] + reader['pure_tests_closure'] + driver['sources']: bound(d)
        for name in ('final_pure_receipt', 'final_pure_stdout', 'final_pure_stderr', 'source_manifest'): bound(reader[name])
        for d in driver['pure_outputs']: bound(d)
        assert reader['pure_test_count'] == 26 and driver['pure_tests'] == 25
        rr = load(reader['final_pure_receipt']['path']); dr = load(driver['pure_original_receipt']['path'])
        assert rr['actual_exit'] == 0 and rr['source_before'] == rr['source_after']
        assert dr['original_exit_code'] == 0 and dr['source_inputs_unchanged'] is True
        assert 'Ran 26 tests' in Path(reader['final_pure_stderr']['path']).read_text()
        assert 'Ran 25 tests' in Path(dr['stderr']['path']).read_text()
        for p, sha in rr['source_after'].items(): assert descriptor(p)['sha256'] == sha
        for d in dr['sources']: bound(d)
        for attempt in (1, 2):
            assert load(PREP / f'build-qualification-attempt-{attempt}/baseline-strict-clippy-actual.json')['original_exit'] == 101
        freeze = load(COHORT / 'freeze-original.json')
        assert freeze['original_exit'] == 0 and freeze['native_worlds'] == freeze['scientific_reader_invocations'] == 0
        bound(freeze['stdout']); bound(freeze['stderr'])
        assert load(freeze['stdout']['path'])['candidate'] == descriptor(path)
        head = git('rev-parse', 'HEAD').decode().strip(); tree = git('rev-parse', 'HEAD^{tree}').decode().strip()
        assert head == binding['root_head'] == '423a3fd7f44128aa440afe47f937039de504f275' and git('status', '--porcelain=v1') == b''
        tracked = [p.decode() for p in git('ls-files', '-z', 'third_party/rapier3d').split(b'\0') if p]
        assert len(tracked) == 123
        for p in tracked:
            assert (ROOT / p).read_bytes() == (PREP / 'vendor_base' / Path(p).relative_to('third_party/rapier3d')).read_bytes()
        after = [descriptor(d['path']) for d in frozen]
        assert before == after
        save('frozen-inputs-after.json', after)
        result.update(status='PASS_PRE_DATA_PHYSICAL20_SOURCE_PROTOCOL_DRIVER_READER', collection_admitted=True,
            reviewed_candidate_sha256=EXPECTED, binaries=spec['binaries'],
            frozen_inputs_sha256=hashlib.sha256(canonical(frozen)).hexdigest(),
            reviewed_candidate=descriptor(path), frozen_input_count=536, frozen_inputs_current_before_after_exact=True,
            actual_build_source_count=264, actual_build_receipt_count=86, actual_build_command_exits=21,
            actual_final_pure_test_executions={'collector_baseline': 10, 'collector_candidate': 10, 'driver': 25, 'reader': 26},
            final_tests_are_execution_counts_not_unique_tests=True,
            operative_protocol=spec['registration_artifacts']['protocol'], operative_declaration=protocol['operative_diagnostic_declaration'],
            prior_doc_conflict_resolved_by_explicit_predata_precedence=True,
            root_head=head, root_tree=tree, root_clean=True, native_output_claims_absent=True,
            guard_before=descriptor(OWN / 'frozen-inputs-before.json'), guard_after=descriptor(OWN / 'frozen-inputs-after.json'),
            actual_freeze_original=descriptor(COHORT / 'freeze-original.json'),
            reviewed_source_methods=[
                'Literal vendor intervention review:123 files each, one exact CRLF-preserving ForceBased inverse-(w+g) branch; AccelerationBased and contact/limit/solve/order paths unchanged.',
                'Source fixed plant:one centered ball of mass0.5/2 kg and radius0.1m, fixed parent/coincident X frames, zero gravity/damping/armature/velocity, N1/PGS1/S2 and one interval; state assertions are checked independently from proposed SI references.',
                'Actual locked build receipts/metadata/features/compiler executable and two native-copy hashes verified; no binary invocation.',
                'Nominal rational clipped backward-Euler reference and unchanged deltaI/deltaP with independently declared velocity/coordinate/torque allowances; every quantity is separate, not an uncertainty propagation inference.',
                'Reader requires exact mass words, complete typed ordered checkpoint/telemetry/phase/cache schema and normal-zero-only native arithmetic; public inertia and cached inverse mass stay distinct.',
                'Whole single-member gzip header/CRC/EOF/no trailing bytes, duplicate-key/nonfinite/JSON-float/type/width gates; freshON completed record literal equality and within-build captured ON/fresh/OFF checkpoints, OFF null observers.',
                'Exclusive120 preclaims and ancestor/root fsync before first Popen; each process guard/original argv/exit and independent conservative launch/storage accounting; first failure stops/no resume/retry/replacement.',
                'Checked coordinator owns a group inherited by native children; owned TERM/KILL interruption cleanup; root checked CLI and inner original exits/stdout-versus-disk receipts required before one separately claimed reader.',
                'Two final pure suites cover synthetic error paths only, with original exit0 and source guards; authentic prior failed preparation receipts remain frozen. No favorable baseline/candidate result is required.'
            ],
            limits=[
                'Admission is solely for the separately frozen120-process/one-reader diagnostic; no production behavior change, promotion, merge or external publication authorization.',
                'No earlier numerical dataset/report/raw/scientific reader was accessed or reclassified; historical four primary gates/global-candidate rejection remain retained.',
                'Runtime rounding/MXCSR/FTZ/DAZ, full hermetic compiler/registry/runtime closure, private solver/cache/mass-LU construction, independent opaque-layout decoder and general multislice f64 aggregation remain unqualified.',
                'No physical outcome, motor accuracy, coupled-contact cause, hardware validation or maturity increase is established before these once-only observations.',
                'Root must create the noncircular descriptor wrapper after this receipt and validate default(no-launch) mode before the sole explicitly flagged launch. Runtime allowlist is four NULL values.',
                'Keep all complete/partial original files regardless of diagnostic labels. Any ADR034 external artifact requires separate concrete publication authorization; this receipt grants none.'
            ])
    except BaseException as failure:
        result['error'] = type(failure).__name__ + ': ' + str(failure)
        raise
    finally:
        result['completed_utc'] = datetime.now(timezone.utc).isoformat()
        result['review_source'] = descriptor(Path(__file__))
        save('admission.json', result)
        print(json.dumps({'status': result['status'], 'receipt': descriptor(OWN / 'admission.json')}), flush=True)


if __name__ == '__main__':
    main()
