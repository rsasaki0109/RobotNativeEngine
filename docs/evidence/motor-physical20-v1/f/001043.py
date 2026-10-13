"""Exactly one authenticated metadata-only default driver invocation."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess

OWN = Path(__file__).resolve().parent
PREP = OWN.parent
STUDY = PREP.parent / 'next-motor-physical-120'
ROOT = Path('/workspace/RobotNativeEngine')
SPEC = STUDY / 'admitted-spec.json'
SHA = '6755bb5d73294fe67353df7ff5ab555314ecd6d425e6343f411132942d35b20c'
OUT = OWN / 'preflight-default-once'


def descriptor(path):
    path = Path(path); raw = path.read_bytes()
    return {'path': str(path), 'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()}


def save(name, value):
    with (OUT / name).open('xb') as f:
        f.write(json.dumps(value, sort_keys=True, indent=2).encode() + b'\n'); f.flush(); os.fsync(f.fileno())
    fd = os.open(OUT, os.O_RDONLY | os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)


def load(path): return json.loads(Path(path).read_bytes())


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT,
                                  env={**os.environ, 'GIT_OPTIONAL_LOCKS': '0'})


def main():
    OUT.mkdir(exist_ok=False)
    fd = os.open(OUT.parent, os.O_RDONLY | os.O_DIRECTORY)
    try: os.fsync(fd)
    finally: os.close(fd)
    receipt = {'kind': 'actual_once_default_no_native_cli_preflight', 'original_exit': None,
               'status': 'FAILED_OR_UNQUALIFIED', 'started_utc': datetime.now(timezone.utc).isoformat(),
               'native_processes_started': 0, 'scientific_reader_invocations': 0, 'original_default_cli_invocations': 0,
               'retries': 0, 'source_mutations': 0, 'github_calls': 0}
    try:
        assert descriptor(SPEC) == {'path': str(SPEC), 'bytes': 561, 'sha256': SHA}
        wrapper = load(SPEC)
        assert set(wrapper) == {'kind', 'schema_version', 'candidate', 'admission'}
        assert wrapper['kind'] == 'admitted_motor_physical20_cohort' and wrapper['schema_version'] == 1
        assert descriptor(wrapper['candidate']['path']) == wrapper['candidate']
        assert descriptor(wrapper['admission']['path']) == wrapper['admission'] == descriptor(OWN / 'admission.json')
        candidate = load(wrapper['candidate']['path']); admission = load(wrapper['admission']['path'])
        assert admission['status'] == 'PASS_PRE_DATA_PHYSICAL20_SOURCE_PROTOCOL_DRIVER_READER'
        assert admission['reviewed_candidate_sha256'] == wrapper['candidate']['sha256']
        assert admission['binaries'] == candidate['binaries']
        paths = [d['path'] for d in candidate['frozen_inputs']]
        paths += [str(SPEC), wrapper['candidate']['path'], wrapper['admission']['path']]
        paths += [d['path'] for d in candidate['binaries'].values()] + [candidate['driver_interpreter']['path']]
        before = [descriptor(p) for p in paths]
        assert all(descriptor(d['path']) == d for d in candidate['frozen_inputs'])
        for name, value in candidate['environment'].items(): assert os.environ.get(name) == value
        output_paths = [candidate[k] for k in ('output_root', 'master_run_root', 'scientific_reader_output_path', 'driver_run_receipt_path')]
        assert all(not Path(p).exists() for p in output_paths)
        head_before = git('rev-parse', 'HEAD').decode().strip()
        tree_before = git('rev-parse', 'HEAD^{tree}').decode().strip()
        assert head_before == admission['root_head'] and tree_before == admission['root_tree'] and git('status', '--porcelain=v1') == b''
        save('inputs-before.json', before)
        command = [candidate['driver_interpreter']['path'], '-B', str(PREP / 'driver/run_registered_once.py'),
                   '--spec', str(SPEC), '--spec-sha256', SHA]
        assert '--launch-native' not in command
        receipt['argv'] = command; receipt['cwd'] = candidate['cwd']
        save('original-command-claim.json', dict(receipt, wrapper=descriptor(SPEC)))
        with (OUT / 'stdout.log').open('xb') as out, (OUT / 'stderr.log').open('xb') as err:
            receipt['original_default_cli_invocations'] = 1
            process = subprocess.run(command, cwd=candidate['cwd'], env=os.environ.copy(),
                                     stdout=out, stderr=err, check=False, timeout=45)
            receipt['original_exit'] = process.returncode
            for stream in (out, err): stream.flush(); os.fsync(stream.fileno())
        receipt['stdout'] = descriptor(OUT / 'stdout.log'); receipt['stderr'] = descriptor(OUT / 'stderr.log')
        assert receipt['original_exit'] == 0
        value = load(OUT / 'stdout.log')
        assert value == {'status': 'validated_no_process_started', 'native_or_reader_processes_started': 0}
        assert receipt['stderr']['bytes'] == 0
        after = [descriptor(p) for p in paths]
        save('inputs-after.json', after); assert before == after
        assert all(not Path(p).exists() for p in output_paths)
        assert git('rev-parse', 'HEAD').decode().strip() == head_before
        assert git('rev-parse', 'HEAD^{tree}').decode().strip() == tree_before
        assert git('status', '--porcelain=v1') == b''
        receipt.update(status='PASS_AUTHENTICATED_DEFAULT_PREFLIGHT_NO_NATIVE_OR_READER_OR_CLAIMS',
                       source_and_wrapper_and_admission_inputs_unchanged=True,
                       native_outputs_and_claims_still_absent=True, root_head=head_before, root_tree=tree_before,
                       root_checkout_clean=True, wrapper=descriptor(SPEC), admission=descriptor(OWN / 'admission.json'))
    except BaseException as failure:
        receipt['error'] = type(failure).__name__ + ': ' + str(failure)
        raise
    finally:
        receipt['completed_utc'] = datetime.now(timezone.utc).isoformat()
        receipt['executor_source'] = descriptor(Path(__file__))
        save('actual-command.json', receipt)
        print(json.dumps({'status': receipt['status'], 'receipt': descriptor(OUT / 'actual-command.json')}), flush=True)


if __name__ == '__main__': main()
