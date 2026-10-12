"""Bind actual independent pre-data reviews into the final once-only spec."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys

G = Path('/workspace/.rne-kick-demo/single-joint-motor-calibration/next-experiment/generation-5')
F = G / 'freeze'

def record(path):
    path = path.resolve()
    with path.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest()
    return dict(path=str(path), bytes=path.stat().st_size, sha256=digest)

def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try: os.fsync(descriptor)
    finally: os.close(descriptor)

reviews = [Path(sys.argv[1]), G / 'predata-storage-review/final-predata-storage-review.json', G / 'reader/concrete-freeze-review-v2.json']
for path in reviews:
    data = json.loads(path.read_text())
    if not data['status'].startswith('PASS'):
        raise ValueError('independent review did not pass: ' + str(path))
if (G / 'collection').exists():
    raise ValueError('collection already exists; no new spec or collection allowed')
combined = dict(status='PASS_ALL_THREE_INDEPENDENT_PRE_DATA_SCOPES', reviews=[record(path) for path in reviews], cohort_id='generation5-independent-all-motor-stage', authoritative_worlds_of_this_cohort_before_freeze=0, registered_utc=datetime.now(timezone.utc).isoformat(), scope='Internal source-only review, pure tests and concrete artifact admission; neither numerical measurement nor independent external-use/hardware validation')
write(F / 'independent-readonly-review.json', combined)
protocol = json.loads((F / 'frozen-protocol-v2.json').read_text())
spec = json.loads((G / 'driver/spec-template.json').read_text())
spec.update(status='FROZEN_BEFORE_GENERATION5_COLLECTION', cohort_id=protocol['cohort_id'], registered_utc=datetime.now(timezone.utc).isoformat(), protocol=record(F / 'frozen-protocol-v2.json'), reader_artifacts=protocol['reader_artifacts'], reader_pure_test_receipt=record(G / 'reader/actual-pure-test-receipt.json'), independent_readonly_review=record(F / 'independent-readonly-review.json'), source_guard_manifest=record(G / 'preparation/baseline-canonical-source-manifest.json'), driver_artifacts=[record(G / 'driver' / name) for name in ['durable_common.py','collect_once.py','prepare_archive.py']], bindings={build: record(F / (build + '-source-binary-binding-v2.json')) for build in ['baseline','rejected_inverse_w_plus_g']}, preflight_gates=dict(reader_pure_tests='PASS', independent_readonly_review='PASS'))
write(F / 'frozen-driver-spec.json', spec)
sys.path.insert(0,str(G / 'driver'))
from durable_common import validate_spec
actual, bindings, source = validate_spec(record(F / 'frozen-driver-spec.json'))
write(F / 'actual-final-driver-spec-preflight.json', dict(status='PASS', actual_exit_code=0, worlds=0, spec=record(F / 'frozen-driver-spec.json'), registered_utc=datetime.now(timezone.utc).isoformat(), fixed_cells=len(actual['fixed_cells']), source_paths=len(source['files'])))
inputs = [record(F / 'frozen-driver-spec.json'), spec['protocol'], *spec['reader_artifacts'], *spec['driver_artifacts'], *spec['bindings'].values(), *[binding['binary'] for binding in bindings.values()], record(G / 'preparation/execute_registered_commands.py')]
write(F / 'collection-command-inputs.json', inputs)
print(json.dumps({'spec':record(F / 'frozen-driver-spec.json'), 'pure_preflight':'PASS', 'worlds':0}))
