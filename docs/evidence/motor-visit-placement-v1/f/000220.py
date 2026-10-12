#!/usr/bin/python3
"""Bind the final source/reader/driver closure before the first new native run."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path

OWN = Path(__file__).resolve().parent
REC = OWN.parent
DRIVER = REC / 'next-motor-visit-placement-driver'
READER = REC / 'next-motor-visit-placement-reader'
REVIEWS = REC / 'next-motor-stabilization-source-study'
STUDY = REC / 'next-motor-visit-placement-18'

def d(p):
    p = Path(p)
    return dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest(), bytes=p.stat().st_size)

def save(p, x):
    with p.open('x') as f:
        json.dump(x, f, indent=2); f.write('\n'); f.flush(); os.fsync(f.fileno())
    fd = os.open(p.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)

assert not (STUDY / 'collection').exists()
assert not (STUDY / 'collect-invocation').exists()
reader_handoff = json.loads((READER / 'handoff.json').read_text())
driver_handoff = json.loads((DRIVER / 'handoff.json').read_text())
manifest = json.loads((OWN / 'selected-source-manifest.json').read_text())
sources = manifest['source_files'][:]
support = [OWN / n for n in ('actual-build-binding.json', 'actual-toolchain-executable-qualification.json', 'fixed-protocol.json', 'selected-source-manifest.json', 'unchanged-vendor-copy.json', 'collector-source-delta.patch', 'freeze_actual_build.py', 'prepare_protocol_sources.py', 'validate_and_build.py', 'run_registered_once.py', 'freeze_candidate.py')]
support += [REVIEWS / n for n in ('producer_review.json', 'pure_tests_review.json', 'source-order-and-conditional-predictions.md')]
support += [REC / 'prepare_visit_placement.py']
support += [REC / 'next-motor-correction-workflow-plan' / n for n in ('outer-launcher-source-review.json', 'selected-diagnostic18-workflow-plan.md')]
support += [REC / 'next-motor-actuation-reference-study' / n for n in ('PREREGISTRATION.md', 'preregistration-fields-template.json')]
for folder in (DRIVER, READER):
    support += [p for p in sorted(folder.iterdir()) if p.is_file() and p.suffix in ('.py', '.json', '.md')]
for folder in (OWN/'actual-build-binding', OWN/'actual-toolchain-executable-qualification', OWN/'validation'):
    support += [p for p in sorted(folder.iterdir()) if p.is_file()]
# Pure receipts and original logs are included whole, independently of mock data.
for folder in (DRIVER, READER):
    for p in sorted(folder.rglob('*')):
        if p.is_file() and (p.name in ('actual-pure-tests.json', 'receipt.json', 'actual-pure-command-completion.json') or (p.name in ('stdout.log', 'stderr.log') and ('pure-attempt' in str(p.parent) or 'synthetic-tests' in str(p.parent)))):
            support.append(p)
sources += [d(p) for p in support]
sources = sorted({x['path']: x for x in sources}.values(), key=lambda x:x['path'])
for entry in sources:
    assert d(entry['path']) == entry
candidate = json.loads((DRIVER / 'candidate-template.json').read_text())
candidate.update(registered_utc=datetime.now(timezone.utc).isoformat(),
    binary=d(OWN/'motor_visit_placement.native'),
    driver_interpreter=d(Path('/usr/bin/python3')),
    checked_driver_source=d(DRIVER/'checked_driver.py'),
    frozen_inputs=sources,
    source_scope=manifest['scope'] + ' Actual loaded reader/driver/source modules and whole preparation/pure-review receipts are also frozen. Runtime FP/full hidden state remain unmeasured.',
    environment={k:os.environ.get(k) for k in ('RUSTFLAGS','CARGO_ENCODED_RUSTFLAGS','RUSTDOCFLAGS','RUST_TEST_THREADS')},
    registration_artifacts=dict(protocol=d(OWN/'fixed-protocol.json'),
        collector_source_manifest=d(OWN/'selected-source-manifest.json'),
        compiler_feature_receipt=d(OWN/'actual-build-binding.json'),
        producer_review=d(REVIEWS/'producer_review.json'),
        pure_tests_review=d(REVIEWS/'pure_tests_review.json')))
assert candidate['cases'] == json.loads((OWN/'fixed-protocol.json').read_text())['cases']
STUDY.mkdir(exist_ok=False)
fd=os.open(STUDY.parent,os.O_RDONLY|os.O_DIRECTORY)
try:
    os.fsync(fd)
finally:
    os.close(fd)
save(STUDY/'frozen-candidate.json',candidate)
print(json.dumps({'candidate':d(STUDY/'frozen-candidate.json'),'frozen_inputs':len(sources),'native_worlds':0}))
