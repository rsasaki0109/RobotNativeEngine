#!/usr/bin/python3
"""Stage whole evidence files locally; never edit Git or publish externally."""
import hashlib
import json
from pathlib import Path
import shutil

OWN = Path(__file__).resolve().parent
REC = OWN.parent
ROOT = Path('/workspace/RobotNativeEngine')
POST = REC / 'next-motor-visit-placement-postreview'
STUDY = REC / 'next-motor-visit-placement-18'
OUT = OWN / 'prepared-inline-bundle'
OUT.mkdir()
(OUT/'f').mkdir()

def d(p):
    return dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest(), bytes=p.stat().st_size)

items = json.loads((POST/'whole-file-ledger.json').read_text())['whole_files'][:]
items += [d(p) for p in sorted(POST.iterdir()) if p.is_file()]
items += [d(p) for p in sorted(OWN.iterdir()) if p.is_file()]
items = sorted({x['path']:x for x in items}.values(), key=lambda x:x['path'])
aliases=[]; local_only=[]; payloads={}
for entry in items:
    original=Path(entry['path'])
    assert d(original)==entry
    if original==STUDY/'actual-cohort-audit.json':
        src=OWN/'actual-cohort-audit.json.gz'
        encoding='lossless-single-member-gzip-original-exact-bytes'
    elif REC not in original.parents and ROOT not in original.parents:
        local_only.append(dict(original=entry,reason='Host/toolchain executable binding; not a scientific raw dataset. Whole hash retained, executable not vendored.'))
        continue
    elif original.suffix=='.native':
        local_only.append(dict(original=entry,reason='Compiled native executable bound by retained build recipe/source/features/hash, not a raw dataset.'))
        continue
    else:
        src=original;encoding='whole-file-exact-bytes'
    value=d(src)
    assert value['bytes']<=1048576, 'Whole file needs external retention: '+str(src)
    digest=value['sha256']
    if digest not in payloads:
        suffix=''.join(src.suffixes)[-32:]
        target=OUT/'f'/f'{len(payloads):06d}{suffix}'
        shutil.copyfile(src,target)
        assert target.read_bytes()==src.read_bytes()
        payloads[digest]=dict(relative_path=str(target.relative_to(OUT)),bytes=value['bytes'],sha256=digest)
    aliases.append(dict(original=entry,representation=payloads[digest],encoding=encoding))
manifest=dict(kind='rne_whole_visit_placement_prepared_inline_retention',schema_version=1,
    aliases=aliases,payloads=list(payloads.values()),local_only=local_only,
    original_audit_restoration_receipt=d(OWN/'whole-audit-compression-receipt.json'),
    scope='Local staging only, whole-file byte deduplication; no splitting/truncation/selection. One entire audit is represented by lossless gzip with exact original restoration binding. Every retained payload is at or below ADR034 stored-file threshold. No Git, Release, network publication or scientific replay performed.')
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'bundle':str(OUT),'aliases':len(aliases),'payloads':len(payloads),'local_only':len(local_only),'payload_bytes':sum(x['bytes'] for x in payloads.values()),'manifest':d(OUT/'manifest.json')}))
