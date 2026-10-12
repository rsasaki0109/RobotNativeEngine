#!/usr/bin/python3
"""Byte-only local whole-file retention admission; no engine/reader/model calls."""
from datetime import datetime,timezone
import hashlib,json
from pathlib import Path
import stat,zlib
OWN=Path(__file__).resolve().parent;REC=OWN.parent
BUNDLE=REC/'next-motor-visit-placement-publication/prepared-inline-bundle'
def desc(p):
 p=Path(p);r=p.read_bytes();return dict(path=str(p),sha256=hashlib.sha256(r).hexdigest(),bytes=len(r))
def bound(d):assert desc(d['path'])==d,d['path']
manifest_path=BUNDLE/'manifest.json'
assert desc(manifest_path)==dict(path=str(manifest_path),sha256='5f6691486ac0ccf78dcd7570dc26b71386b856039adfec320320f1795cad8950',bytes=443052)
m=json.loads(manifest_path.read_bytes());assert m['kind']=='rne_whole_visit_placement_prepared_inline_retention' and m['schema_version']==1
assert len(m['aliases'])==716 and len(m['payloads'])==396 and len(m['local_only'])==2
payload_map={p['relative_path']:p for p in m['payloads']};assert len(payload_map)==396
stored_bytes=0
for p in m['payloads']:
 rel=Path(p['relative_path']);assert not rel.is_absolute() and '..' not in rel.parts and rel.parts[0]=='f'
 path=BUNDLE/rel;assert stat.S_ISREG(path.lstat().st_mode) and not path.is_symlink()
 assert desc(path)==dict(path=str(path),sha256=p['sha256'],bytes=p['bytes'])
 assert p['bytes']<=1048576;stored_bytes+=p['bytes']
assert stored_bytes==9669691
actual_payloads={str(p.relative_to(BUNDLE)) for p in BUNDLE.rglob('*') if p.is_file() and p!=manifest_path}
assert actual_payloads==set(payload_map)
origins={};used=set();encoded=0
for a in m['aliases']:
 original=a['original'];bound(original);assert original['path'] not in origins;origins[original['path']]=original
 representation=a['representation'];assert payload_map[representation['relative_path']]==representation
 used.add(representation['relative_path']);stored=(BUNDLE/representation['relative_path']).read_bytes()
 if a['encoding']=='whole-file-exact-bytes':assert stored==Path(original['path']).read_bytes()
 else:
  assert a['encoding']=='lossless-single-member-gzip-original-exact-bytes';encoded+=1
  assert original['sha256']=='d7022bb982ab997862da1f3aabbfa4306a6fa568b4707a2b2a9aa0bd9581acfe' and original['bytes']==5447072
  assert representation['sha256']=='4611f3fbeb711e83c749b986b71081c6297b7fd4e4f196b8ea18c4eea3d6c2c0' and representation['bytes']==886291
  assert int.from_bytes(stored[4:8],'little')==0
  state=zlib.decompressobj(16+zlib.MAX_WBITS);raw=state.decompress(stored,original['bytes']+1)
  assert state.eof and state.unused_data==state.unconsumed_tail==b'' and raw==Path(original['path']).read_bytes()
assert used==set(payload_map) and encoded==1
expected_local={'/usr/bin/python3',str(REC/'next-motor-visit-placement-preparation/motor_visit_placement.native')}
assert {d['original']['path'] for d in m['local_only']}==expected_local
for item in m['local_only']:
 original=item['original'];bound(original);assert original['path'] not in origins;origins[original['path']]=original
 assert 'executable' in item['reason']
ledger_path=REC/'next-motor-visit-placement-postreview/whole-file-ledger.json';ledger=json.loads(ledger_path.read_bytes())
assert len(ledger['whole_files'])==702
for d in ledger['whole_files']:assert origins[d['path']]==d
postreview_files=[p for p in sorted((REC/'next-motor-visit-placement-postreview').rglob('*')) if p.is_file()]
for p in postreview_files:assert origins[str(p)]==desc(p)
candidate=json.loads((REC/'next-motor-visit-placement-18/frozen-candidate.json').read_bytes())
assert len(candidate['frozen_inputs'])==508
for d in candidate['frozen_inputs']:assert origins[d['path']]==d
native_raw=[d for path,d in origins.items() if path.startswith(candidate['output_root']+'/') and path.endswith('/raw.jsonl.gz')]
assert len(native_raw)==18
for c in candidate['cases']:
 path=str(Path(candidate['output_root'])/c['label']/'raw.jsonl.gz');assert path in origins
 alias=next(a for a in m['aliases'] if a['original']['path']==path);assert alias['encoding']=='whole-file-exact-bytes'
bound(m['original_audit_restoration_receipt'])
assert origins[m['original_audit_restoration_receipt']['path']]==m['original_audit_restoration_receipt']
result=dict(kind='independent_prepared_inline_whole_bundle_byte_review',status='PASS_LOCAL_BUNDLE_READY_WHOLE_BYTE_RETENTION',schema_version=1,created_utc=datetime.now(timezone.utc).isoformat(),manifest=desc(manifest_path),aliases_verified=716,payloads_verified=396,total_payload_stored_bytes=stored_bytes,threshold_bytes_per_stored_file=1048576,all_payloads_regular_non_symlink_and_at_or_below_threshold=True,all702_ledger_origins_covered=True,current_postreview_whole_files_covered=len(postreview_files),all508_frozen_source_tool_review_origins_covered=True,all18_native_raw_gzip_files_included_whole_exact=True,whole_audit_lossless_restoration_verified=True,compressed_audit_representation_bytes=886291,original_audit_bytes=5447072,only_two_host_compiled_executable_bindings_local_only=sorted(expected_local),raw_dataset_omissions=0,native_reader_math_Git_Release_or_publication_actions=0,public_retention_completed=False,limits=['This is local preparation of an ordinary future PR, not evidence of public download/recovery or merge permission.','All original scientific outcomes and fixed budgets are unchanged; compressed audit retains complete original bytes without splitting/truncation/selection.','The stored compressed representation satisfies current ADR034 size rule; no decompressed-size requirement is imposed.'],issues=[])
out=OWN/'bundle-ready-review.json'
with out.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps(desc(out),sort_keys=True))
