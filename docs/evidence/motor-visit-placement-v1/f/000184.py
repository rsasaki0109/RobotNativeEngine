#!/usr/bin/python3
"""Verify lossless WHOLE audit gzip and literal stored-file ADR eligibility only."""
from datetime import datetime,timezone
import hashlib,json
from pathlib import Path
import stat,zlib
OWN=Path(__file__).resolve().parent;REC=OWN.parent
original=REC/'next-motor-visit-placement-18/actual-cohort-audit.json'
compressed=REC/'next-motor-visit-placement-publication/actual-cohort-audit.json.gz'
receipt_path=compressed.parent/'whole-audit-compression-receipt.json'
adr=Path('/workspace/RobotNativeEngine/docs/adr/034-external-large-evidence-retention.md')
implementation=Path('/workspace/RobotNativeEngine/xtask/src/docs_evidence_retention.rs')
def descriptor(p):
    r=p.read_bytes();return dict(path=str(p),sha256=hashlib.sha256(r).hexdigest(),bytes=len(r))
r=original.read_bytes();g=compressed.read_bytes()
assert descriptor(original)==dict(path=str(original),sha256='d7022bb982ab997862da1f3aabbfa4306a6fa568b4707a2b2a9aa0bd9581acfe',bytes=5447072)
assert descriptor(compressed)==dict(path=str(compressed),sha256='4611f3fbeb711e83c749b986b71081c6297b7fd4e4f196b8ea18c4eea3d6c2c0',bytes=886291)
assert g[:3]==bytes.fromhex('1f8b08') and int.from_bytes(g[4:8],'little')==0
state=zlib.decompressobj(16+zlib.MAX_WBITS)
restored=state.decompress(g,len(r)+1)
assert state.eof and state.unused_data==b'' and state.unconsumed_tail==b''
assert restored==r and len(restored)==5447072 and hashlib.sha256(restored).hexdigest()==descriptor(original)['sha256']
assert int.from_bytes(g[-4:],'little')==len(r)
assert stat.S_ISREG(compressed.lstat().st_mode) and not compressed.is_symlink()
root_receipt=json.loads(receipt_path.read_bytes())
assert root_receipt['original']==descriptor(original) and root_receipt['compressed_representation']==descriptor(compressed)
assert root_receipt['gzip_members']==1 and root_receipt['gzip_mtime']==0 and root_receipt['roundtrip_original_bytes_exact']
adr_text=adr.read_text();source=implementation.read_text()
assert 'a file added under `docs/evidence` that exceeds 1 MiB' in adr_text
assert 'A file at or below\n1 MiB never needs an allowlist entry' in adr_text
assert 'let size = metadata.len();' in source and 'if size <= MAX_INLINE_EVIDENCE_BYTES {\n            continue;\n        }' in source
assert 'const MAX_INLINE_EVIDENCE_BYTES: u64 = 1024 * 1024;' in source
assert len(g)<=1048576
result=dict(kind='independent_lossless_whole_audit_compressed_representation_review',schema_version=1,status='PASS_WHOLE_BYTES_ROUNDTRIP_AND_LITERAL_ADR034_STORED_FILE_ELIGIBILITY',created_utc=datetime.now(timezone.utc).isoformat(),original=descriptor(original),compressed_representation=descriptor(compressed),root_preparation_receipt=descriptor(receipt_path),policy_sources=[descriptor(adr),descriptor(implementation)],gzip_members=1,gzip_mtime=0,complete_single_member_no_trailing_data=True,decompressed_byte_length=5447072,decompressed_original_sha256=descriptor(original)['sha256'],bytewise_entire_original_identical=True,rational_digits_whitespace_fields_rows_preserved_by_byte_identity=True,stored_file_bytes=886291,inline_threshold_bytes=1048576,literal_eligibility='A regular non-symlink whole .json.gz file of886291 stored bytes can be added under docs/evidence without a pointer/allowlist under current ADR034 and metadata.len-based enforcement. The policy does not impose a decompressed-size threshold.',new_native_worlds_or_scientific_reader_or_math_reinterpretations=0,Git_Release_publication_actions=0,public_retention_completed=False,existing_study_README_not_modified=True,limits=['Only local lossless representation preparation is verified. Ordinary future PR must retain these exact compressed bytes, original SHA/length and documented decompression/recovery; public retrieval remains unverified.','This review does not authorize #410 merge, production adoption, or a Release publication.'],issues=[])
out=OWN/'whole-audit-compression-review.json'
with out.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps(descriptor(out),sort_keys=True))
