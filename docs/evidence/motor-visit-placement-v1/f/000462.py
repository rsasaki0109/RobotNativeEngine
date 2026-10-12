#!/usr/bin/python3
"""Retain original new portable-check command exits; no scientific entrypoints."""
from datetime import datetime,timezone
import hashlib,json
from pathlib import Path
import shutil,subprocess,time
OWN=Path(__file__).resolve().parent
OUT=OWN/'validation-attempt-1';OUT.mkdir()
def desc(p):
 p=Path(p);r=p.read_bytes();return dict(path=str(p),sha256=hashlib.sha256(r).hexdigest(),bytes=len(r))
sources=[OWN/n for n in ('verify_literals.py','test_verify_literals.py','README.md','run_pure_checks.py')]
before=[desc(p) for p in sources]
(OUT/'source-before.json').write_text(json.dumps(before,indent=2)+'\n')
def command(name,argv,cwd):
 start=time.time_ns()
 with (OUT/(name+'-stdout.log')).open('xb') as out,(OUT/(name+'-stderr.log')).open('xb') as err:
  p=subprocess.run(argv,cwd=cwd,stdout=out,stderr=err,check=False)
 result=dict(kind='original_new_portable_byte_check_command',command=argv,cwd=str(cwd),original_exit_code=p.returncode,start_unix_ns=start,end_unix_ns=time.time_ns(),stdout=desc(OUT/(name+'-stdout.log')),stderr=desc(OUT/(name+'-stderr.log')),native_scientific_reader_or_archived_helper_invocations=0)
 (OUT/(name+'-actual-command.json')).write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(dict(command=name,original_exit_code=p.returncode)),flush=True)
 if p.returncode:
  print((OUT/(name+'-stderr.log')).read_text())
  raise SystemExit(p.returncode)
 return result
pure=command('pure-tests',['/usr/bin/python3','-B',str(OWN/'test_verify_literals.py')],OWN)
source=OWN.parent/'next-visit-pr-preparation/patch-tree/docs/evidence/motor-visit-placement-v1'
copy=OWN/'final-prepared-copy'
original_files=[desc(p) for p in sorted(source.rglob('*')) if p.is_file()]
shutil.copytree(source,copy)
(OUT/'actual-copy-original-files.json').write_text(json.dumps(original_files,indent=2)+'\n')
for d in original_files:
 rel=Path(d['path']).relative_to(source)
 assert (copy/rel).read_bytes()==Path(d['path']).read_bytes()
(copy/'verify_literals.py').write_bytes((OWN/'verify_literals.py').read_bytes())
actual=command('actual-prepared-copy',['/usr/bin/python3','-B',str(copy/'verify_literals.py'),'--root',str(copy)],OWN)
after=[desc(p) for p in sources]
assert before==after
(OUT/'source-after.json').write_text(json.dumps(after,indent=2)+'\n')
assert original_files==[desc(p) for p in sorted(source.rglob('*')) if p.is_file()]
result=dict(kind='new_portable_verifier_original_pure_and_actual_copy_checks',created_utc=datetime.now(timezone.utc).isoformat(),status='PASS_NEW_PURE_TESTS_AND_ACTUAL_PORTABLE_LITERAL_COPY',pure_tests=pure,actual_prepared_copy=actual,source_inputs_unchanged=True,sources=after,original_copy_inputs_unchanged=True,source_copy_manifest=desc(source/'manifest.json'),owned_copy_manifest=desc(copy/'manifest.json'),actual_scoped_result=json.loads((OUT/'actual-prepared-copy-stdout.log').read_text()),new_native_or_scientific_reader_invocations=0,outputs_stored_outside_evidence=True)
(OUT/'checks-complete.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(completion=desc(OUT/'checks-complete.json')),sort_keys=True))
