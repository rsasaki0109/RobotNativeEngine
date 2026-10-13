"""Retain new stub-only test command/source guards and every genuine failure."""
from datetime import datetime,timezone
import hashlib,json
from pathlib import Path
import subprocess,time
OWN=Path(__file__).resolve().parent
stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');OUT=OWN/'validation'/stamp;OUT.mkdir(parents=True)
def desc(p):
 p=Path(p);r=p.read_bytes();return dict(path=str(p),sha256=hashlib.sha256(r).hexdigest(),bytes=len(r))
sources=[p for p in sorted(OWN.iterdir()) if p.is_file() and p.suffix=='.py']
before=[desc(p) for p in sources]
(OUT/'sources-before.json').write_text(json.dumps(before,indent=2)+'\n')
argv=['/usr/bin/python3','-B',str(OWN/'test_driver_pure.py')];start=time.time_ns()
with (OUT/'stdout.log').open('xb') as out,(OUT/'stderr.log').open('xb') as err:
 p=subprocess.run(argv,cwd=OWN,stdout=out,stderr=err,check=False)
after=[desc(p) for p in sources]
(OUT/'sources-after.json').write_text(json.dumps(after,indent=2)+'\n')
receipt=dict(kind='NEW120_DRIVER_PURE_STUB_TEST_ORIGINAL_COMMAND',argv=argv,cwd=str(OWN),original_exit_code=p.returncode,
 start_unix_ns=start,end_unix_ns=time.time_ns(),source_inputs_unchanged=before==after,sources=after,
 stdout=desc(OUT/'stdout.log'),stderr=desc(OUT/'stderr.log'),native_or_scientific_reader_processes=0,real_process_signals=0,old_scientific_data_reads=0,
 test_scope='Subprocess.Popen and os.killpg patched globally to prohibit actual driver/reader execution/signals. All exercised processes are injected stubs and every invented artifact is retained.')
(OUT/'actual-command.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(dict(receipt=desc(OUT/'actual-command.json'),original_exit_code=p.returncode),sort_keys=True))
print((OUT/'stderr.log').read_text())
raise SystemExit(p.returncode if p.returncode else (0 if before==after else 3))
