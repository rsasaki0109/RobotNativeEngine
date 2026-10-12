#!/usr/bin/python3
"""Retain original inspection CLI exit; no native or scientific reader call."""
from datetime import datetime,timezone
import hashlib,json
from pathlib import Path
import subprocess,time
OWN=Path(__file__).resolve().parent
command=['/usr/bin/python3','-B',str(OWN/'postreview_retention.py')]
claim=OWN/'original-postreview-command-claim.json'
with claim.open('x') as f:json.dump(dict(command=command,created_utc=datetime.now(timezone.utc).isoformat()),f,indent=2);f.write('\n')
start=time.time_ns()
with (OWN/'stdout.log').open('xb') as out,(OWN/'stderr.log').open('xb') as err:
    p=subprocess.run(command,cwd=OWN,stdout=out,stderr=err,check=False)
def desc(path):return dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),bytes=path.stat().st_size)
x=dict(kind='original_independent_postcollection_inspection_CLI',command=command,cwd=str(OWN),original_exit_code=p.returncode,start_unix_ns=start,end_unix_ns=time.time_ns(),stdout=desc(OWN/'stdout.log'),stderr=desc(OWN/'stderr.log'),native_or_scientific_reader_invocations=0)
with (OWN/'actual-original-command.json').open('x') as f:json.dump(x,f,indent=2);f.write('\n')
print(json.dumps(dict(original_exit_code=p.returncode,receipt=desc(OWN/'actual-original-command.json')),sort_keys=True))
if p.returncode:print((OWN/'stderr.log').read_text())
raise SystemExit(p.returncode)
