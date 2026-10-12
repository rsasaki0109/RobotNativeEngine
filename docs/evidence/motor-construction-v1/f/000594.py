from pathlib import Path
import datetime,hashlib,json,os,subprocess,sys
root=Path(__file__).resolve().parent
label=sys.argv[1];command=sys.argv[2:]
logs=root/'validation';logs.mkdir(exist_ok=True)
log=logs/(label+'.log');receipt=logs/(label+'.json')
if log.exists() or receipt.exists():raise SystemExit('retained validation stage exists')
def source_snapshot():
    records=[]
    for path in sorted((root/'source-copy/third_party/rapier3d').rglob('*')):
        if path.is_file():
            data=path.read_bytes();records.append({'path':str(path.relative_to(root/'source-copy')),'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)})
    return records
before_sources=source_snapshot()
start=datetime.datetime.now(datetime.timezone.utc).isoformat()
with log.open('xb') as output:
    result=subprocess.run(command,cwd=root/'source-copy/third_party/rapier3d',stdout=output,stderr=subprocess.STDOUT)
end=datetime.datetime.now(datetime.timezone.utc).isoformat();data=log.read_bytes()
sources=[]
for path in sorted((root/'source-copy/third_party/rapier3d').rglob('*')):
    if path.is_file():
        content=path.read_bytes();sources.append({'path':str(path.relative_to(root/'source-copy')),'sha256':hashlib.sha256(content).hexdigest(),'bytes':len(content)})
value={'schema_version':1,'stage':label,'argv':command,'cwd':str(root/'source-copy/third_party/rapier3d'),'started_utc':start,'ended_utc':end,'exit_code':result.returncode,'activation_script':'/workspace/.rne-onboarding/env.sh','environment':{k:os.environ.get(k) for k in ['CARGO_BUILD_JOBS','CARGO_TARGET_DIR','CARGO_INCREMENTAL','CARGO_PROFILE_DEV_DEBUG','CARGO_PROFILE_TEST_DEBUG','RUSTUP_TOOLCHAIN','RUSTDOCFLAGS','RUSTFLAGS']},'log':{'path':str(log),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()},'source_inventory_sha256':hashlib.sha256(json.dumps(sources,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'source_inventory_sha256_before':hashlib.sha256(json.dumps(before_sources,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'source_unchanged_during_command':sources==before_sources,'source_inventory':sources,'worlds_executed':0,'scope':'Cargo compilation/documentation or explicitly filtered pure storage/schema/flag tests only; no simulation run'}
with receipt.open('x') as stream:json.dump(value,stream,indent=2);stream.write('\n')
print(json.dumps({'stage':label,'exit':result.returncode,'log':str(log),'receipt_sha256':hashlib.sha256(receipt.read_bytes()).hexdigest()}),flush=True)
if result.returncode:print(data.decode(errors='replace')[-10000:])
raise SystemExit(result.returncode)
