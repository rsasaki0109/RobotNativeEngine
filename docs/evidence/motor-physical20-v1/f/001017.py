"""Exclusive NEW120 collection plus once-only independent reader; no retry/resume."""
import argparse
from datetime import datetime,timezone
import os
from pathlib import Path
import signal,subprocess,time
import byte_header
from cohort_driver import guard
from checked_driver import stop_owned_group
from spec_contract import expected_checked_command,expected_reader_command,load_spec
from strict_io import bound,canonical,descriptor,loads,need,sync_directory,write_exclusive


def utc():return datetime.now(timezone.utc).isoformat()


def master_guard(spec):
    guard(spec)
    paths={d['path'] for d in spec['frozen_inputs']}
    for p in (Path(__file__).resolve(),Path(byte_header.__file__).resolve()):need(str(p) in paths,'actual master/header module not frozen')
    bound(spec['driver_interpreter']);bound(spec['checked_driver_source']);bound(spec['scientific_reader_source'])


def run_phase(spec,mode,command,folder,spawn,timeout):
    folder.mkdir(exist_ok=False);sync_directory(folder.parent)
    claim=dict(kind='exclusive_original_cli_claim',mode=mode,command=command,registered_wrapper_sha256=spec['_authenticated_wrapper_sha256'],
        created_utc=utc(),cwd=spec['cwd'],retries=0,resume=False)
    write_exclusive(folder/'command-claim.json',claim)
    receipt=dict(claim,process_started=False,pid=None,original_exit=None,launch_outcome='reserved_known_not_launched',error=None,start_unix_ns=time.time_ns())
    process=None
    try:
        master_guard(spec)
        with (folder/'stdout.log').open('xb') as out,(folder/'stderr.log').open('xb') as err:
            try:
                receipt['launch_outcome']='launch_call_outcome_unknown'
                process=spawn(command,cwd=spec['cwd'],stdout=out,stderr=err,env=os.environ.copy(),start_new_session=True)
                receipt.update(process_started=True,pid=process.pid,launch_outcome='known_started_exit_unknown')
                write_exclusive(folder/'process-started.json',dict(receipt))
                receipt['original_exit']=process.wait(timeout=timeout)
                receipt['launch_outcome']='known_started_exit_observed'
            except BaseException as error:
                receipt['error']=type(error).__name__+': '+str(error)
                if process is not None:
                    try:
                        receipt['original_exit'],actions=stop_owned_group(process)
                        receipt['cleanup_actions']=actions
                        receipt['launch_outcome']='known_started_exit_observed'
                    except BaseException as cleanup:
                        receipt['cleanup_error']=type(cleanup).__name__+': '+str(cleanup)
            finally:
                for f in (out,err):f.flush();os.fsync(f.fileno())
    except BaseException as error:
        receipt['error']=(receipt['error'] or '')+'; originalphase storage: '+type(error).__name__+': '+str(error)
    finally:
        receipt['end_unix_ns']=time.time_ns()
        try:master_guard(spec);receipt['source_inputs_unchanged']=True
        except BaseException as error:receipt['source_inputs_unchanged']=False;receipt['source_guard_error']=type(error).__name__+': '+str(error)
        for key in ('stdout','stderr'):
            p=folder/(key+'.log')
            if p.exists():receipt[key]=descriptor(p)
        # Failure to retain this original propagates; it must never permit reader launch.
        write_exclusive(folder/'actual-command.json',receipt)
    return receipt


def passed(receipt):
    return receipt['original_exit']==0 and receipt['error'] is None and receipt['source_inputs_unchanged'] and receipt['launch_outcome']=='known_started_exit_observed'


def run_once(spec,*,launch_native=False,spawn=None):
    master_guard(spec)
    if not launch_native:return dict(status='validated_no_process_started',native_or_reader_processes_started=0)
    spawn=subprocess.Popen if spawn is None else spawn
    need(not Path(spec['scientific_reader_output_path']).exists(),'registered reader output already exists; no collection permitted')
    master=Path(spec['master_run_root']);master.mkdir(exist_ok=False);sync_directory(master.parent)
    write_exclusive(master/'master-once-claim.json',dict(kind='exclusive_new120_collection_and_one_reader_claim',created_utc=utc(),
        registered_wrapper_sha256=spec['_authenticated_wrapper_sha256'],registered_candidate_sha256=spec['_authenticated_candidate_sha256'],
        process_order=spec['process_order'],binaries=spec['binaries'],scientific_reader=spec['scientific_reader_source'],retries=0,resume=False))
    result=dict(kind='new_physical20_once_master_terminal',status='FAILED_OR_PARTIAL_RETAINED',created_utc=utc(),ended_utc=None,
        original_collection_CLI=None,original_reader_CLI=None,prereader_byte_guard=None,
        reader_launch_outcome='reserved_known_not_launched',registered_wrapper_sha256=spec['_authenticated_wrapper_sha256'],
        retries=0,resumed=False,error=None,source_inputs_unchanged=False)
    try:
        collect=run_phase(spec,'collect',expected_checked_command(spec),master/'collect-invocation',spawn,spec['timeout_seconds']*120+240)
        result['original_collection_CLI']=collect
        need(passed(collect),'collection original CLI failed/unknown; reader remains unlaunched')
        inner=loads(Path(spec['driver_run_receipt_path']).read_bytes())
        need(inner['actual_exit_code']==0 and inner['source_inputs_unchanged'] and inner['runtime_error'] is inner['source_guard_error'] is None,'inner checked-coordinator command failed')
        stdout=Path(collect['stdout']['path']).read_bytes()
        need(stdout.endswith(b'\n') and canonical(loads(stdout.splitlines()[-1]))==canonical(inner),'collection CLI memory differs from inner original receipt')
        byte_guard=byte_header.complete_cohort(spec)
        write_exclusive(master/'prereader-byte-guard.json',byte_guard)
        result['prereader_byte_guard']=descriptor(master/'prereader-byte-guard.json')
        master_guard(spec)
        # This intent claim consumes the reader budget before Popen. An uncertain
        # launch may have existed; no changed output filename can replace it.
        intent=Path(spec['output_root'])/'scientific-reader-launch-claim.json'
        write_exclusive(intent,dict(kind='exclusive_scientific_reader_launch_intent',created_utc=utc(),
            command=expected_reader_command(spec,master/'collect-invocation/actual-command.json'),registered_wrapper_sha256=spec['_authenticated_wrapper_sha256'],
            prereader_byte_guard=result['prereader_byte_guard'],maximum_invocations=1,retries=0))
        result['reader_launch_outcome']='launch_call_outcome_unknown'
        reader=run_phase(spec,'audit',expected_reader_command(spec,master/'collect-invocation/actual-command.json'),master/'audit-invocation',spawn,spec['reader_timeout_seconds'])
        result['original_reader_CLI']=reader;result['reader_launch_outcome']=reader['launch_outcome']
        need(passed(reader),'scientific reader original exit/guard failed; result retained, no rerun')
        result['status']='completed_fixed120_collection_and_one_reader'
    except BaseException as error:
        result['error']=type(error).__name__+': '+str(error)
    finally:
        try:master_guard(spec);result['source_inputs_unchanged']=True
        except BaseException as error:result['error']=result['error'] or type(error).__name__+': '+str(error)
        result['ended_utc']=utc()
        write_exclusive(master/'master-terminal.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--spec',required=True)
    parser.add_argument('--spec-sha256',required=True);parser.add_argument('--launch-native',action='store_true');args=parser.parse_args()
    def interrupted(_signal,_frame):raise KeyboardInterrupt('owned NEW120 master termination requested')
    previous=signal.signal(signal.SIGTERM,interrupted)
    try:result=run_once(load_spec(args.spec,args.spec_sha256),launch_native=args.launch_native)
    finally:signal.signal(signal.SIGTERM,previous)
    import json
    print(json.dumps(result,sort_keys=True))
    return 0 if result['status'] in ('validated_no_process_started','completed_fixed120_collection_and_one_reader') else 2


if __name__=='__main__':raise SystemExit(main())
