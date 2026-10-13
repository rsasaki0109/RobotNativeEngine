"""NEW120 driver storage/lifecycle tests, with only stub processes and invented bytes."""
from datetime import datetime,timezone
import gzip,hashlib,json,os
from pathlib import Path
import subprocess,tempfile,unittest
from unittest.mock import patch
import byte_header,cohort_driver,checked_driver,run_registered_once
from fixed_contract import CASES,PROCESS_ORDER,STUDY_KIND
from spec_contract import ADMISSION_STATUS,ENVIRONMENT_KEYS,folder_for,load_spec,native_argv
from strict_io import canonical,descriptor,loads_metadata

OWN=Path(__file__).resolve().parent
ARTIFACTS=OWN/'pure-test-artifacts'


def save(path,value):path.write_text(json.dumps(value,indent=2)+'\n')


def raw_records(process):
    condition=next(c for c in CASES if c['case_id']==process['case_id'])
    target='negative' if '_neg_' in condition['case_id'] else 'positive' if '_pos_' in condition['case_id'] else 'zero'
    checkpoint=dict(schema_version=1,native_scalar_bits=32,state_scope=byte_header.STATE_SCOPE,synthetic_opaque_bytes=[1,2,3])
    start=dict(study_kind=STUDY_KIND,case_id=process['case_id'],variant=process['variant'],kind='run_start',schema_version=1,
        run_label=process['label'],target=target,target_position_rad_word=int(condition['input_f32_words']['target_pos_rad'],16),
        observer_enabled=process['observer']=='on',expected_outer_intervals=1,native_scalar_bits=32,state_codec='postcard-1.1.3-raw-exact-consumption',
        state_scope=byte_header.STATE_SCOPE,declared_conditions=byte_header.conditions(condition))
    observer={'SYNTHETIC_NO_WORLD_OR_SCIENTIFIC_VALUE':True} if process['observer']=='on' else None
    done=dict(kind='completed_interval',schema_version=1,completed_outer_interval=1,checkpoint=checkpoint,
        construction_companion=observer,stage_v1=observer,impulse_v1_ieee_words=observer)
    progress=dict(fixture_construction_started=True,fixture_constructed=True,pipeline_step_started=True,pipeline_step_returned=True,
        completed_outer_intervals=1,initial_records_written=1,completed_records_written=1,partial_records_written=0)
    return [start,dict(kind='initial_state',schema_version=1,checkpoint=checkpoint),done,
            dict(kind='run_end',schema_version=1,status='collected',progress=progress,records_written_before_terminal=3,error=None)]


def gzip_stub(process,path):
    data=b''.join(json.dumps(r,separators=(',',':')).encode()+b'\n' for r in raw_records(process))
    raw=bytearray(gzip.compress(data,mtime=0));raw[8]=0;raw[9]=255
    path.write_bytes(raw)


class FakeProcess:
    def __init__(self,pid,exit_code=0,wait_error=None):self.pid=pid;self.exit_code=exit_code;self.wait_error=wait_error;self.killed=False;self.returncode=None
    def wait(self,timeout=None):
        if self.wait_error is not None:
            error=self.wait_error;self.wait_error=None;raise error
        self.returncode=self.exit_code;return self.exit_code
    def poll(self):return self.returncode
    def kill(self):self.killed=True;self.exit_code=-9;self.returncode=-9


class NativeFactory:
    def __init__(self,spec,*,exit_code=0,wait_error=None,after_first=None):self.spec=spec;self.calls=[];self.exit_code=exit_code;self.wait_error=wait_error;self.after_first=after_first;self.processes=[]
    def __call__(self,argv,**kwargs):
        i=len(self.calls);process=PROCESS_ORDER[i]
        assert argv==native_argv(self.spec,process) and kwargs['start_new_session'] is False
        self.calls.append(argv)
        gzip_stub(process,Path(argv[-1]))
        kwargs['stdout'].write(b'SYNTHETIC_STUB_NO_NATIVE_WORLD\n')
        p=FakeProcess(200000+i,self.exit_code,self.wait_error if i==0 else None);self.processes.append(p)
        if self.after_first and i==0:self.after_first()
        return p


class DriverFaultTests(unittest.TestCase):
    def setUp(self):
        ARTIFACTS.mkdir(exist_ok=True);self.root=Path(tempfile.mkdtemp(prefix=self._testMethodName+'-',dir=ARTIFACTS))
        self.never_real=patch('subprocess.Popen',side_effect=AssertionError('REAL SUBPROCESS FORBIDDEN IN PURE TEST'));self.never_real.start();self.addCleanup(self.never_real.stop)
        self.no_signals=patch('os.killpg',side_effect=AssertionError('REAL PROCESS SIGNAL FORBIDDEN IN PURE TEST'));self.no_signals.start();self.addCleanup(self.no_signals.stop)
        for name in ('baseline.native','candidate.native','physical_reader.py'): (self.root/name).write_bytes(b'NONEXECUTABLE_SYNTHETIC_STUB_NOT_A_WORLD\n')
        roles=('protocol','collector_schema','baseline_build_binding','candidate_build_binding','producer_review','pure_tests_review')
        artifacts={}
        for role in roles:
            p=self.root/(role+'.json');save(p,dict(kind='SYNTHETIC_PURE_PREPARATION_ONLY'));artifacts[role]=descriptor(p)
        source_files=[OWN/n for n in ('fixed_contract.py','strict_io.py','spec_contract.py','cohort_driver.py','checked_driver.py','byte_header.py','run_registered_once.py')]
        frozen=[descriptor(p) for p in source_files]+[descriptor(self.root/'physical_reader.py')]
        self.candidate=dict(kind='frozen_motor_physical20_cohort',schema_version=1,registered_utc=datetime.now(timezone.utc).isoformat(),collection_worlds_before_registration=0,
            cases=CASES,process_order=PROCESS_ORDER,binaries={v:descriptor(self.root/(v+'.native')) for v in ('baseline','candidate')},cwd=str(self.root),
            output_root=str(self.root/'collection'),frozen_inputs=frozen,registration_artifacts=artifacts,source_scope='SYNTHETIC_NO_ACTUAL_BUILD_OR_WORLD',
            environment={k:os.environ.get(k) for k in ENVIRONMENT_KEYS.split()},timeout_seconds=1,reader_timeout_seconds=1,process_failure_policy='stop_on_first_failure_no_resume_or_replacement',
            driver_run_receipt_path=str(self.root/'checked-driver/actual-command.json'),driver_interpreter=descriptor('/usr/bin/python3'),
            checked_driver_source=descriptor(OWN/'checked_driver.py'),scientific_reader_source=descriptor(self.root/'physical_reader.py'),scientific_reader_output_path=str(self.root/'audit.json'),master_run_root=str(self.root/'master'))
        self.refresh()

    def refresh(self):
        cp=self.root/'candidate.json';save(cp,self.candidate)
        admission=dict(status=ADMISSION_STATUS,reviewed_candidate_sha256=descriptor(cp)['sha256'],binaries=self.candidate['binaries'],
            frozen_inputs_sha256=hashlib.sha256(canonical(self.candidate['frozen_inputs'])).hexdigest(),SYNTHETIC_MOCK_NOT_INDEPENDENT_ADMISSION=True)
        ap=self.root/'admission.json';save(ap,admission)
        wrapper=dict(kind='admitted_motor_physical20_cohort',schema_version=1,candidate=descriptor(cp),admission=descriptor(ap))
        self.wrapper=self.root/'wrapper.json';save(self.wrapper,wrapper);self.sha=descriptor(self.wrapper)['sha256'];self.spec=load_spec(str(self.wrapper),self.sha)

    def collect(self,**factory_args):
        factory=NativeFactory(self.spec,**factory_args)
        return cohort_driver.drive(self.spec,launch_native=True,spawn=factory),factory

    def master_factory(self,native_factory,reader_exit=0,reader_fault=None):
        calls=[]
        def spawn(command,**kwargs):
            calls.append(command)
            if len(calls)==1:
                def coordinator(coordinator_command,**ckw):
                    result=cohort_driver.drive(self.spec,launch_native=True,spawn=native_factory)
                    ckw['stdout'].write(json.dumps(result,sort_keys=True).encode()+b'\n')
                    return FakeProcess(300001,0 if result['status']=='all_120_processes_completed' else 2)
                inner=checked_driver.run_checked(self.spec,launch_native=True,spawn=coordinator)
                kwargs['stdout'].write(json.dumps(inner,sort_keys=True).encode()+b'\n')
                return FakeProcess(300000,0 if inner['actual_exit_code']==0 else 2)
            if reader_fault:raise reader_fault
            save(Path(self.spec['scientific_reader_output_path']),dict(kind='SYNTHETIC_READER_STUB_NO_SI_OR_NATIVE_EXECUTION'))
            return FakeProcess(300002,reader_exit)
        return spawn,calls

    def test_root_literal20_order120_no_pilot_or_tuning(self):
        self.assertEqual(len(CASES),20);self.assertEqual(len(PROCESS_ORDER),120)
        self.assertEqual([p['slot'] for p in PROCESS_ORDER],list(range(120)))
        self.assertEqual(sum('_zero_' in c['case_id'] for c in CASES),4)
        self.assertEqual([p['variant'] for p in PROCESS_ORDER[:60]],['baseline']*60)
        for p in PROCESS_ORDER:
            a=native_argv(self.spec,p);self.assertIn('--case',a);self.assertIn('--variant',a);self.assertNotIn('--dt',a)

    def test_default_validation_never_claims_or_spawns(self):
        self.assertEqual(cohort_driver.drive(self.spec)['actual_processes_started'],0)
        self.assertEqual(checked_driver.run_checked(self.spec)['actual_driver_processes'],0)
        self.assertEqual(run_registered_once.run_once(self.spec)['native_or_reader_processes_started'],0)
        self.assertFalse(Path(self.spec['output_root']).exists());self.assertFalse(Path(self.spec['master_run_root']).exists())

    def test_all120_stub_originals_and_byte_headers_complete(self):
        result,factory=self.collect()
        self.assertEqual(result['status'],'all_120_processes_completed');self.assertEqual(len(factory.calls),120)
        self.assertEqual(byte_header.complete_cohort(self.spec)['native_original_processes'],120)
        with self.assertRaises(FileExistsError):cohort_driver.drive(self.spec,launch_native=True,spawn=factory)

    def test_all120_claims_and_ancestor_barriers_precede_first_stub(self):
        def check_first():
            root=Path(self.spec['output_root'])
            self.assertTrue((root/'prelaunch-durability.json').exists())
            self.assertTrue(all((folder_for(self.spec,p)/'launch-claim.json').exists() for p in PROCESS_ORDER))
        result,_=self.collect(after_first=check_first);self.assertTrue(result['prelaunch_directory_barriers_complete'])

    def test_nonzero_original_stops_after_one_no_replacement(self):
        result,factory=self.collect(exit_code=7)
        self.assertEqual(len(factory.calls),1);self.assertEqual(result['cases'][0]['actual_exit_code'],7)
        self.assertEqual(len(result['unlaunched_case_labels']),119);self.assertEqual(result['retries'],0)

    def test_uncertain_spawn_never_claims_zero_or_resumes(self):
        def uncertain(*args,**kwargs):raise OSError('SYNTHETIC_POPEN_OUTCOME_UNKNOWN')
        result=cohort_driver.drive(self.spec,launch_native=True,spawn=uncertain)
        self.assertFalse(result['native_processes_started_count_is_complete']);self.assertEqual(len(result['launch_outcome_unknown_case_labels']),1)
        with self.assertRaises(FileExistsError):cohort_driver.drive(self.spec,launch_native=True,spawn=uncertain)

    def test_timeout_reaps_stub_and_keeps_original_negative_exit(self):
        result,factory=self.collect(wait_error=subprocess.TimeoutExpired('SYNTHETIC',1))
        self.assertTrue(factory.processes[0].killed);self.assertEqual(result['cases'][0]['actual_exit_code'],-9)
        self.assertTrue(result['cases'][0]['timed_out'])

    def test_interrupted_wait_reaps_stub_preserves_original_exit_and_error(self):
        result,factory=self.collect(wait_error=KeyboardInterrupt('SYNTHETIC_INTERRUPTED_WAIT'))
        self.assertTrue(factory.processes[0].killed)
        self.assertEqual(result['cases'][0]['actual_exit_code'],-9)
        self.assertFalse(result['cases'][0]['timed_out'])
        self.assertIn('KeyboardInterrupt',result['cases'][0]['error'])

    def test_wrong_binary_admission_does_not_authorize_any_call(self):
        wrapper=loads_metadata(self.wrapper.read_bytes())
        admission_path=Path(wrapper['admission']['path'])
        admission=loads_metadata(admission_path.read_bytes())
        admission['binaries']={'baseline':self.candidate['binaries']['candidate'],'candidate':self.candidate['binaries']['baseline']}
        save(admission_path,admission);wrapper['admission']=descriptor(admission_path);save(self.wrapper,wrapper)
        with self.assertRaises(ValueError):load_spec(str(self.wrapper),descriptor(self.wrapper)['sha256'])
        self.assertFalse(Path(self.spec['output_root']).exists())

    def test_process_started_receipt_failure_keeps_started_memory_and_reaps(self):
        original=cohort_driver.write_exclusive
        def fail(path,value):
            if Path(path).name=='process-started.json':raise OSError('SYNTHETIC_ENOSPC_STARTED')
            return original(path,value)
        with patch.object(cohort_driver,'write_exclusive',side_effect=fail):result,factory=self.collect()
        self.assertTrue(factory.processes[0].killed);self.assertEqual(result['actual_processes_started'],1)
        self.assertEqual(result['cases'][0]['actual_exit_code'],-9)

    def test_late_terminal_storage_failure_does_not_erase_original_start_exit(self):
        original=cohort_driver.write_exclusive
        def fail(path,value):
            if Path(path).name=='process-terminal.json':raise OSError('SYNTHETIC_ENOSPC_TERMINAL')
            return original(path,value)
        with patch.object(cohort_driver,'write_exclusive',side_effect=fail):result,factory=self.collect()
        self.assertEqual(len(factory.calls),1);self.assertEqual(result['actual_processes_started'],1)
        self.assertEqual(result['cases'][0]['actual_exit_code'],0);self.assertEqual(result['original_process_terminals'],0)

    def test_final_coordinator_storage_failure_cannot_claim_completed(self):
        original=cohort_driver.write_exclusive
        def fail(path,value):
            if Path(path).name=='driver-terminal.json':raise OSError('SYNTHETIC_ENOSPC_FINAL')
            return original(path,value)
        with patch.object(cohort_driver,'write_exclusive',side_effect=fail):result,_=self.collect()
        self.assertEqual(result['status'],'FAILED_OR_PARTIAL_RETAINED')
        self.assertIn('driver_terminal_storage_error',result)

    def test_directory_barrier_failure_starts_no_stub(self):
        with patch.object(cohort_driver,'sync_directory',side_effect=OSError('SYNTHETIC_DIRECTORY_SYNC')):
            result,factory=self.collect()
        self.assertEqual(len(factory.calls),0);self.assertFalse(result['prelaunch_directory_barriers_complete'])

    def test_source_or_binary_mutation_after_one_stops_further_calls(self):
        def mutate():Path(self.spec['binaries']['candidate']['path']).write_bytes(b'SYNTHETIC_MUTATED')
        result,factory=self.collect(after_first=mutate)
        self.assertEqual(len(factory.calls),1);self.assertFalse(result['source_inputs_unchanged'])

    def test_spec_case_order_and_bool_world_count_rejected(self):
        self.candidate['process_order']=list(reversed(PROCESS_ORDER));self.refresh_fails()
        self.candidate['process_order']=PROCESS_ORDER;self.candidate['collection_worlds_before_registration']=False;self.refresh_fails()

    def refresh_fails(self):
        with self.assertRaises(ValueError):self.refresh()

    def test_wrong_external_wrapper_sha_rejected_before_claim(self):
        with self.assertRaises(ValueError):load_spec(str(self.wrapper),'f'*64)
        self.assertFalse(Path(self.spec['output_root']).exists())

    def test_checked_nonzero_original_is_retained(self):
        result=checked_driver.run_checked(self.spec,launch_native=True,spawn=lambda *a,**k:FakeProcess(400000,8))
        self.assertEqual(result['actual_exit_code'],8)

    def test_checked_group_cleanup_uses_owned_group_with_no_real_signals(self):
        p=FakeProcess(400001,0,KeyboardInterrupt('SYNTHETIC_INTERRUPT'))
        with patch('os.killpg') as signals:
            result=checked_driver.run_checked(self.spec,launch_native=True,spawn=lambda *a,**k:p)
        self.assertEqual(signals.call_count,1);self.assertIn('SIGTERM',result['runtime_error'])
        self.assertEqual(result['actual_exit_code'],0)

    def test_master_exactly_one_reader_after120_headers_and_no_resume(self):
        factory=NativeFactory(self.spec);spawn,calls=self.master_factory(factory)
        result=run_registered_once.run_once(self.spec,launch_native=True,spawn=spawn)
        self.assertEqual(result['status'],'completed_fixed120_collection_and_one_reader')
        self.assertEqual(len(factory.calls),120);self.assertEqual(len(calls),2)
        self.assertTrue((Path(self.spec['output_root'])/'scientific-reader-launch-claim.json').exists())
        with self.assertRaises((FileExistsError,ValueError)):run_registered_once.run_once(self.spec,launch_native=True,spawn=spawn)

    def test_occupied_reader_output_refuses_before_any_claim_or_stub_call(self):
        Path(self.spec['scientific_reader_output_path']).write_bytes(b'SYNTHETIC_EXISTING_AUDIT')
        factory=NativeFactory(self.spec);spawn,calls=self.master_factory(factory)
        with self.assertRaises(ValueError):run_registered_once.run_once(self.spec,launch_native=True,spawn=spawn)
        self.assertEqual(calls,[]);self.assertEqual(factory.calls,[])
        self.assertFalse(Path(self.spec['output_root']).exists())
        self.assertFalse(Path(self.spec['master_run_root']).exists())

    def test_master_failed_collection_never_launches_reader(self):
        factory=NativeFactory(self.spec,exit_code=4);spawn,calls=self.master_factory(factory)
        result=run_registered_once.run_once(self.spec,launch_native=True,spawn=spawn)
        self.assertEqual(len(calls),1);self.assertIsNone(result['original_reader_CLI'])
        self.assertEqual(result['reader_launch_outcome'],'reserved_known_not_launched')

    def test_master_failed_byte_header_blocks_reader_without_new_world(self):
        factory=NativeFactory(self.spec);spawn,calls=self.master_factory(factory)
        with patch.object(byte_header,'complete_cohort',side_effect=ValueError('SYNTHETIC_HEADER_FAILURE')):
            result=run_registered_once.run_once(self.spec,launch_native=True,spawn=spawn)
        self.assertEqual(len(calls),1);self.assertIsNone(result['original_reader_CLI']);self.assertEqual(len(factory.calls),120)

    def test_master_reader_nonzero_is_retained_with_no_rerun(self):
        factory=NativeFactory(self.spec);spawn,calls=self.master_factory(factory,reader_exit=2)
        result=run_registered_once.run_once(self.spec,launch_native=True,spawn=spawn)
        self.assertEqual(len(calls),2);self.assertEqual(result['original_reader_CLI']['original_exit'],2)
        self.assertEqual(result['status'],'FAILED_OR_PARTIAL_RETAINED')
        with self.assertRaises((FileExistsError,ValueError)):run_registered_once.run_once(self.spec,launch_native=True,spawn=spawn)

    def test_master_reader_spawn_uncertain_claim_cannot_replace(self):
        factory=NativeFactory(self.spec);spawn,calls=self.master_factory(factory,reader_fault=OSError('SYNTHETIC_UNKNOWN_READER'))
        result=run_registered_once.run_once(self.spec,launch_native=True,spawn=spawn)
        self.assertEqual(result['reader_launch_outcome'],'launch_call_outcome_unknown')
        self.assertTrue((Path(self.spec['output_root'])/'scientific-reader-launch-claim.json').exists())
        with self.assertRaises((FileExistsError,ValueError)):run_registered_once.run_once(self.spec,launch_native=True,spawn=spawn)

    def test_native_header_tamper_truncation_and_off_payload_rejected(self):
        p=PROCESS_ORDER[2];file=self.root/'stub.jsonl.gz';gzip_stub(p,file)
        self.assertEqual(byte_header.header(file,p)['process'],p)
        records=raw_records(p);records[2]['construction_companion']={}
        raw=bytearray(gzip.compress(b''.join(json.dumps(r).encode()+b'\n' for r in records),mtime=0));raw[8]=0;raw[9]=255;file.write_bytes(raw)
        with self.assertRaises(ValueError):byte_header.header(file,p)
        gzip_stub(p,file);file.write_bytes(file.read_bytes()[:-4])
        with self.assertRaises(ValueError):byte_header.header(file,p)


if __name__=='__main__':unittest.main(verbosity=2)
