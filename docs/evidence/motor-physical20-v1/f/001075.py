"""Invented metadata/records and injected fake processes only; no subprocess."""
import copy
from datetime import datetime,timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import physical_reader as cohort_reader
from fixed_contract import CASES, PROCESS_ORDER
from spec_contract import expected_driver_command,load_spec,REGISTRATION_ROLES,ADMISSION_STATUS
from strict_io import canonical,descriptor,loads,write_exclusive
from synthetic_fixtures import mock_records,raw,gz
from physical_reference import ideal_row,one_slice
from fractions import Fraction as F

HERE=Path(__file__).resolve().parent
DRIVER=HERE.parent/'driver'
module_spec=importlib.util.spec_from_file_location('synthetic_injected_driver',DRIVER/'cohort_driver.py')
fake_driver=importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(fake_driver)


class FakeNative:
    def __init__(self,argv,stdout,case,index):
        self.pid=710000+index
        self.returncode=None
        Path(argv[-1]).write_bytes(gz(raw(mock_records(case))))
        stdout.write(b'SYNTHETIC_ONLY_NOT_NATIVE\n')
    def wait(self,timeout=None):
        self.returncode=0;return 0
    def poll(self):return self.returncode
    def kill(self):raise AssertionError('pure success fake cannot need cleanup')


class CohortTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(dir=HERE)
        self.folder=Path(self.tmp.name)
        self.root=self.folder/'collection'

    def tearDown(self):self.tmp.cleanup()

    def make_collection(self):
        binaries={}
        for name in ('baseline','candidate'):
            binary=self.folder/('NONEXECUTABLE_SYNTHETIC_'+name);binary.write_bytes(('no executable '+name).encode());binaries[name]=descriptor(binary)
        cwd=self.folder/'collector';cwd.mkdir()
        roles={}
        for role in REGISTRATION_ROLES.split():
            p=self.folder/(role+'.json');p.write_text('{"synthetic_only":true}\n');roles[role]=descriptor(p)
        inputs=[descriptor(p) for p in HERE.glob('*.py')]
        inputs += [descriptor(DRIVER/n) for n in ['cohort_driver.py','checked_driver.py','spec_contract.py','fixed_contract.py','strict_io.py']]
        candidate=dict(kind='frozen_motor_physical20_cohort',schema_version=1,
            registered_utc=datetime.now(timezone.utc).isoformat(),collection_worlds_before_registration=0,
            cases=copy.deepcopy(CASES),process_order=copy.deepcopy(PROCESS_ORDER),binaries=binaries,cwd=str(cwd),output_root=str(self.root),
            frozen_inputs=inputs,registration_artifacts=roles,source_scope='SYNTHETIC_ONLY_NO_NATIVE_SCOPE',
            environment={k:os.environ.get(k) for k in ['RUSTFLAGS','CARGO_ENCODED_RUSTFLAGS','RUSTDOCFLAGS','RUST_TEST_THREADS']},
            timeout_seconds=1,reader_timeout_seconds=3600,scientific_reader_source=descriptor(HERE/'physical_reader.py'),
            scientific_reader_output_path=str(self.folder/'registered-audit.json'),master_run_root=str(self.folder/'master'),process_failure_policy='stop_on_first_failure_no_resume_or_replacement',
            driver_run_receipt_path=str(self.folder/'actual-command.json'),
            driver_interpreter=descriptor('/usr/bin/python3'),checked_driver_source=descriptor(DRIVER/'checked_driver.py'))
        p=self.folder/'candidate.json';write_exclusive(p,candidate)
        admission=self.folder/'SYNTHETIC_ADMISSION.json'
        write_exclusive(admission,dict(status=ADMISSION_STATUS,
            reviewed_candidate_sha256=descriptor(p)['sha256'],binaries=candidate['binaries'],
            frozen_inputs_sha256=hashlib.sha256(canonical(inputs)).hexdigest(),synthetic_only=True))
        wrapper=self.folder/'wrapper.json'
        write_exclusive(wrapper,dict(kind='admitted_motor_physical20_cohort',schema_version=1,
            candidate=descriptor(p),admission=descriptor(admission)))
        spec=load_spec(wrapper,descriptor(wrapper)['sha256'])
        started=datetime.now(timezone.utc).isoformat();calls=[]
        def spawn(argv,*,stdout,stderr,**kwargs):
            i=len(calls);calls.append(argv);return FakeNative(argv,stdout,PROCESS_ORDER[i],i)
        with patch('subprocess.Popen',side_effect=AssertionError('no native/subprocess allowed')):
            terminal=fake_driver.drive(spec,launch_native=True,spawn=spawn)
        ended=datetime.now(timezone.utc).isoformat()
        stdout=self.folder/'checked.stdout';stdout.write_bytes(canonical(terminal)+b'\n')
        stderr=self.folder/'checked.stderr';stderr.write_bytes(b'')
        inner=dict(kind='original_checked_driver_command',
            schema_version=2,command=expected_driver_command(spec),cwd=spec['cwd'],
            registered_wrapper_sha256=spec['_authenticated_wrapper_sha256'],
            registered_candidate_sha256=spec['_authenticated_candidate_sha256'],started_utc=started,ended_utc=ended,
            driver_process_started=True,driver_process_id=900001,launch_outcome='known_started_exit_observed',
            actual_exit_code=0,source_inputs_unchanged=True,source_guard_error=None,runtime_error=None,
            stdout=descriptor(stdout),stderr=descriptor(stderr))
        write_exclusive(Path(spec['driver_run_receipt_path']),inner)
        outer_dir=Path(spec['master_run_root'])/'collect-invocation';outer_dir.mkdir(parents=True)
        outer_stdout=outer_dir/'stdout.log';outer_stdout.write_bytes(canonical(inner)+b'\n')
        outer_stderr=outer_dir/'stderr.log';outer_stderr.write_bytes(b'')
        outer=outer_dir/'actual-command.json'
        def nanos(value):
            delta=datetime.fromisoformat(value)-datetime(1970,1,1,tzinfo=timezone.utc)
            return (delta.days*86400+delta.seconds)*1000000000+delta.microseconds*1000
        write_exclusive(outer,dict(kind='exclusive_original_cli_claim',mode='collect',
            command=['/usr/bin/python3','-B',spec['checked_driver_source']['path'],'--spec',
                spec['_authenticated_wrapper_path'],'--spec-sha256',spec['_authenticated_wrapper_sha256'],'--launch-native'],
            registered_wrapper_sha256=spec['_authenticated_wrapper_sha256'],created_utc=started,
            cwd=spec['cwd'],retries=0,resume=False,process_started=True,pid=900000,
            original_exit=0,launch_outcome='known_started_exit_observed',error=None,
            start_unix_ns=nanos(started),end_unix_ns=nanos(ended),source_inputs_unchanged=True,
            stdout=descriptor(outer_stdout),stderr=descriptor(outer_stderr)))
        self.outer=outer
        self.outer_sha=descriptor(outer)['sha256']
        return spec,wrapper

    def test_whole_120_synthetic_original_receipts_and_arithmetic(self):
        spec,_=self.make_collection()
        result=cohort_reader.qualify_cohort(spec,self.root,self.outer,self.outer_sha)
        self.assertEqual(result['fidelity_status'],'PASS_COMPLETE_DECLARED_SCOPE')
        self.assertEqual(result['actual_processes'],120)
        self.assertEqual(result['native_motor_solve_checks'],240)
        self.assertEqual(result['constructor_checks'],80)
        self.assertFalse(result['native_motor_accuracy_verified'])
        self.assertEqual(len(result['physical_diagnostics']),40)
        self.assertEqual(len(result['baseline_candidate_comparison']),20)
        self.assertFalse(result['old_calibration_reclassified'])

    def test_missing_index_wrong_original_exit_and_guard_fail(self):
        spec,_=self.make_collection()
        p=self.root/'collection-index.json';saved=p.read_bytes();p.unlink()
        with self.assertRaises(FileNotFoundError):cohort_reader.qualify_cohort(spec,self.root,self.outer,self.outer_sha)
        p.write_bytes(saved)
        receipt=Path(spec['driver_run_receipt_path']);value=loads(receipt.read_bytes());value['actual_exit_code']=2
        receipt.write_bytes(canonical(value))
        with self.assertRaises(ValueError):cohort_reader.qualify_cohort(spec,self.root,self.outer,self.outer_sha)

    def test_exclusive_audit_claim_blocks_new_output_after_failure(self):
        self.root.mkdir()
        first=self.folder/'failure.json'
        self.assertEqual(cohort_reader.audit_once(self.folder/'absent_spec.json','0'*64,self.root,first,self.folder/'absent_outer.json','0'*64),2)
        self.assertEqual(loads(first.read_bytes())['fidelity_status'],'FAILED_SCHEMA_BINDING_OR_PARTIAL_COLLECTION_RETAINED')
        with self.assertRaises(FileExistsError):
            cohort_reader.audit_once(self.folder/'absent_spec.json','0'*64,self.root,self.folder/'another.json',self.folder/'absent_outer.json','0'*64)
        self.assertFalse((self.folder/'another.json').exists())

    def test_null_comparison_can_be_inconclusive_without_requiring_favorable_change(self):
        spec,_=self.make_collection()
        result=cohort_reader.qualify_cohort(spec,self.root,self.outer,self.outer_sha)
        zero=[r for r in result['baseline_candidate_comparison'] if 'zero' in r['case_id']]
        self.assertEqual(len(zero),4)
        self.assertTrue(all(x['status']=='INCONCLUSIVE_NO_CAPTURED_DIFFERENCE' for x in zero))

    def test_raw_and_launch_coverage_faults_reject_not_silently_skip(self):
        spec,_=self.make_collection()
        terminal=self.root/'driver-terminal.json';v=loads(terminal.read_bytes())
        v['case_launch_accounting'][119]['actual_exit_code']=2;terminal.write_bytes(canonical(v))
        stdout=Path(loads(Path(spec['driver_run_receipt_path']).read_bytes())['stdout']['path'])
        stdout.write_bytes(canonical(v)+b'\n')
        # Receipt hash disagreement alone is a truthful rejection; it must not be ignored.
        with self.assertRaises(ValueError):cohort_reader.qualify_cohort(spec,self.root,self.outer,self.outer_sha)

    def test_late_source_guard_metadata_fault_rejects(self):
        spec,_=self.make_collection()
        guard=self.root/'candidate'/'m2_h2_zero_cap2'/'off'/'source-guard-after.json'
        value=loads(guard.read_bytes());value['registered_spec_sha256']='0'*64;guard.write_bytes(canonical(value))
        with self.assertRaises(ValueError):cohort_reader.qualify_cohort(spec,self.root,self.outer,self.outer_sha)

    def test_outer_checked_cli_original_failure_can_not_be_hidden_by_child_success(self):
        spec,_=self.make_collection()
        inner=loads(Path(spec['driver_run_receipt_path']).read_bytes())
        original=loads(self.outer.read_bytes())
        for mutate in [lambda v:v.update(original_exit=2),lambda v:v.update(process_started=False),
                       lambda v:v.update(source_inputs_unchanged=False),lambda v:v.update(resume=True),
                       lambda v:v.update(cleanup_requested='extra failure field')]:
            value=copy.deepcopy(original);mutate(value);self.outer.write_bytes(canonical(value))
            with self.assertRaises(ValueError):
                cohort_reader.verify_checked_cli(spec,self.outer,descriptor(self.outer)['sha256'],inner)

    def test_outer_memory_and_disk_receipt_must_agree(self):
        spec,_=self.make_collection()
        inner=loads(Path(spec['driver_run_receipt_path']).read_bytes())
        value=loads(self.outer.read_bytes())
        Path(value['stdout']['path']).write_bytes(b'{"fake":true}\n')
        value['stdout']=descriptor(value['stdout']['path']);self.outer.write_bytes(canonical(value))
        with self.assertRaises(ValueError):
            cohort_reader.verify_checked_cli(spec,self.outer,descriptor(self.outer)['sha256'],inner)


if __name__=='__main__':unittest.main()
