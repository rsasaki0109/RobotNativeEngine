#!/usr/bin/env python3
"""Synthetic JSON metadata receipts only: no processes, native worlds or builds."""
import copy
import datetime
import json
from pathlib import Path
import tempfile
import unittest

from audit_coupled_stages import (COHORT_ID, INDEPENDENCE, REPO, SOURCE_SCOPE,
                                 check_durable_provenance, record_for, stamp)

HERE=Path(__file__).parent

def metadata_fixture(directory):
    directory=Path(directory);driver=directory/'driver';driver.mkdir()
    def write(path,value):
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(value,sort_keys=True,indent=2)+'\n');return record_for(path)
    for name in ('collect_once.py','durable_common.py','prepare_archive.py'):
        (driver/name).write_text('# SYNTHETIC NONEXECUTED driver subject\n')
    closure=[record_for(driver/name) for name in ('collect_once.py','durable_common.py','prepare_archive.py')]
    root=directory/'collection';root.mkdir()
    source={'scope':SOURCE_SCOPE,'files':{'SYNTHETIC_NOT_A_RUNTIME_INPUT.rs':'0'*64}}
    binary=directory/'NONEXECUTED_SYNTHETIC_BINARY';binary.write_bytes(b'NO EXECUTION\n')
    binary=record_for(binary)
    readers=[record_for(HERE/'stage_roles.py')]
    protocol_record=write(directory/'SYNTHETIC-protocol.json',{'scope':'pure mock metadata only'})
    protocol={'root_driver':closure[0],'registered_utc':'2030-01-01T00:00:00+00:00',
              'fixed_cells':[{'build':build,'mass_kg':mass,'mode':mode,'run_label':f'{build}-m{mass}-{mode}'}
                   for build in ('baseline','rejected_inverse_w_plus_g') for mass in (5,20,50) for mode in ('on','fresh_on','off')],
              'reader_artifacts':readers}
    bindings={build:{'binary':binary,'repo_source_sha256':source['files']} for build in ('baseline','rejected_inverse_w_plus_g')}
    binding_records={build:write(directory/(build+'-binding.json'),value) for build,value in bindings.items()}
    proof=write(directory/'SYNTHETIC-pure-review.json',{'scope':'synthetic metadata; no actual collection'})
    spec={'schema_version':1,'generation':5,'status':'FROZEN_BEFORE_GENERATION5_COLLECTION',
          'independence':INDEPENDENCE,'cohort_id':COHORT_ID,'registered_utc':'2030-01-01T00:00:01+00:00',
          'authoritative_worlds_of_this_cohort_before_freeze':0,'fixed_cells':protocol['fixed_cells'],
          'world_limit':18,'preflight_gates':{'reader_pure_tests':'PASS','independent_readonly_review':'PASS'},
          'cohort_root':str(root),'repository_root':str(REPO),'protocol':protocol_record,
          'reader_artifacts':readers,'reader_pure_test_receipt':proof,'independent_readonly_review':proof,
          'source_guard_manifest':write(directory/'SYNTHETIC-source-map.json',source),
          'driver_artifacts':closure,'bindings':binding_records}
    spec_record=write(directory/'SYNTHETIC-spec.json',spec)
    base=datetime.datetime(2030,1,1,tzinfo=datetime.timezone.utc)
    times=lambda value:(base+datetime.timedelta(seconds=value)).isoformat()
    rows=[];cases=[]
    for ordinal,cell in enumerate(protocol['fixed_cells']):
        folder=root/'cases'/f'{ordinal:02d}-{cell["run_label"]}';folder.mkdir(parents=True)
        before=write(folder/'source-before.json',source);after=write(folder/'source-after.json',source)
        argv=[binary['path'],'--mass-kg',str(cell['mass_kg']),'--observer','off' if cell['mode']=='off' else 'on',
              '--label',cell['run_label'],'--output',str(root/'raw'/(cell['run_label']+'.jsonl.gz'))]
        row=dict(cell,attempted=True,exit_code=0,started_utc=times(3+5*ordinal),ended_utc=times(6+5*ordinal),
                 argv=argv,trace=None,stdout=None,stderr=None,source_guard_before=before,source_guard_after=after,error=None)
        claim=dict(row,attempted=False,exit_code=None,ended_utc=None,source_guard_after=None,case_index=ordinal,
                   cohort_id=COHORT_ID,launch_reserved=True,original_exit_code=None,binary=binary,frozen_spec=spec_record,
                   environment_policy='Inherited process environment; no driver overrides',retry_or_resume_permitted=False)
        launch=write(folder/'launch-claim.json',claim)
        write(folder/'process-started.json',{'case_index':ordinal,'pid':100+ordinal,'started_utc':times(4+5*ordinal),
                                             'argv':argv,'launch_claim':launch})
        exit_record=write(folder/'process-exit.json',{'case_index':ordinal,'pid':100+ordinal,'ended_utc':times(5+5*ordinal),
                        'original_exit_code':0,'source':'Actual subprocess.Popen.returncode; no status substitution'})
        case={'schema_version':1,'cohort_id':COHORT_ID,'case_index':ordinal,'original_exit_code':0,'launch_reserved':True,
              'process_started':True,'binary':binary,'original_process_exit_receipt':exit_record,
              'raw_output_exclusive_create_contract':'Unchanged collector142 OpenOptions::create_new(true)',
              'source_inputs_unchanged':True,'launch_error':None,'row':copy.deepcopy(row),'retry_or_resume_permitted':False}
        cases.append(write(folder/'case-terminal.json',case));rows.append(row)
    collection={'frozen_spec':spec_record,'source_binary_bindings':binding_records,'root_driver':closure[0],
                'case_receipts':cases,'runs':rows,'status':'GENERATION5_18_WORLD_PROCESSES_COMPLETED'}
    collection_record=write(root/'collection-index.json',collection)
    write(root/'collection-terminal.json',{'schema_version':1,'status':collection['status'],'cohort_id':COHORT_ID,
            'independence':INDEPENDENCE,'index':collection_record,'frozen_spec':spec_record,
            'worlds_launch_reserved':18,'worlds_attempted':18,'worlds_completed':18,'driver_exit_code':0,
            'case_exit_codes_preserved':True,'retry_or_resume_permitted':False})
    return collection,collection_record,protocol,protocol_record,bindings,stamp(times(2)),stamp(times(99)),write

class ProvenanceTests(unittest.TestCase):
    def test_complete_source_only_metadata_contract(self):
        with tempfile.TemporaryDirectory(prefix='synthetic-provenance-',dir=HERE) as temp:
            values=metadata_fixture(temp)
            check_durable_provenance(*values[:-1])

    def test_strict_original_exit_spec_and_launch_tamper(self):
        names=('wrong_case_index','signal_exit','missing_process_start','bool_pid','late_spec_freeze',
               'missing_case_receipt','changed_source_guard','wrong_binary','wrong_driver_closure',
               'retry_allowed','missing_launch','terminal_count_bool','unknown_spec_field')
        for name in names:
            with self.subTest(name=name),tempfile.TemporaryDirectory(prefix='synthetic-provenance-',dir=HERE) as temp:
                collection,record,protocol,p_record,bindings,start,end,write=metadata_fixture(temp)
                root=Path(record['path']).parent;folder=root/'cases'/'00-baseline-m5-on'
                def rewrite_case(change):
                    case=json.loads(Path(collection['case_receipts'][0]['path']).read_bytes());change(case)
                    collection['case_receipts'][0]=write(Path(collection['case_receipts'][0]['path']),case)
                if name=='wrong_case_index': rewrite_case(lambda case:case.update(case_index=1))
                elif name=='signal_exit':
                    exit_path=folder/'process-exit.json';value=json.loads(exit_path.read_bytes());value['original_exit_code']=-9
                    updated=write(exit_path,value);rewrite_case(lambda case:case.update(original_process_exit_receipt=updated))
                elif name=='missing_process_start': (folder/'process-started.json').unlink()
                elif name=='bool_pid':
                    path=folder/'process-started.json';value=json.loads(path.read_bytes());value['pid']=True;write(path,value)
                elif name=='late_spec_freeze':
                    path=Path(collection['frozen_spec']['path']);value=json.loads(path.read_bytes());value['registered_utc']='2031-01-01T00:00:00+00:00'
                    collection['frozen_spec']=write(path,value)
                elif name=='missing_case_receipt': collection['case_receipts'].pop()
                elif name=='changed_source_guard':
                    path=Path(collection['frozen_spec']['path']);value=json.loads(path.read_bytes())
                    new={'scope':SOURCE_SCOPE,'files':{'different.rs':'0'*64}}
                    value['source_guard_manifest']=write(Path(value['source_guard_manifest']['path']),new)
                    collection['frozen_spec']=write(path,value)
                elif name=='wrong_binary': rewrite_case(lambda case:case.update(binary=p_record))
                elif name=='wrong_driver_closure':
                    path=Path(collection['frozen_spec']['path']);value=json.loads(path.read_bytes());value['driver_artifacts'].pop()
                    collection['frozen_spec']=write(path,value)
                elif name=='retry_allowed': rewrite_case(lambda case:case.update(retry_or_resume_permitted=True))
                elif name=='missing_launch': (folder/'launch-claim.json').unlink()
                elif name=='terminal_count_bool':
                    path=root/'collection-terminal.json';value=json.loads(path.read_bytes());value['worlds_completed']=True;write(path,value)
                elif name=='unknown_spec_field':
                    path=Path(collection['frozen_spec']['path']);value=json.loads(path.read_bytes());value['extra']='unexpected'
                    collection['frozen_spec']=write(path,value)
                with self.assertRaises((ValueError,KeyError,FileNotFoundError)):
                    check_durable_provenance(collection,record,protocol,p_record,bindings,start,end)

if __name__=='__main__': unittest.main(verbosity=2)
