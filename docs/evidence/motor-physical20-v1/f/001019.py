"""NEW20/120 prospective source/binary/protocol authentication; no engine calls."""
from datetime import datetime,timezone
import hashlib
from pathlib import Path
import strict_io,fixed_contract
from fixed_contract import CASES,PROCESS_ORDER
from strict_io import bound,canonical,integer,keys,loads_metadata,need

CANDIDATE_KEYS=('kind schema_version registered_utc collection_worlds_before_registration cases process_order binaries '
 'cwd output_root frozen_inputs registration_artifacts source_scope environment timeout_seconds reader_timeout_seconds '
 'process_failure_policy driver_run_receipt_path driver_interpreter checked_driver_source scientific_reader_source '
 'scientific_reader_output_path master_run_root')
REGISTRATION_ROLES='protocol collector_schema baseline_build_binding candidate_build_binding producer_review pure_tests_review'
ENVIRONMENT_KEYS='RUSTFLAGS CARGO_ENCODED_RUSTFLAGS RUSTDOCFLAGS RUST_TEST_THREADS'
ADMISSION_STATUS='PASS_PRE_DATA_PHYSICAL20_SOURCE_PROTOCOL_DRIVER_READER'


def same(a,b,message):need(canonical(a)==canonical(b),message)


def persistent(path,message):
    need(type(path) is str,'path must be string')
    p=Path(path)
    need(p.is_absolute() and p.resolve()==p and p!=Path('/workspace') and Path('/workspace') in p.parents,message)
    return p


def driver_source(spec):
    matches=[d for d in spec['frozen_inputs'] if Path(d['path']).name=='cohort_driver.py']
    need(len(matches)==1,'exactly one actual NEW coordinator source required')
    return matches[0]


def folder_for(spec,process):
    return Path(spec['output_root'])/process['variant']/process['case_id']/process['label']


def native_argv(spec,process):
    return [spec['binaries'][process['variant']]['path'],'--case',process['case_id'],'--variant',process['variant'],
            '--observer',process['observer'],'--label',process['label'],'--output',str(folder_for(spec,process)/'raw.jsonl.gz')]


def load_spec(path,expected_sha):
    need(type(expected_sha) is str and len(expected_sha)==64 and all(c in '0123456789abcdef' for c in expected_sha),'invalid wrapper SHA')
    p=persistent(str(Path(path)),'wrapper must be canonical persistent /workspace path')
    need(hashlib.sha256(p.read_bytes()).hexdigest()==expected_sha,'wrapper SHA differs')
    wrapper=loads_metadata(p.read_bytes());keys(wrapper,'kind schema_version candidate admission')
    need(wrapper['kind']=='admitted_motor_physical20_cohort' and type(wrapper['schema_version']) is int and wrapper['schema_version']==1,'wrong wrapper schema')
    bound(wrapper['candidate']);bound(wrapper['admission'])
    spec=loads_metadata(Path(wrapper['candidate']['path']).read_bytes());keys(spec,CANDIDATE_KEYS)
    need(spec['kind']=='frozen_motor_physical20_cohort' and type(spec['schema_version']) is int and spec['schema_version']==1,'wrong candidate schema')
    need(type(spec['collection_worlds_before_registration']) is int and spec['collection_worlds_before_registration']==0,'not prospective zero-world registration')
    same(spec['cases'],CASES,'case20 table differs');same(spec['process_order'],PROCESS_ORDER,'exact120 process order differs')
    registered=datetime.fromisoformat(spec['registered_utc']);need(registered.tzinfo is not None and registered<=datetime.now(timezone.utc),'bad registration UTC')
    keys(spec['binaries'],'baseline candidate')
    for d in spec['binaries'].values():bound(d);persistent(d['path'],'native executable must be persistent canonical workspace file')
    for name in ('cwd','output_root','master_run_root','scientific_reader_output_path','driver_run_receipt_path'):
        persistent(spec[name],'registered canonical workspace path required: '+name)
    need(Path(spec['cwd']).is_dir(),'cwd is absent')
    need(Path(spec['driver_run_receipt_path']).name=='actual-command.json','checked coordinator receipt filename differs')
    need(spec['output_root']!=spec['master_run_root'],'cohort and master claims must be distinct')
    need(spec['process_failure_policy']=='stop_on_first_failure_no_resume_or_replacement','wrong failure policy')
    for name,maximum in [('timeout_seconds',120),('reader_timeout_seconds',3600)]:
        integer(spec[name]);need(1<=spec[name]<=maximum,'bounded timeout required')
    need(type(spec['source_scope']) is str and spec['source_scope'],'source scope missing')
    keys(spec['environment'],ENVIRONMENT_KEYS)
    for value in spec['environment'].values():need(value is None or type(value) is str,'runtime environment binding type')
    bound(spec['driver_interpreter']);need(spec['driver_interpreter']['path']=='/usr/bin/python3','interpreter must be pinned /usr/bin/python3')
    bound(spec['checked_driver_source']);bound(spec['scientific_reader_source'])
    need(type(spec['frozen_inputs']) is list and spec['frozen_inputs'],'source/tool closure absent')
    paths=[]
    for d in spec['frozen_inputs']:bound(d);paths.append(d['path'])
    need(len(paths)==len(set(paths)),'duplicate frozen source path')
    for module in (strict_io,fixed_contract):need(str(Path(module.__file__).resolve()) in paths,'actually loaded shared source is not frozen')
    need(str(Path(__file__).resolve()) in paths,'actually loaded spec contract is not frozen')
    d=driver_source(spec)
    need(spec['checked_driver_source']['path']==str(Path(d['path']).parent/'checked_driver.py'),'wrong checked-driver path')
    for source in ('checked_driver_source','scientific_reader_source'):need(spec[source]['path'] in paths,'actual program not frozen: '+source)
    keys(spec['registration_artifacts'],REGISTRATION_ROLES)
    for d in spec['registration_artifacts'].values():bound(d)
    admission=loads_metadata(Path(wrapper['admission']['path']).read_bytes())
    need(admission.get('status')==ADMISSION_STATUS and admission.get('reviewed_candidate_sha256')==wrapper['candidate']['sha256'] and
         canonical(admission.get('binaries'))==canonical(spec['binaries']) and
         admission.get('frozen_inputs_sha256')==hashlib.sha256(canonical(spec['frozen_inputs'])).hexdigest(),
         'independent admission must bind candidate/two binaries/source closure')
    spec['_authenticated_wrapper_sha256']=expected_sha
    spec['_authenticated_candidate_sha256']=wrapper['candidate']['sha256']
    spec['_authenticated_wrapper_path']=str(p)
    return spec


def expected_driver_command(spec):
    return [spec['driver_interpreter']['path'],'-B',driver_source(spec)['path'],'--spec',spec['_authenticated_wrapper_path'],
            '--spec-sha256',spec['_authenticated_wrapper_sha256'],'--launch-native']


def expected_checked_command(spec):
    return [spec['driver_interpreter']['path'],'-B',spec['checked_driver_source']['path'],'--spec',spec['_authenticated_wrapper_path'],
            '--spec-sha256',spec['_authenticated_wrapper_sha256'],'--launch-native']


def expected_reader_command(spec,outer_receipt):
    from strict_io import descriptor
    return [spec['driver_interpreter']['path'],'-B',spec['scientific_reader_source']['path'],'--spec',spec['_authenticated_wrapper_path'],
            '--spec-sha256',spec['_authenticated_wrapper_sha256'],'--result-root',spec['output_root'],'--output',spec['scientific_reader_output_path'],
            '--checked-outer-receipt',str(outer_receipt),'--checked-outer-sha256',descriptor(outer_receipt)['sha256']]
