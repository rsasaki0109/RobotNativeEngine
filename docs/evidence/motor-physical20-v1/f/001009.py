"""Freeze NEW20/120 candidate bytes from actual root bindings; never launches a world."""
import argparse
from datetime import datetime,timezone
import os
from pathlib import Path
from fixed_contract import CASES,PROCESS_ORDER
from spec_contract import ENVIRONMENT_KEYS,REGISTRATION_ROLES
from strict_io import bound,canonical,descriptor,loads_metadata,need,write_exclusive


def assemble(*,binding_path,protocol_path,collector_schema_path,reader_source_path,producer_review_path,pure_tests_review_path,
             cwd,output_root,master_run_root,driver_run_receipt_path,reader_output_path,output_candidate_path,extra_paths=(),
             environment=None,timeout_seconds=60,reader_timeout_seconds=600):
    own=Path(__file__).resolve().parent
    binding=loads_metadata(Path(binding_path).read_bytes());protocol=loads_metadata(Path(protocol_path).read_bytes())
    need(binding['kind']=='NEW_PHYSICAL_MOTOR_ACTUAL_GUARDED_BUILDS' and type(binding['native_worlds_before_binding']) is int and binding['native_worlds_before_binding']==0,'root actual guarded build required')
    need(canonical(protocol['cases'])==canonical(CASES) and canonical(protocol['process_order'])==canonical(PROCESS_ORDER),'root20/120 protocol differs')
    for d in binding['binaries'].values():bound(d)
    registration=dict(protocol=descriptor(protocol_path),collector_schema=descriptor(collector_schema_path),
        baseline_build_binding=descriptor(binding_path),candidate_build_binding=descriptor(binding_path),
        producer_review=descriptor(producer_review_path),pure_tests_review=descriptor(pure_tests_review_path))
    paths={d['path'] for d in binding['source_descriptors']+binding['receipt_files']}
    paths.update(d['path'] for d in registration.values())
    paths.update(str(p) for p in own.iterdir() if p.is_file() and (p.suffix=='.py' or p.name in ('README.md','handoff.json','shared-contract-template.json','protocol-template.json')))
    reader=Path(reader_source_path)
    # Freeze actual sibling reader modules only; archived helper/source payloads are never imported.
    paths.update(str(p) for p in reader.parent.iterdir() if p.is_file() and p.suffix=='.py')
    paths.update(str(Path(p)) for p in extra_paths)
    env={k:os.environ.get(k) for k in ENVIRONMENT_KEYS.split()} if environment is None else environment
    need(set(env)==set(ENVIRONMENT_KEYS.split()) and all(v is None or type(v) is str for v in env.values()),'non-secret exact runtime allowlist required')
    candidate=dict(kind='frozen_motor_physical20_cohort',schema_version=1,registered_utc=datetime.now(timezone.utc).isoformat(),
        collection_worlds_before_registration=0,cases=CASES,process_order=PROCESS_ORDER,binaries=binding['binaries'],
        cwd=str(cwd),output_root=str(output_root),frozen_inputs=[descriptor(p) for p in sorted(paths)],registration_artifacts=registration,
        source_scope=binding['scope']+' NEW20/120 driver/header and independent reader sources/actual pure receipts remain separately byte-bound. No hardware float-mode or physical adoption claim.',
        environment=env,timeout_seconds=timeout_seconds,reader_timeout_seconds=reader_timeout_seconds,
        process_failure_policy='stop_on_first_failure_no_resume_or_replacement',driver_run_receipt_path=str(driver_run_receipt_path),
        driver_interpreter=descriptor('/usr/bin/python3'),checked_driver_source=descriptor(own/'checked_driver.py'),
        scientific_reader_source=descriptor(reader_source_path),scientific_reader_output_path=str(reader_output_path),master_run_root=str(master_run_root))
    write_exclusive(Path(output_candidate_path),candidate)
    return descriptor(output_candidate_path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('binding','protocol','collector-schema','reader-source','producer-review','pure-tests-review','cwd','output-root',
                 'master-run-root','driver-run-receipt','reader-output','output'):
        parser.add_argument('--'+name,required=True)
    parser.add_argument('--extra-path',action='append',default=[])
    parser.add_argument('--timeout-seconds',type=int,default=60);parser.add_argument('--reader-timeout-seconds',type=int,default=600)
    args=parser.parse_args()
    result=assemble(binding_path=args.binding,protocol_path=args.protocol,collector_schema_path=args.collector_schema,
        reader_source_path=args.reader_source,producer_review_path=args.producer_review,pure_tests_review_path=args.pure_tests_review,
        cwd=args.cwd,output_root=args.output_root,master_run_root=args.master_run_root,driver_run_receipt_path=args.driver_run_receipt,
        reader_output_path=args.reader_output,output_candidate_path=args.output,extra_paths=args.extra_path,
        timeout_seconds=args.timeout_seconds,reader_timeout_seconds=args.reader_timeout_seconds)
    import json
    print(json.dumps(dict(candidate=result,native_or_scientific_reader_invocations=0,independent_admission_required=True),sort_keys=True))


if __name__=='__main__':main()
