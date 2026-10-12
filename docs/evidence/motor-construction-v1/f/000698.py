"""Plan/hash or copy whole opaque evidence into an external review staging tree."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess

ROOT = Path('/workspace/.rne-onboarding/recovery-20261012')
OWN = ROOT/'next-motor-evidence-export-preparation'
REPO = Path('/workspace/RobotNativeEngine')
LIMIT = 1 << 20
PUBLIC = 'docs/evidence/motor-construction-v1'
MEDIA = 'docs/media/position-motor-calibration-motor-construction-source.json'
BASE = '3be26ef0'
PURE_DIR = 'SYNTHETIC-STUB-FIXTURES-NOT-SCIENTIFIC-DATA'
FAULT_NAMES = {'terminal.json','start.json','pid.json','claim.json','collector-launch-reservation.json',
               'literal-comparison.json','stdout.log','stderr.log'}
SEEDS = ['next-motor-source-transition-72','next-motor-construction-ci-transition',
         'next-motor-construction-cohort-reader','next-motor-construction-cohort-reader-v2',
         'next-motor-construction-six','next-motor-construction-protocol',
         'next-motor-construction-protocol-review','next-motor-construction-producer-review',
         'next-integrated-collector-review','next-motor-integrated-build-binding',
         'next-motor-construction-preparation','next-motor-construction-collector',
         'next-companion-validation']
SKIP_DIRS = {'__pycache__','target','target-durable','baseline','source-copy','formatting-only-intermediates'}

def require(ok, message):
    if not ok: raise RuntimeError(message)

def canonical(x):
    return (json.dumps(x,sort_keys=True,separators=(',',':'),ensure_ascii=True)+'\n').encode('ascii')

def descriptor(path):
    path=Path(path);require(path.is_absolute() and path == path.resolve(),'noncanonical/symlink input')
    fd=os.open(path,os.O_RDONLY|os.O_NOFOLLOW)
    try:
        a=os.fstat(fd);require(stat.S_ISREG(a.st_mode),'nonregular input')
        h=hashlib.sha256();count=0
        while True:
            block=os.read(fd,1<<20)
            if not block:break
            h.update(block);count+=len(block)
        b=os.fstat(fd)
        require((a.st_ino,a.st_size,a.st_mtime_ns,a.st_ctime_ns)==(b.st_ino,b.st_size,b.st_mtime_ns,b.st_ctime_ns),'input changed while hashing')
        require(count==b.st_size,'short hash read')
        return {'path':str(path),'sha256':h.hexdigest(),'size_bytes':count}
    finally:os.close(fd)

def write_new(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('xb') as f:
        f.write(data);f.flush();os.fsync(f.fileno())

def external(path):
    path=Path(path)
    require(path.is_absolute() and path == path.resolve(),'noncanonical staging output')
    require(path==OWN or OWN in path.parents,'output must remain in owned external preparation directory')
    require(not path.exists(),'output already exists; immutable attempts are not replaced')
    return path

def bindings():
    p=ROOT/'next-motor-source-transition-72'
    f=descriptor(p/'frozen-v2/frozen-bindings.json')
    require(f['sha256']=='6a99b2ccb8573c1141369c5498bcea6e9bc8f61f62189988066cd7f020f2270b','72 frozen changed')
    frozen=json.loads(Path(f['path']).read_bytes());a=frozen['prepared_binding']['inputs']['artifacts']
    review=p/'independent-postcollection-review/independent-postcollection-source-transition-review.json'
    require(descriptor(review)['sha256']=='12b08d6018456e9b2a9fbf5959da8ad6e79f2dc3714f602428308990549d060d','72 independent review changed')
    guard=json.loads(Path(a['source_guard']['path']).read_bytes())
    return frozen,a,{row['path']:row for row in guard['inputs']}

def plan(output):
    output=external(output)
    frozen,artifacts,guard=bindings()
    candidates=set();omitted=[];skipped_groups=set()
    for seed in SEEDS:
        root=ROOT/seed
        for current,dirs,files in os.walk(root):
            for name in list(dirs):
                if name in SKIP_DIRS:
                    skipped_groups.add(str(Path(current)/name));dirs.remove(name)
            dirs.sort()
            for name in sorted(files):
                path=Path(current)/name
                if PURE_DIR in path.parts and name not in FAULT_NAMES:
                    d=descriptor(path)
                    omitted.append([d['path'],d['sha256'],d['size_bytes'],'REGENERABLE_SYNTHETIC_COPY_LOCAL_ONLY'])
                else:candidates.add(path)
    # Preserve exact 43 bound artifacts; do not traverse 1344/1414 source lists.
    for d in artifacts.values():
        require(descriptor(d['path'])==d,'bound 72 artifact changed')
        candidates.add(Path(d['path']))
    prep=ROOT/'next-motor-construction-preparation/final-changed-files-and-patches-v2.json'
    export=json.loads(prep.read_bytes())
    source_names=set(frozen['prepared_binding']['inputs']['transition']['after_source_sha256'])
    source_names.update(row['path'] for row in export['files'])
    source_names.update('examples/143_motor_construction/'+name for name in ['Cargo.toml','lib.rs','main.rs','tests.rs'])
    source_names.add('third_party/rapier3d/Cargo.lock')
    source_descriptors=[]
    for name in sorted(source_names):
        path=REPO/name;d=descriptor(path)
        if name in guard:
            require(d['sha256']==guard[name]['sha256'] and d['size_bytes']==guard[name]['size_bytes'],'source differs from actual 72 guard')
        else:
            require(name=='third_party/rapier3d/RNE_PATCH.md' and d['sha256']=='ada06e874021e6310795838a0adaecb5a63ff27aef2fdf6e2d1a31b415bd9d3e','unguarded source/doc not admitted')
        candidates.add(path);source_descriptors.append(d)
    candidates.add(OWN/'prepare_literal_export.py')
    candidates.add(OWN/'verify_literals.py')
    candidates.add(OWN/'test_export_guards.py')
    candidates.add(OWN/'prepare_literal_export-before-six-and-test-closure.py')
    for folder in sorted(OWN.glob('pure-tests-attempt-*')):
        for name in ('terminal.json','stdout.log','stderr.log'):
            if (folder/name).is_file():candidates.add(folder/name)
    six_raw=[descriptor(path) for path in sorted((ROOT/'next-motor-construction-six/collection').glob('*/raw.jsonl.gz'))]
    if six_raw:require(len(six_raw)==6,'six retention must include all six whole gzip, not a subset')
    six_audit_path=ROOT/'next-motor-construction-six/actual-cohort-audit-v2.json'
    six_audit=descriptor(six_audit_path) if six_audit_path.is_file() else None
    if six_audit:require(six_audit['sha256']=='a3f3e60fb199390e1dd761f6e96357df684bbcb0bdc996d5ec24a0745c3e107c' and six_audit['size_bytes']==260603,'actual six audit descriptor differs')
    rows=[];large=[];media=None;by_bytes={};payloads=[]
    for path in sorted(candidates,key=str):
        d=descriptor(path)
        if str(path)==str(ROOT/'next-motor-source-transition-72/collection/report.json'):
            require(d['sha256']=='bf6a721ca92d609c2e80df2e3b7f1bddeb79fcb5974934351d615a88f18db662','new scalar report differs')
            media={**d,'repository_path':MEDIA,'status':'COMPLETE_PRIMARY_REPORT_PLANNED_NOT_COPIED'};continue
        with path.open('rb') as f:magic=f.read(4)
        if d['size_bytes']>LIMIT or magic==b'\x7fELF':
            status = 'ALREADY_PUBLIC_PRIMARY_REPORT_UNCHANGED_NOT_RECOPIED' if path==Path(artifacts['baseline_report']['path']) else 'LOCAL_ONLY_NOT_DISTRIBUTED'
            large.append({**d,'status':status,'reason':'native binary' if magic==b'\x7fELF' else 'whole file exceeds evidence inline limit'})
            continue
        key=(d['sha256'],d['size_bytes'])
        if key not in by_bytes:
            suffix=path.suffix.lower()
            if not suffix.isascii() or not suffix or len(suffix)>10:suffix='.bin'
            rel='verify_literals.py' if path==OWN/'verify_literals.py' else f'f/{len(payloads)+1:06d}{suffix}'
            by_bytes[key]=rel;payloads.append({**d,'export_path':rel})
        rows.append([d['path'],d['sha256'],d['size_bytes'],by_bytes[key]])
    # Metadata are newly generated categorical receipts, not split raw artifacts.
    origins={'kind':'rne_literal_export_origin_map','schema_version':1,
             'columns':['full_original_path','whole_sha256','whole_size_bytes','short_export_path'],
             'rows':rows,'absolute_original_paths_preserved_in_literal_snapshots':True}
    omissions={'kind':'rne_literal_export_omitted_regenerable_copies','schema_version':1,
               'columns':['full_original_path','whole_sha256','whole_size_bytes','reason'],
               'rows':omitted,'originals_retained_locally':True,'not_claimed_publicly_available':True,
               'whole_compilation_cache_groups_not_hashed_or_exported':sorted(skipped_groups)}
    origin_bytes,omission_bytes=canonical(origins),canonical(omissions)
    require(len(origin_bytes)<=LIMIT,'whole origin manifest exceeds 1MiB; do not split it')
    require(len(omission_bytes)<=LIMIT,'whole omission metadata exceeds 1MiB; do not split it')
    publicpaths=[f'{PUBLIC}/{p["export_path"]}' for p in payloads]+[PUBLIC+'/origin-map.json',PUBLIC+'/omitted-copies.json',PUBLIC+'/.gitattributes',PUBLIC+'/README.md']
    require(all(path.isascii() and len(path)<=166 for path in publicpaths),'Windows checkout path limit')
    require(len(set(path.casefold() for path in publicpaths))==len(publicpaths),'casefold path collision')
    require(media is not None,'whole scalar report missing')
    readme = (OWN/'README-export-template.md').read_bytes()
    require(len(readme)<=LIMIT,'whole README exceeds limit')
    metadata_bytes={'origin-map.json':origin_bytes,'omitted-copies.json':omission_bytes,'.gitattributes':b'* -text\n','README.md':readme}
    generated_metadata=[{'path':str(output/name),'export_path':name,'sha256':hashlib.sha256(data).hexdigest(),'size_bytes':len(data)} for name,data in metadata_bytes.items()]
    result={'kind':'rne_motor_construction_literal_export_plan','schema_version':1,
      'status':'PREPARATION_ONLY_SIX_POSTCOLLECTION_CLOSURE_PENDING_NOT_PUBLISHED',
      'created_at':datetime.datetime.now(datetime.timezone.utc).isoformat(),
      'public_target':PUBLIC,'publication_ready':False,'six_success_claim_allowed':False,
      'basis72_frozen_sha256':'6a99b2ccb8573c1141369c5498bcea6e9bc8f61f62189988066cd7f020f2270b',
      'basis72_independent_review_sha256':'12b08d6018456e9b2a9fbf5959da8ad6e79f2dc3714f602428308990549d060d',
      'actual_six_whole_raw':six_raw,'actual_six_audit':six_audit,
      'six_retention_scope':'Whole-byte retention only; no gzip decode, numerical parsing or scientific qualification by exporter.',
      'source_reconstruction_git_base':BASE,'current_source_snapshots':source_descriptors,
      'literal_payloads':payloads,'generated_metadata':generated_metadata,'whole_scalar_primary_media':media,'excluded_whole_large_or_native':large,
      'origin_map_size_bytes':len(origin_bytes),'omitted_copy_manifest_size_bytes':len(omission_bytes),
      'source_origin_count':len(rows),'literal_payload_count':len(payloads),'omitted_regenerable_copy_count':len(omitted),
      'max_planned_evidence_repository_path_chars':max(map(len,publicpaths)),
      'scope_notes':['Native compiler binaries are locally retained and not distributed. Hash receipts do not make them publicly available.',
       '1344/1414 input guard JSON lists are preserved literally; unrelated source snapshots are not auto-copied; compiler closure remains non-exhaustive.',
       'Historical frozen /workspace paths remain unmodified. Public ordinal mapping supports byte verification, not direct replay of machine-specific frozen registrations.',
       'All payload files are whole originals with hash and size; no truncation, trace splitting or token remapping.',
       'Old scalar DB0 and all old failed cohorts remain untouched; new scalar is whole in legitimate docs/media primary-report path.',
       'No new archive upload/tag/publication is planned or authorized by this preparation.',
       'Complete actual six gzip/claim/terminal/admission/reader audit is mandatory after collection. Success requires actual root exit0 and passed qualification; missing or unknown outcomes stay retained and unqualified.'],
      'missing_publication_closure':(['actual-six-whole-gzip-and-original-process-receipts'] if not six_raw else [])+['actual-six-independent-postcollection-qualified-admission','root-approved-final-source-and-publication-origin-manifest'],
      'tool':descriptor(Path(__file__).resolve())}
    output.mkdir()
    for name,data in metadata_bytes.items():write_new(output/name,data)
    write_new(output/'plan.json',canonical(result))
    print(json.dumps(descriptor(output/'plan.json'),sort_keys=True))
    print(json.dumps({k:result[k] for k in ['literal_payload_count','source_origin_count','omitted_regenerable_copy_count','origin_map_size_bytes','omitted_copy_manifest_size_bytes','max_planned_evidence_repository_path_chars']},sort_keys=True))

def copy_literals(plan_path, expected_sha, destination):
    d=descriptor(plan_path);require(d['sha256']==expected_sha,'plan SHA mismatch')
    data=json.loads(Path(plan_path).read_bytes());destination=external(destination)
    require(data['kind']=='rne_motor_construction_literal_export_plan','wrong plan kind')
    require(data['status']=='PREPARATION_ONLY_SIX_POSTCOLLECTION_CLOSURE_PENDING_NOT_PUBLISHED','unsupported finalization status')
    # Always provisional and external. It cannot publish or write the checkout.
    all_rows=data['literal_payloads']+data.get('generated_metadata',[])
    paths=[p['export_path'] for p in all_rows]
    require(len(set(p.casefold() for p in paths))==len(paths),'duplicate/casefold output')
    for row in all_rows:
        rel=Path(row['export_path'])
        require(not rel.is_absolute() and '..' not in rel.parts and str(rel)==row['export_path'] and str(rel).isascii(),'unsafe export path')
        require(len(PUBLIC+'/'+str(rel))<=166,'Windows checkout path limit')
        require(row['size_bytes']<=LIMIT,'whole inline file exceeds limit')
        require(descriptor(row['path'])=={k:row[k] for k in ['path','sha256','size_bytes']},'stale input')
    destination.mkdir()
    for row in all_rows:
        target=destination/row['export_path'];target.parent.mkdir(exist_ok=True,parents=True)
        with Path(row['path']).open('rb') as src, target.open('xb') as dst:
            shutil.copyfileobj(src,dst,1<<20);dst.flush();os.fsync(dst.fileno())
        actual=descriptor(target)
        require(actual['sha256']==row['sha256'] and actual['size_bytes']==row['size_bytes'],'copy changed; retained partial staging')
    for row in all_rows:
        require(descriptor(row['path'])=={k:row[k] for k in ['path','sha256','size_bytes']},'input changed during staging')
    print('PROVISIONAL_EXTERNAL_LITERALS_COPIED_NO_PUBLICATION_NO_SIX_QUALIFICATION')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    p=sub.add_parser('plan');p.add_argument('--output',type=Path,required=True)
    c=sub.add_parser('copy-literals');c.add_argument('--plan',type=Path,required=True);c.add_argument('--expected-plan-sha256',required=True);c.add_argument('--destination',type=Path,required=True)
    args=parser.parse_args()
    if args.command=='plan':plan(args.output)
    else:copy_literals(args.plan,args.expected_plan_sha256,args.destination)
