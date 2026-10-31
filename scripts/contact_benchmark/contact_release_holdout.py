"""Held-out frozen solves and paired Python costs for contact release.

These are new steps of the existing four scenes, not new scenes or runtime costs.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import time
import numpy as np
import contact_frozen_newton as newton
import contact_frozen_release as release


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def indices(count,excluded):
    remaining=[i for i in range(count) if i not in excluded]
    if len(remaining)<4:raise ValueError('four disjoint holdout cases required')
    return [remaining[((2*k+1)*len(remaining))//8] for k in range(4)]


def freeze(raw_directory,original,directory):
    if directory.exists():raise ValueError('new fixture directory required')
    data=json.loads(original.read_text());groups=[]
    # Validate every source before creating the output directory.
    for capture in data['captures']:
        path=raw_directory/f"{capture['layout']}-{capture['count']}.jsonl"
        if digest(path)!=capture['sha256']:raise ValueError('capture hash differs')
        rows=[json.loads(line) for line in path.read_text().splitlines()]
        if len(rows)!=capture['failed_solves']:raise ValueError('capture count differs')
        selected=indices(len(rows),capture['selected_indices']);cases=[]
        for i in selected:
            c=rows[i];newton.system(c)
            c['label']=f"{capture['layout']}-{capture['count']}-failed-solve-{i+1}"
            c['selection']='heldout_midpoint_quartiles_after_excluding_development_cases'
            cases.append(c)
        groups.append(dict(protocol='frozen_contact_systems_v1',
            original_fixtures_sha256=digest(original),raw_sha256=capture['sha256'],
            excluded_indices=capture['selected_indices'],selected_indices=selected,cases=cases))
    directory.mkdir(parents=True)
    for group,capture in zip(groups,data['captures']):
        path=directory/f"{capture['layout']}-{capture['count']}.json"
        path.write_text(json.dumps(group,indent=2,allow_nan=False)+'\n')
    print('Saved 16 held-out problems without inspecting solver outcomes')


def sources():
    return {Path(p).name:digest(p) for p in [__file__,newton.__file__,release.__file__]}


def summary(rows):
    result={}
    for name in ['baseline','release']:
        selected=[r[name] for r in rows]
        result[name]=dict(passed=sum(x['audit']['passed'] for x in selected),
            accepted_newton_steps=sum(x['iterations'] for x in selected),
            total_median_seconds=sum(statistics.median(r['timing'][name]) for r in rows))
    result['gained']=[r['label']+':'+r['start'] for r in rows
                      if not r['baseline']['audit']['passed'] and r['release']['audit']['passed']]
    result['lost']=[r['label']+':'+r['start'] for r in rows
                    if r['baseline']['audit']['passed'] and not r['release']['audit']['passed']]
    result['release_candidates']=sum(r['release']['release_candidates'] for r in rows)
    return result


def run(directory,output,verify=False,first="baseline"):
    files=sorted(directory.glob('*.json'))
    if len(files)!=4:raise ValueError('four fixture groups required')
    cases=[]
    for path in files:
        data=json.loads(path.read_text())
        if data['protocol']!='frozen_contact_systems_v1' or len(data['cases'])!=4:
            raise ValueError('fixture protocol or count differs')
        cases.extend(data['cases'])
    labels=[c['label'] for c in cases]
    if len(labels)!=len(set(labels)):raise ValueError('duplicate fixtures')
    metadata=dict(protocol='contact_release_holdout_v1',
        fixtures_sha256={p.name:digest(p) for p in files},source_sha256=sources(),
        numpy_version=np.__version__)
    if first not in ['baseline','release']:raise ValueError('invalid timing order')
    functions=dict(baseline=newton.solve,release=release.solve)
    if verify:
        report=json.loads(output.read_text())
        if any(report[k]!=v for k,v in metadata.items()):raise ValueError('provenance differs')
        if report['first'] not in functions:raise ValueError('invalid stored timing order')
        if [(r['label'],r['start']) for r in report['results']]!=[(c['label'],s) for c in cases for s in ['seed','gs']]:
            raise ValueError('coverage differs')
        for index,c in enumerate(cases):
            for offset,start in enumerate(['seed','gs']):
                r=report['results'][2*index+offset]
                for name,solve in functions.items():
                    if solve(c,c[start])!=r[name]:raise ValueError('fresh solve differs')
                    samples=r['timing'][name]
                    if len(samples)!=5 or any(not math.isfinite(t) or t<=0 for t in samples):
                        raise ValueError('invalid timings')
        if report['summary']!=summary(report['results']):raise ValueError('summary differs')
        print('Verified all 32 paired solves, timing metadata, summaries and hashes');return
    if output.exists():raise ValueError('new output required')
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    rows=[]
    for c in cases:
        for start in ['seed','gs']:
            row=dict(label=c['label'],start=start,timing=dict(baseline=[],release=[]))
            for name,solve in functions.items():row[name]=solve(c,c[start])
            order=[first,next(name for name in functions if name!=first)]
            if len(rows)%2:order.reverse()
            for repeat in range(5):
                for name in order if repeat%2==0 else order[::-1]:
                    begin=time.perf_counter();result=functions[name](c,c[start]);elapsed=time.perf_counter()-begin
                    if result!=row[name]:raise ValueError('timed solve differs')
                    row['timing'][name].append(elapsed)
            rows.append(row)
            print(row['label'],start,row['baseline']['audit']['passed'],row['release']['audit']['passed'],flush=True)
    report=dict(**metadata,first=first,cpu_affinity=sorted(os.sched_getaffinity(0)),results=rows,summary=summary(rows))
    output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    f=sub.add_parser('freeze');f.add_argument('raw_directory',type=Path);f.add_argument('original',type=Path);f.add_argument('directory',type=Path)
    r=sub.add_parser('run');r.add_argument('directory',type=Path);r.add_argument('output',type=Path);r.add_argument('--verify',action='store_true');r.add_argument('--first',choices=['baseline','release'],default='baseline')
    a=p.parse_args()
    if a.command=='freeze':freeze(a.raw_directory,a.original,a.directory)
    else:run(a.directory,a.output,a.verify,a.first)


if __name__=='__main__':main()
