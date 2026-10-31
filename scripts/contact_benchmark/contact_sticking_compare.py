"""Paired disabled/enabled evaluation of the experimental sticking block path."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import contact_stopping_compare as comparison
import contact_crowd as base
import contact_tower as tower

CASES = comparison.CASES
SOURCES = ['contact_sticking_compare.py'] + comparison.SOURCES


def run(binary,count,layout,mode,enabled):
    args=[str(binary.resolve()),str(count),mode,layout]
    if enabled:args+=['1','sticking-block']
    return json.loads(subprocess.run(args,check=True,capture_output=True,text=True,timeout=120).stdout)


def summary(row):
    for name,enabled in [('before',False),('after',True)]:
        for a in [row[name]]+row['timing'][name]:
            if a.get('experimental_sticking_block',False) is not enabled:
                raise ValueError('sticking mode differs')
    return comparison.summary(row)


def hashes():return {n:base.digest(Path(__file__).with_name(n)) for n in SOURCES}


def verify(report,binary):
    if (report['protocol']!='contact_sticking_compare_v1' or report['source_sha256']!=hashes()
            or report['binary_sha256']!=base.digest(binary)
            or report['settings']!=dict(before=False,after=True)
            or [(r['layout'],r['count']) for r in report['results']]!=CASES):
        raise ValueError('provenance or cases differ')
    for row in report['results']:
        if row['summary']!=summary(row):raise ValueError('summary differs')
        for name,enabled in report['settings'].items():
            tower.equal_audits(row[name],run(binary,row['count'],row['layout'],'audit',enabled))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('binary',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--verify',action='store_true');p.add_argument('--first',choices=['before','after'],default='before')
    a=p.parse_args()
    if a.verify:
        verify(json.loads(a.output.read_text()),a.binary);print('Verified mode, quality, costs and fresh per-mode replays');return
    if a.output.exists():p.error('new output required')
    os.sched_setaffinity(0,{min(os.sched_getaffinity(0))})
    settings=dict(before=False,after=True)
    report=dict(protocol='contact_sticking_compare_v1',source_sha256=hashes(),
        binary_sha256=base.digest(a.binary),settings=settings,first=a.first,
        cpu_affinity=sorted(os.sched_getaffinity(0)),results=[])
    order=['before','after'] if a.first=='before' else ['after','before']
    for layout,count in CASES:
        row=dict(layout=layout,count=count,timing=dict(before=[],after=[]))
        for name,enabled in settings.items():
            row[name]=run(a.binary,count,layout,'audit',enabled)
            run(a.binary,count,layout,'timing',enabled)
        for i in range(5):
            for name in (order if i%2==0 else order[::-1]):
                row['timing'][name].append(run(a.binary,count,layout,'timing',settings[name]))
        row['summary']=summary(row);report['results'].append(row)
        print(json.dumps(dict(layout=layout,count=count,summary=row['summary'])),flush=True)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');verify(report,a.binary)


if __name__=='__main__':main()
