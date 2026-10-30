"""Offline null-space contact release after a failed frozen Newton solve.

The intermediate release is only a new starting point. Original-law and energy
checks are mandatory for every successful result; no production solver change.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import contact_frozen_newton as base


def project(p,mu):
    p=np.asarray(p,dtype=float).reshape(-1,3).copy()
    p[:,2]=np.maximum(p[:,2],0)
    length=np.linalg.norm(p[:,:2],axis=1)
    ratio=np.ones_like(length)
    np.divide(mu*p[:,2],length,out=ratio,where=length>0)
    p[:,:2]*=np.minimum(1,ratio)[:,None]
    return p.ravel()


def release(g,b,mu,p,nullspace,vt,it):
    """Try removing each positively separating loaded point; choose lowest merit."""
    velocity=(g@p+b).reshape(-1,3);loads=p.reshape(-1,3)[:,2]
    best=None;evaluated=0
    if nullspace.shape[1]==0:return None,0
    for i in range(len(mu)):
        if velocity[i,2]<=vt or loads[i]<=it:continue
        local=nullspace[3*i:3*i+3]
        correction=nullspace@np.linalg.lstsq(local,p[3*i:3*i+3],rcond=1e-12)[0]
        candidate=project(p-correction,mu)
        candidate[3*i:3*i+3]=0
        if not np.isfinite(candidate).all():continue
        f,_=base.residual_jacobian(g,b,mu,candidate);merit=float(f@f)
        evaluated+=1
        if np.isfinite(merit) and (best is None or merit<best['merit']):
            best=dict(contact=i,merit=merit,impulses=candidate,
                      correction_velocity_inf=float(np.max(np.abs(g@correction))),
                      projected_velocity_change_inf=float(np.max(np.abs(g@(candidate-p)))))
    return best,evaluated


def solve(case,start,max_iterations=50,max_releases=8):
    if any(type(x) is not int or x<0 for x in [max_iterations,max_releases]):
        raise ValueError('budgets must be nonnegative integers')
    g,b,mu=base.system(case);p=np.asarray(start,dtype=float).reshape(-1).copy()
    if p.shape!=b.shape or not np.isfinite(p).all():raise ValueError('invalid seed')
    nullspace=None;total=0;attempts=[];releases=[];evaluated=0
    for restart in range(max_releases+1):
        r=base.solve(case,p,max_iterations=max_iterations-total)
        total+=r['iterations'];p=np.asarray(r['impulses']).ravel()
        f,j=base.residual_jacobian(g,b,mu,p)
        attempts.append(dict(reason=r['reason'],iterations=r['iterations'],
                             merit_history=r['merit_history'],audit=r['audit'],
                             gradient_inf=float(np.max(np.abs(j.T@f)))))
        if r['audit']['passed']:reason='certified';break
        if total>=max_iterations:reason='iteration_limit';break
        if restart==max_releases:reason='release_limit';break
        if nullspace is None:
            _,s,v=np.linalg.svd(g,full_matrices=False)
            nullspace=v[s<1e-12*s[0]].T
        candidate,count=release(g,b,mu,p,nullspace,case['vt'],case['it']);evaluated+=count
        if candidate is None:reason='no_separating_loaded_contact';break
        p=candidate.pop('impulses');releases.append(candidate)
    return dict(impulses=p.reshape(-1,3).tolist(),iterations=total,reason=reason,
                attempts=attempts,releases=releases,release_candidates=evaluated,
                audit=base.audit(g,b,mu,p,case['vt'],case['it']))


def report(fixtures):
    data=json.loads(fixtures.read_text())
    labels=[c['label'] for c in data['cases']]
    if data['protocol']!='frozen_contact_systems_v1' or not labels or len(labels)!=len(set(labels)):
        raise ValueError('fixture protocol or labels differ')
    rows=[]
    for c in data['cases']:
        row=dict(label=c['label'])
        for name in ['seed','gs']:row[name]=solve(c,c[name])
        rows.append(row)
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    return dict(protocol='frozen_contact_release_v1',fixtures_sha256=digest(fixtures),
        source_sha256={Path(p).name:digest(p) for p in [__file__,base.__file__]},
        numpy_version=np.__version__,max_iterations=50,max_releases=8,results=rows)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('fixtures',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--verify',action='store_true');a=p.parse_args()
    if a.output.exists() and not a.verify:p.error('new output required')
    result=report(a.fixtures)
    if a.verify:
        if json.loads(a.output.read_text())!=result:raise ValueError('result or provenance differs')
        print('Verified all releases, Newton solves, independent audits and hashes');return
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for row in result['results']:
        print(row['label'],[(k,row[k]['reason'],row[k]['iterations'],len(row[k]['releases']))
                            for k in ['seed','gs']])


if __name__=='__main__':main()
