"""Offline release solver: evaluate candidate merit without unused Jacobians."""
import numpy as np
import contact_frozen_newton as base


def residual(g,free,mu,p):
    """Same arithmetic as the reference residual, without its Jacobian work."""
    v=g@p+free;f=np.empty_like(p)
    for i,m in enumerate(mu):
        k=3*i;alpha=1/max(np.diag(g)[k:k+3])
        y=p[k:k+2]-alpha*v[k:k+2];length=np.linalg.norm(y)
        radius=m*max(p[k+2],0)
        if length<=radius:f[k:k+2]=alpha*v[k:k+2]
        else:
            direction=y/length
            f[k:k+2]=p[k:k+2]-radius*direction
        normal=p[k+2];speed=alpha*v[k+2];length=np.hypot(normal,speed)
        f[k+2]=length-normal-speed
    return f


def project(p,mu):
    p=np.asarray(p,dtype=float).reshape(-1,3).copy()
    p[:,2]=np.maximum(p[:,2],0)
    length=np.linalg.norm(p[:,:2],axis=1)
    ratio=np.ones_like(length)
    np.divide(mu*p[:,2],length,out=ratio,where=length>0)
    p[:,:2]*=np.minimum(1,ratio)[:,None]
    return p.ravel()


def release(g,b,mu,p,nullspace,vt,it,cap):
    """Try removing each positively separating loaded point; choose lowest merit."""
    velocity=(g@p+b).reshape(-1,3);loads=p.reshape(-1,3)[:,2]
    best=None;evaluated=0
    if nullspace.shape[1]==0:return None,0
    eligible=[i for i in range(len(mu)) if velocity[i,2]>vt and loads[i]>it]
    eligible.sort(key=lambda i:(-velocity[i,2]*loads[i],i))
    for i in eligible[:cap]:
        local=nullspace[3*i:3*i+3]
        correction=nullspace@np.linalg.lstsq(local,p[3*i:3*i+3],rcond=1e-12)[0]
        candidate=project(p-correction,mu)
        candidate[3*i:3*i+3]=0
        if not np.isfinite(candidate).all():continue
        f=residual(g,b,mu,candidate);merit=float(f@f)
        evaluated+=1
        if np.isfinite(merit) and (best is None or merit<best['merit']):
            best=dict(contact=i,merit=merit,impulses=candidate,
                      correction_velocity_inf=float(np.max(np.abs(g@correction))),
                      projected_velocity_change_inf=float(np.max(np.abs(g@(candidate-p)))))
    return best,evaluated


def solve(case,start,max_iterations=50,max_releases=8,cap=4):
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
        candidate,count=release(g,b,mu,p,nullspace,case['vt'],case['it'],cap);evaluated+=count
        if candidate is None:reason='no_separating_loaded_contact';break
        p=candidate.pop('impulses');releases.append(candidate)
    return dict(impulses=p.reshape(-1,3).tolist(),iterations=total,reason=reason,
                attempts=attempts,releases=releases,release_candidates=evaluated,
                audit=base.audit(g,b,mu,p,case['vt'],case['it']))



if __name__=='__main__':
    import argparse
    import hashlib
    import json
    from pathlib import Path
    import contact_frozen_release as reference
    parser=argparse.ArgumentParser(description='Screen capped candidates on already-seen problems')
    parser.add_argument('development',type=Path);parser.add_argument('previous_directory',type=Path)
    parser.add_argument('output',type=Path);args=parser.parse_args()
    if args.output.exists():parser.error('new output required')
    files=[args.development]+sorted(args.previous_directory.glob('*.json'))
    cases=[]
    for path in files:cases.extend(json.loads(path.read_text())['cases'])
    rows=[]
    for case in cases:
        for start in ['seed','gs']:
            row=dict(label=case['label'],start=start,reference=reference.solve(case,case[start]))
            for cap in [1,4]:row['cap'+str(cap)]=solve(case,case[start],cap=cap)
            rows.append(row)
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    result=dict(protocol='contact_release_cap_screen_v1',
        sources={Path(p).name:digest(p) for p in [__file__,base.__file__,reference.__file__]},
        inputs={str(p):digest(p) for p in files},numpy_version=np.__version__,results=rows)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
