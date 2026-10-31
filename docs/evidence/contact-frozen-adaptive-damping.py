"""Offline adaptive damping for the frozen Coulomb Newton experiment."""
import numpy as np
import contact_frozen_newton as base


def solve(case,start,max_iterations=50):
    if type(max_iterations) is not int or max_iterations<0:raise ValueError('invalid iteration limit')
    g,b,mu=base.system(case);p=np.asarray(start,dtype=float).reshape(-1).copy()
    if p.shape!=b.shape or not np.isfinite(p).all():raise ValueError('invalid seed')
    damping=0.;iterations=0;history=[];trials=[];reason='iteration_limit'
    for iteration in range(max_iterations+1):
        q=base.audit(g,b,mu,p,case['vt'],case['it'])
        f,j=base.residual_jacobian(g,b,mu,p);merit=float(f@f);history.append(merit)
        if q['passed']:reason='certified';break
        if iteration==max_iterations:break
        u,s,transpose=np.linalg.svd(j,full_matrices=False);rhs=u.T@(-f)
        floor=1e-12*s[0]**2
        accepted=False
        for attempt in range(24):
            if damping==0:
                coefficients=np.zeros_like(s)
                np.divide(1.,s,out=coefficients,where=s>1e-12*s[0])
            else:coefficients=s/(s*s+damping)
            direction=transpose.T@(coefficients*rhs);candidate=p+direction
            residual,_=base.residual_jacobian(g,b,mu,candidate)
            value=float(residual@residual)
            model=f+j@direction;predicted=merit-float(model@model)
            ratio=(merit-value)/predicted if predicted>0 else -1.
            if np.isfinite(value) and value<merit and ratio>1e-4:
                p=candidate;accepted=True;iterations+=1
                trials.append(dict(attempts=attempt+1,damping=float(damping),ratio=float(ratio)))
                if ratio>.75:damping*=.1
                elif ratio<.25:damping=max(floor,2*damping)
                break
            damping=max(floor,10*damping)
        if not accepted:reason='trust_step_failed';break
    return dict(impulses=p.reshape(-1,3).tolist(),iterations=iterations,reason=reason,
                merit_history=history,trials=trials,
                audit=base.audit(g,b,mu,p,case['vt'],case['it']))


if __name__=='__main__':
    import argparse
    import hashlib
    import json
    from pathlib import Path
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('fixtures',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args()
    if args.output.exists():parser.error('new output required')
    rows=[]
    for case in json.loads(args.fixtures.read_text())['cases']:
        row=dict(label=case['label'])
        for name in ['seed','gs']:row[name]=solve(case,case[name])
        rows.append(row)
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    result=dict(protocol='frozen_contact_adaptive_damping_trial_v1',
        fixtures_sha256=digest(args.fixtures),
        source_sha256={Path(p).name:digest(p) for p in [__file__,base.__file__]},
        numpy_version=np.__version__,results=rows)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
