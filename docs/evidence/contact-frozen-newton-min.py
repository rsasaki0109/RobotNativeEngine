"""Offline semismooth Newton experiment on frozen rigid Coulomb systems.

No runtime integration or performance claim. NumPy least-squares handles singular
linearizations; all candidates are checked against the original equations.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def system(case):
    mu=np.asarray(case['mu'],dtype=float)
    n=mu.size; g=np.asarray(case['g'],dtype=float).reshape(3*n,3*n)
    free=np.asarray(case['free'],dtype=float)
    if (not 1<=n<=64 or free.shape!=(3*n,) or mu.shape!=(n,)
            or not np.isfinite(g).all() or not np.isfinite(free).all()
            or not np.isfinite(mu).all() or np.any(mu<0)
            or not np.isfinite(case['vt']) or case['vt']<=0
            or not np.isfinite(case['it']) or case['it']<0
            or case['regularization']!=0 or not np.allclose(g,g.T,rtol=0,atol=1e-12)
            or np.any(np.diag(g)<=0)):
        raise ValueError('expected a finite symmetric rigid contact system, 1..64 contacts')
    return g,free,mu


def residual_jacobian(g,free,mu,p):
    """min(pn, alpha*vn) and pt - disk(pt-alpha*vt, mu*max(pn,0))."""
    v=g@p+free; eye=np.eye(len(p)); f=np.empty_like(p); j=np.empty_like(g)
    for i,m in enumerate(mu):
        k=3*i; alpha=1/max(np.diag(g)[k:k+3])
        y=p[k:k+2]-alpha*v[k:k+2]; length=np.linalg.norm(y)
        radius=m*max(p[k+2],0)
        if length<=radius:
            f[k:k+2]=alpha*v[k:k+2];j[k:k+2]=alpha*g[k:k+2]
        else:
            direction=y/length
            derivative=(radius/length)*(np.eye(2)-np.outer(direction,direction))
            f[k:k+2]=p[k:k+2]-radius*direction
            j[k:k+2]=eye[k:k+2]-derivative@(eye[k:k+2]-alpha*g[k:k+2])
            if p[k+2]>0:j[k:k+2,k+2]-=m*direction
        if p[k+2]<=alpha*v[k+2]:
            f[k+2]=p[k+2];j[k+2]=eye[k+2]
        else:
            f[k+2]=alpha*v[k+2];j[k+2]=alpha*g[k+2]
    return f,j


def audit(g,free,mu,p,vt,it):
    """Independent physical tests, including an absolute dissipation error gate."""
    p=np.asarray(p).reshape(-1,3);v=(g@p.ravel()+free).reshape(-1,3)
    if not np.isfinite(p).all() or not np.isfinite(v).all():
        raise ValueError('nonfinite candidate')
    tangent=np.linalg.norm(p[:,:2],axis=1); slip=np.linalg.norm(v[:,:2],axis=1)
    limit=mu*np.maximum(p[:,2],0);loaded=p[:,2]>it
    interior=loaded & (limit-tangent>it)
    maxima=dict(negative_normal_impulse_n_s=float(np.maximum(-p[:,2],0).max()),
        closing_velocity_m_s=float(np.maximum(-v[:,2],0).max()),
        loaded_normal_velocity_m_s=float(np.abs(v[loaded,2]).max()) if loaded.any() else 0.,
        cone_violation_n_s=float(np.maximum(tangent-limit,0).max()),
        interior_slip_m_s=float(slip[interior].max()) if interior.any() else 0.,
        dissipation_error_j=float(np.abs(tangent*slip+np.sum(p[:,:2]*v[:,:2],axis=1)).max()))
    # Directly verify sliding opposition too, independent of Newton residual.
    errors=[]
    for i in range(len(mu)):
        if limit[i]>it and slip[i]>np.sqrt(2)*vt:
            if tangent[i]==0:errors.append(float(slip[i]))
            else:errors.append(float(np.linalg.norm(v[i,:2]+slip[i]*p[i,:2]/tangent[i])))
    maxima['sliding_direction_error_m_s']=max(errors,default=0.)
    limits=dict(negative_normal_impulse_n_s=it,closing_velocity_m_s=vt,
        loaded_normal_velocity_m_s=vt,cone_violation_n_s=it,
        interior_slip_m_s=np.sqrt(2)*vt,dissipation_error_j=1e-10,
        sliding_direction_error_m_s=np.sqrt(2)*vt)
    gates={k:bool(val<=limits[k]) for k,val in maxima.items()}
    return dict(maxima=maxima,gates=gates,passed=all(gates.values()))


def solve(case,start,max_iterations=50):
    if type(max_iterations) is not int or max_iterations<0:raise ValueError('invalid iteration limit')
    g,b,mu=system(case);p=np.asarray(start,dtype=float).reshape(-1).copy()
    if p.shape!=b.shape or not np.isfinite(p).all():raise ValueError('invalid seed')
    history=[];reason='iteration_limit';iterations=0
    for iteration in range(max_iterations+1):
        q=audit(g,b,mu,p,case['vt'],case['it'])
        f,j=residual_jacobian(g,b,mu,p);merit=float(f@f)
        history.append(merit)
        if q['passed']:reason='certified';break
        if iteration==max_iterations:break
        direction=np.linalg.lstsq(j,-f,rcond=1e-12)[0]
        accepted=False
        for backtrack in range(12):
            candidate=p+2.**(-backtrack)*direction
            trial,_=residual_jacobian(g,b,mu,candidate)
            if np.isfinite(trial).all() and trial@trial<merit*(1-1e-4*2.**(-backtrack)):
                p=candidate;accepted=True;iterations+=1;break
        if not accepted:reason='line_search_failed';break
    return dict(impulses=p.reshape(-1,3).tolist(),iterations=iterations,reason=reason,
                merit_history=history,audit=audit(g,b,mu,p,case['vt'],case['it']))


def evaluate(case):
    g,b,mu=system(case)
    return dict(gs_audit=audit(g,b,mu,np.asarray(case['gs']),case['vt'],case['it']),
        from_seed=solve(case,case['seed']),from_gs=solve(case,case['gs']))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('fixtures',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args()
    if a.output.exists():p.error('new output required')
    cases=json.loads(a.fixtures.read_text())
    results=[dict(label=c['label'],result=evaluate(c)) for c in cases['cases']]
    report=dict(protocol='frozen_contact_newton_v1',
        fixtures_sha256=hashlib.sha256(a.fixtures.read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        numpy_version=np.__version__,results=results)
    a.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    for row in results:
        print(row['label'],[(k,row['result'][k]['reason'],row['result'][k]['iterations'])
                            for k in ['from_seed','from_gs']])


if __name__=='__main__':main()
