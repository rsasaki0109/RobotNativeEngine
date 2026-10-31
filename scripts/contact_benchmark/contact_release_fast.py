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


def newton_solve(case,start,max_iterations=50):
    if type(max_iterations) is not int or max_iterations<0:raise ValueError('invalid iteration limit')
    g,b,mu=base.system(case);p=np.asarray(start,dtype=float).reshape(-1).copy()
    if p.shape!=b.shape or not np.isfinite(p).all():raise ValueError('invalid seed')
    history=[];reason='iteration_limit';iterations=0
    for iteration in range(max_iterations+1):
        q=base.audit(g,b,mu,p,case['vt'],case['it'])
        f,j=base.residual_jacobian(g,b,mu,p);merit=float(f@f)
        history.append(merit)
        if q['passed']:reason='certified';break
        if iteration==max_iterations:break
        u,s,transpose=np.linalg.svd(j,full_matrices=False)
        rhs=u.T@(-f)
        inverse=np.zeros_like(s);np.divide(1.,s,out=inverse,where=s>1e-12*s[0])
        direction=transpose.T@(inverse*rhs)
        radius=1+np.linalg.norm(p)
        damping=1e-12*s[0]**2
        # Damping the Newton step does not alter G, free velocity or the law.
        for _ in range(30):
            if np.linalg.norm(direction)<=radius:break
            direction=transpose.T@((s/(s*s+damping))*rhs)
            damping*=10
        accepted=False
        for backtrack in range(12):
            candidate=p+2.**(-backtrack)*direction
            trial=residual(g,b,mu,candidate)
            if np.isfinite(trial).all() and trial@trial<merit*(1-1e-4*2.**(-backtrack)):
                p=candidate;accepted=True;iterations+=1;break
        if not accepted:reason='line_search_failed';break
    return dict(impulses=p.reshape(-1,3).tolist(),iterations=iterations,reason=reason,
                merit_history=history,audit=base.audit(g,b,mu,p,case['vt'],case['it']))



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
        f=residual(g,b,mu,candidate);merit=float(f@f)
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
        r=newton_solve(case,p,max_iterations=max_iterations-total)
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

