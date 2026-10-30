"""Candidate retention and model-specific contact optimality audit over 50 ms."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
import force_friction_matched as fixture
import native_contact_diagnostics as native

CASE=dict(dt_ticks=8000,mass_kg=3,force_ratio=1.2,mu=0.5)
REFS=[[.02,1],[.005,1],[2e-6,10]]
SETTINGS=[dict(solref=r,margin_m=0.0,gap_m=0.0) for r in REFS]+[dict(solref=[.02,1],margin_m=1e-4,gap_m=1e-4)]
COLUMNS=['step','contact_index','fn','ft1','ft2','gn','gt1','gt2']
STEP_COLUMNS=['end_time_s','geometric_contacts','constraint_contacts','loaded_contacts','solver_iterations','spin_rad_s']
SOURCES=['contact_constraint_audit.py','native_contact_diagnostics.py','force_friction_matched.py','force_friction.py']


def native_summary(audit):
    native.summarize(audit)
    if audit['settings']['regularization']!=0:
        raise ValueError('rigid residual audit requires zero regularization')
    normal_violation=complementarity=cone=0.0
    counts=[];iterations=[];converged=[]
    for row in audit['contact_diagnostics']:
        c=row['pipeline'];counts.append([c['generated'],c['deduplicated'],c['retained']])
        if not c['generated']>=c['deduplicated']>=c['retained']==len(row['points']):
            raise ValueError('candidate counts differ')
        for p in row['points']:
            n=np.asarray(p['normal_world']);v=np.asarray(p['relative_velocity_world_m_s'])
            jn=p['normal_impulse_n_s'];jt=np.asarray(p['tangential_impulse_world_n_s'])
            w=float(n@v+p['gap_m']/p['dt_s']*(1 if p['gap_m']>=0 else audit['settings']['penetration_recovery']))
            if abs(w-p['normal_constraint_velocity_m_s'])>1e-12:
                raise ValueError('normal constraint velocity differs')
            normal_violation=max(normal_violation,-w)
            complementarity=max(complementarity,abs(jn*w))
            cone=max(cone,np.linalg.norm(jt)-p['friction_coefficient']*jn,-jn)
            iterations.append(p['solver_iterations']);converged.append(p['solver_converged'])
    return dict(candidate_counts=counts,max_normal_velocity_violation_m_s=normal_violation,
                max_normal_complementarity_n_m=complementarity,max_friction_cone_excess_n_s=float(cone),
                max_solver_iterations=max(iterations),all_solver_stopping_flags=all(converged))


def run_mujoco(ref,margin_m=0.0,gap_m=0.0):
    xml=fixture.model_xml(CASE,ref).replace('margin="0" gap="0"',f'margin="{margin_m}" gap="{gap_m}"')
    model=mujoco.MjModel.from_xml_string(xml);data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    if mujoco.mj_isSparse(model):
        raise ValueError('this six-DOF fixture requires dense constraint Jacobian')
    points=[];steps=[]
    for i in range(6250):
        data.xfrc_applied[1,:]=[17.658,0,0,0,0,0]
        mujoco.mj_step(model,data)
        J=data.efc_J.reshape(data.nefc,model.nv)
        # Gradient of the soft constraint-space dual objective. Do not confuse
        # this acceleration-unit quantity with Native's rigid velocity residual.
        g=J@data.qacc-data.efc_aref+data.efc_R*data.efc_force
        active=loaded=0
        for k,contact in enumerate(data.contact):
            address=contact.efc_address
            if address<0:continue
            if contact.dim!=3:raise ValueError('expected elliptic three-dimensional contact')
            f=data.efc_force[address:address+3];gradient=g[address:address+3]
            points.append([i,k,*f,*gradient]);active+=1;loaded+=int(f[0]>1e-8)
        steps.append([(i+1)*model.opt.timestep,data.ncon,active,loaded,int(data.solver_niter[0]),float(np.linalg.norm(data.qvel[3:6]))])
    if any(w.number for w in data.warning):raise ValueError('MuJoCo warning')
    return np.asarray(steps),np.asarray(points)


def mj_summary(steps,points):
    if steps.shape!=(6250,6) or points.ndim!=2 or points.shape[1]!=8 or not np.isfinite(steps).all() or not np.isfinite(points).all():
        raise ValueError('invalid diagnostic arrays')
    if not np.allclose(steps[:,0],np.arange(1,6251)*8e-6,rtol=0,atol=1e-12):
        raise ValueError('clock differs')
    indices=points[:,0].astype(int)
    if not np.array_equal(indices,points[:,0]) or (indices<0).any() or (indices>=6250).any():
        raise ValueError('point index differs')
    count=np.zeros(6250);loaded=np.zeros(6250)
    np.add.at(count,indices,1);np.add.at(loaded,indices,points[:,2]>1e-8)
    if not np.array_equal(count,steps[:,2]) or not np.array_equal(loaded,steps[:,3]) or (steps[:,1]<count).any():
        raise ValueError('point counts differ')
    f=points[:,2:5];g=points[:,5:8];mu=CASE['mu']
    primal=np.maximum(0,np.linalg.norm(f[:,1:],axis=1)-mu*f[:,0])
    # Dual of ||ft|| <= mu fn is gn >= mu ||gt||.
    dual=np.maximum(0,mu*np.linalg.norm(g[:,1:],axis=1)-g[:,0])
    complementarity=np.abs(np.sum(f*g,axis=1))
    crossed=np.flatnonzero(steps[:,5]>.01)
    return dict(max_primal_cone_excess_n=float(primal.max()),max_dual_cone_violation_m_s2=float(dual.max()),
                max_soft_complementarity_n_m_s2=float(complementarity.max()),
                max_solver_iterations=int(steps[:,4].max()),iteration_cap_steps=int(np.sum(steps[:,4]>=200)),
                geometric_contact_counts=sorted(set(steps[:,1].astype(int).tolist())),
                constraint_contact_counts=sorted(set(steps[:,2].astype(int).tolist())),
                loaded_contact_counts=sorted(set(steps[:,3].astype(int).tolist())),
                first_spin_gate_time_s=float(steps[crossed[0],0]) if len(crossed) else None)


def verify(directory,binary):
    r=json.loads((directory/'report.json').read_text())
    if r['protocol']!='contact_constraint_audit_v1' or r['mujoco_version']!='3.9.0' or not r['observation_preserves_trace'] or r['case']!=CASE or [x['setting'] for x in r['mujoco']]!=SETTINGS or r['columns']!=COLUMNS or r['step_columns']!=STEP_COLUMNS:
        raise ValueError('protocol differs')
    if r['binary_sha256']!=hashlib.sha256(binary.read_bytes()).hexdigest():raise ValueError('binary differs')
    for name in SOURCES:
        if r['source_sha256'][name]!=hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest():raise ValueError('source differs')
    if native_summary(r['native'])!=r['native_summary']:raise ValueError('native residual differs')
    repeated=native.run(binary.resolve(),True)
    if any(repeated[k]!=r['native'][k] for k in ['trace','contact_diagnostics','settings']):raise ValueError('native replay differs')
    for i,row in enumerate(r['mujoco']):
        if row['file']!=f'constraints-{i}.npz' or not row['exact_replay']:raise ValueError('record identity differs')
        path=directory/row['file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('trace differs')
        with np.load(path,allow_pickle=False) as a:
            if mj_summary(a['steps'],a['points'])!=row['summary']:raise ValueError('soft residual differs')
    return r


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary',type=Path);parser.add_argument('directory',type=Path);parser.add_argument('--verify',action='store_true')
    args=parser.parse_args()
    if args.verify:
        verify(args.directory,args.binary);print('Verified candidate and constraint diagnostics');return
    if args.directory.exists() or mujoco.__version__!='3.9.0':parser.error('new directory and MuJoCo 3.9.0 required')
    args.directory.mkdir(parents=True)
    audit=native.run(args.binary.resolve(),True);repeat=native.run(args.binary.resolve(),True);disabled=native.run(args.binary.resolve(),False)
    if audit['contact_diagnostics']!=repeat['contact_diagnostics'] or audit['trace']!=repeat['trace'] or audit['trace']!=disabled['trace'][:26]:raise ValueError('observation/replay differs')
    report=dict(protocol='contact_constraint_audit_v1',mujoco_version=mujoco.__version__,observation_preserves_trace=True,case=CASE,columns=COLUMNS,step_columns=STEP_COLUMNS,
                binary_sha256=hashlib.sha256(args.binary.read_bytes()).hexdigest(),
                source_sha256={n:hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in SOURCES},
                native=audit,native_summary=native_summary(audit),mujoco=[])
    for i,setting in enumerate(SETTINGS):
        ref=setting['solref'];margin=setting['margin_m'];gap=setting['gap_m']
        steps,points=run_mujoco(ref,margin,gap);again=run_mujoco(ref,margin,gap)
        if not np.array_equal(steps,again[0]) or not np.array_equal(points,again[1]):raise ValueError('MuJoCo replay differs')
        path=args.directory/f'constraints-{i}.npz';np.savez_compressed(path,steps=steps,points=points)
        summary=mj_summary(steps,points)
        report['mujoco'].append(dict(setting=setting,file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),summary=summary,exact_replay=True))
        print(json.dumps(dict(setting=setting,summary=summary)),flush=True)
    (args.directory/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    main()
