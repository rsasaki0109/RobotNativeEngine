"""First-50-ms contact force/moment audit, with Native momentum-balance reference."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
import force_friction as base
import force_friction_matched as matched

CASE = dict(dt_ticks=8000, mass_kg=3, force_ratio=1.2, mu=0.5)
SETTINGS = [dict(solref=r, margin_m=0.0) for r in [[0.02,1], [0.005,1], [2e-6,10]]] + [dict(solref=[0.02,1], margin_m=m) for m in [1e-8,1e-7]]
DURATION_S = 0.05
SOURCES = ['friction_contact_diagnostics.py','force_friction_matched.py','force_friction.py']
STEP_COLUMNS = ['time_start_s','time_end_s','contacts','loaded_contacts',
    'com_x','com_y','com_z','force_x','force_y','force_z','moment_x','moment_y','moment_z',
    'constraint_force_x','constraint_force_y','constraint_force_z',
    'constraint_moment_x','constraint_moment_y','constraint_moment_z',
    'accel_x','accel_y','accel_z','angular_accel_x','angular_accel_y','angular_accel_z',
    'velocity_after_x','velocity_after_y','velocity_after_z',
    'omega_after_x','omega_after_y','omega_after_z']
POINT_COLUMNS = ['step','geom1','geom2','distance_m','point_x','point_y','point_z',
    'normal_x','normal_y','normal_z','force_x','force_y','force_z',
    'moment_x','moment_y','moment_z','normal_force_n','tangent_force_n',
    'relative_v_x','relative_v_y','relative_v_z','power_w']


def run_mujoco(ref, margin_m=0.0):
    xml=matched.model_xml(CASE,ref).replace('margin="0"', f'margin="{margin_m}"')
    model=mujoco.MjModel.from_xml_string(xml)
    data=mujoco.MjData(model)
    mujoco.mj_forward(model,data)
    steps=round(DURATION_S/model.opt.timestep)
    rows=[]; points=[]
    mass=CASE['mass_kg']
    external=np.array([mass*CASE['force_ratio']*CASE['mu']*9.81,0,0])
    for i in range(steps):
        # Forward-stage positions, contacts and forces left by mj_step describe
        # the pre-integration pose; qpos/qvel have advanced. Preserve that pose.
        before_position=data.qpos[:3].copy()
        before_rotation=np.empty(9)
        mujoco.mju_quat2Mat(before_rotation,data.qpos[3:7])
        rotation=before_rotation.reshape(3,3)
        velocity=data.qvel[:3].copy()
        omega=rotation@data.qvel[3:6]
        data.xfrc_applied[1,:]=[*external,0,0,0]
        mujoco.mj_step(model,data)
        total_force=np.zeros(3); total_moment=np.zeros(3); loaded=0
        for j in range(data.ncon):
            contact=data.contact[j]
            if {int(contact.geom1),int(contact.geom2)}!={0,1}:
                raise ValueError('unexpected contact pair')
            local=np.zeros(6)
            mujoco.mj_contactForce(model,data,j,local)
            sign=1 if contact.geom2==1 else -1
            frame=contact.frame.reshape(3,3)
            normal=sign*frame[0]
            force=sign*(frame.T@local[:3])
            free_torque=sign*(frame.T@local[3:])
            lever=contact.pos-before_position
            moment=np.cross(lever,force)+free_torque
            point_velocity=velocity+np.cross(omega,lever)
            normal_force=float(force@normal)
            tangent=float(np.linalg.norm(force-normal_force*normal))
            total_force+=force;total_moment+=moment
            loaded+=int(normal_force>1e-8)
            points.append([i,int(contact.geom1),int(contact.geom2),contact.dist,*contact.pos,
                           *normal,*force,*moment,normal_force,tangent,*point_velocity,float(force@point_velocity)])
        # Free-joint translation is world-frame, angular coordinates body-frame.
        constraint=np.r_[data.qfrc_constraint[:3],rotation@data.qfrc_constraint[3:6]]
        after_rotation=np.empty(9)
        mujoco.mju_quat2Mat(after_rotation,data.qpos[3:7])
        rows.append([i*model.opt.timestep,(i+1)*model.opt.timestep,data.ncon,loaded,
                     *before_position,*total_force,*total_moment,*constraint,
                     *data.qacc[:3],*(rotation@data.qacc[3:6]),
                     *data.qvel[:3],*(after_rotation.reshape(3,3)@data.qvel[3:6])])
    return np.asarray(rows),np.asarray(points),[int(w.number) for w in data.warning]


def summarize(steps,points):
    if steps.shape!=(6250,len(STEP_COLUMNS)) or points.ndim!=2 or points.shape[1]!=len(POINT_COLUMNS):
        raise ValueError('unexpected diagnostic shape')
    if not np.isfinite(steps).all() or not np.isfinite(points).all():
        raise ValueError('nonfinite diagnostics')
    expected_time=np.arange(6250)*CASE['dt_ticks']*1e-9
    if not np.allclose(steps[:,0],expected_time,atol=1e-12,rtol=0) or not np.allclose(steps[:,1],expected_time+8e-6,atol=1e-12,rtol=0):
        raise ValueError('diagnostic clock differs')
    mass=CASE['mass_kg']; inertia=mass*0.1**2/6
    expected_force=mass*steps[:,19:22]-np.array([mass*CASE['force_ratio']*CASE['mu']*9.81,-mass*9.81,0])
    force_residual=float(np.abs(steps[:,7:10]-expected_force).max())
    torque_residual=float(np.abs(steps[:,10:13]-inertia*steps[:,22:25]).max())
    spin=np.linalg.norm(steps[:,28:31],axis=1)
    crossed=np.flatnonzero(spin>0.01)
    first=int(crossed[0]) if len(crossed) else None
    loaded_changes=np.flatnonzero(np.diff(steps[:,3])!=0)+1
    # Reconstruct aggregate contact wrench from raw per-point forces independently.
    forces=np.zeros((len(steps),3));moments=np.zeros_like(forces);counts=np.zeros(len(steps))
    indices=points[:,0].astype(int)
    if not np.array_equal(indices,points[:,0]) or (indices<0).any() or (indices>=len(steps)).any():
        raise ValueError('invalid point step')
    calculated_moments=np.cross(points[:,4:7]-steps[indices,4:7],points[:,10:13])
    if not np.allclose(calculated_moments,points[:,13:16],atol=1e-12,rtol=0):
        raise ValueError('point lever-arm moment differs')
    if not np.allclose(np.linalg.norm(points[:,7:10],axis=1),1,atol=1e-12,rtol=0):
        raise ValueError('nonunit contact normal')
    normal_force=np.sum(points[:,10:13]*points[:,7:10],axis=1)
    tangential=np.linalg.norm(points[:,10:13]-normal_force[:,None]*points[:,7:10],axis=1)
    if not np.allclose(normal_force,points[:,16],atol=1e-12,rtol=0) or not np.allclose(tangential,points[:,17],atol=1e-12,rtol=0):
        raise ValueError('normal/tangential decomposition differs')
    loaded_counts=np.zeros(len(steps))
    np.add.at(loaded_counts,indices,normal_force>1e-8)
    if not np.array_equal(loaded_counts,steps[:,3]):
        raise ValueError('loaded contact count differs')
    np.add.at(forces,indices,points[:,10:13]);np.add.at(moments,indices,points[:,13:16]);np.add.at(counts,indices,1)
    if not np.allclose(forces,steps[:,7:10],atol=1e-12,rtol=0) or not np.allclose(moments,steps[:,10:13],atol=1e-12,rtol=0) or not np.array_equal(counts,steps[:,2]):
        raise ValueError('contact aggregation differs')
    if np.abs(steps[:,7:13]-steps[:,13:19]).max()>1e-8 or force_residual>1e-8 or torque_residual>1e-8:
        raise ValueError('contact frame/sign or dynamics balance failed')
    gate_support=None
    if first is not None:
        select=indices==first
        lever=points[select,4:7]-steps[first,4:7]
        normal_vectors=normal_force[select,None]*points[select,7:10]
        tangent_vectors=points[select,10:13]-normal_vectors
        total_normal=normal_force[select].sum()
        gate_support=dict(normal_force_n=float(total_normal),
            normal_moment_world_nm=np.cross(lever,normal_vectors).sum(axis=0).tolist(),
            tangential_moment_world_nm=np.cross(lever,tangent_vectors).sum(axis=0).tolist(),
            normal_weighted_support_offset_world_m=(np.sum(lever*normal_force[select,None],axis=0)/total_normal).tolist() if total_normal>0 else None)
    return dict(gate_support=gate_support,max_friction_cone_excess_n=float(np.maximum(0,tangential-CASE['mu']*normal_force).max()),force_balance_max_error_n=force_residual,moment_balance_max_error_nm=torque_residual,
                constraint_wrench_max_error=float(np.abs(steps[:,7:13]-steps[:,13:19]).max()),
                first_spin_gate_time_s=float(steps[first,1]) if first is not None else None,
                at_spin_gate=steps[first].tolist() if first is not None else None,
                first_loaded_count_changes=[dict(time_s=float(steps[i,0]),before=int(steps[i-1,3]),after=int(steps[i,3])) for i in loaded_changes[:12]],
                max_spin_rad_s=float(spin.max()),max_abs_contact_moment_nm=float(np.linalg.norm(steps[:,10:13],axis=1).max()),
                max_abs_force_n=float(np.linalg.norm(steps[:,7:10],axis=1).max()),
                min_loaded_contacts=int(steps[:,3].min()),max_loaded_contacts=int(steps[:,3].max()))


def native_reference(binary):
    c=dict(CASE,dt_ticks=2000000)
    audit=base.run_native(binary,c,True)
    if audit['trace']!=base.run_native(binary,c,True)['trace']:
        raise ValueError('native replay differs')
    x=np.asarray(audit['trace'])[:26]
    mass=CASE['mass_kg'];inertia=mass*0.1**2/6;dt=0.002
    force=mass*np.diff(x[:,8:11],axis=0)/dt-np.array([mass*CASE['force_ratio']*CASE['mu']*9.81,-mass*9.81,0])
    moment=inertia*np.diff(x[:,11:14],axis=0)/dt
    return dict(case=c,trace=x.tolist(),force_world_n=force.tolist(),moment_world_nm=moment.tolist(),
                interpretation='net contact wrench inferred from ECS velocity differences, not per-point solver forces',
                max_spin_rad_s=float(np.linalg.norm(x[:,11:14],axis=1).max()),
                max_abs_contact_moment_nm=float(np.linalg.norm(moment,axis=1).max()),
                exact_replay=True)


def verify(directory, binary):
    report=json.loads((directory/'report.json').read_text())
    if (report['protocol']!='friction_contact_diagnostics_v1' or report['case']!=CASE
            or report['duration_s']!=DURATION_S or report['step_columns']!=STEP_COLUMNS
            or report['point_columns']!=POINT_COLUMNS or report['mujoco']!='3.9.0'
            or [r['setting'] for r in report['results']]!=SETTINGS):
        raise ValueError('protocol differs')
    if report['binary_sha256']!=hashlib.sha256(binary.read_bytes()).hexdigest():
        raise ValueError('native binary differs')
    for name in SOURCES:
        if report['source_sha256'][name]!=hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest():
            raise ValueError('source differs')
    for i,row in enumerate(report['results']):
        if row['file']!=f'contacts-{i}.npz' or not row['exact_replay'] or any(row['warnings']):
            raise ValueError('trace identity/health differs')
        path=directory/row['file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:
            raise ValueError('trace hash differs')
        with np.load(path,allow_pickle=False) as f:
            result=summarize(f['steps'],f['points'])
        if result!=row['summary']:
            raise ValueError('summary differs')
    if native_reference(binary.resolve())!=report['native']:
        raise ValueError('native balance reference differs')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary',type=Path);parser.add_argument('directory',type=Path)
    parser.add_argument('--verify',action='store_true')
    args=parser.parse_args()
    if args.verify:
        verify(args.directory, args.binary)
        print('Verified contact force and moment balance for five settings')
        return
    if args.directory.exists() or mujoco.__version__!='3.9.0':
        parser.error('new directory and MuJoCo 3.9.0 required')
    args.directory.mkdir(parents=True)
    report=dict(protocol='friction_contact_diagnostics_v1',case=CASE,duration_s=DURATION_S,
                step_columns=STEP_COLUMNS,point_columns=POINT_COLUMNS,mujoco=mujoco.__version__,
                source_sha256={n:hashlib.sha256(Path(__file__).with_name(n).read_bytes()).hexdigest() for n in SOURCES},
                binary_sha256=hashlib.sha256(args.binary.read_bytes()).hexdigest(),results=[])
    for i,setting in enumerate(SETTINGS):
        ref=setting['solref']; margin=setting['margin_m']
        steps,points,warnings=run_mujoco(ref,margin)
        repeated=run_mujoco(ref,margin)
        if not np.array_equal(steps,repeated[0]) or not np.array_equal(points,repeated[1]) or any(warnings):
            raise ValueError('replay/health failed')
        summary=summarize(steps,points)
        path=args.directory/f'contacts-{i}.npz'
        np.savez_compressed(path,steps=steps,points=points)
        report['results'].append(dict(setting=setting,file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),summary=summary,exact_replay=True,warnings=warnings))
        print(json.dumps(dict(setting=setting,summary=summary)),flush=True)
    report['native']=native_reference(args.binary.resolve())
    (args.directory/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':
    main()
