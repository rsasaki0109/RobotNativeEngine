"""Check Native per-point impulses against momentum balance and MuJoCo diagnostics."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
import friction_contact_diagnostics as mj


def run(binary, enabled):
    args=[str(binary),'2000000','3','1.2','0','200','audit','0.5','wrench']
    if enabled:
        args.append('diagnostics')
    return json.loads(subprocess.run(args,check=True,capture_output=True,text=True,timeout=120).stdout)


def summarize(audit):
    trace=np.asarray(audit['trace'])
    rows=audit['contact_diagnostics']
    if trace.shape!=(26,15) or len(rows)!=25 or audit['debug_assertions']:
        raise ValueError('expected 50 ms release diagnostics')
    if not np.allclose(trace[:,0],np.arange(26)*.002,atol=1e-12,rtol=0):
        raise ValueError('clock differs')
    forces=[];moments=[];loaded=[];support=[];normal_moments=[];tangent_moments=[]
    force_errors=[];moment_errors=[]
    for i,row in enumerate(rows):
        if not np.array_equal(row['before'][1:14],trace[i,1:14]):
            raise ValueError('pre-step pose differs')
        total=np.zeros(3);moment=np.zeros(3);normal_moment=np.zeros(3);tangent_moment=np.zeros(3)
        weighted=np.zeros(3);weight=0;count=0
        for p in row['points']:
            n=np.asarray(p['normal_world']);jt=np.asarray(p['tangential_impulse_world_n_s'])
            jn=p['normal_impulse_n_s'];point=np.asarray(p['point_world_m']);opposite=np.asarray(p['other_point_world_m'])
            if p['dt_s']!=.002 or abs(np.linalg.norm(n)-1)>1e-12 or abs(n@jt)>1e-12:
                raise ValueError('normal/tangent/duration differs')
            if not np.allclose(point-opposite,n*p['gap_m'],atol=1e-12,rtol=0):
                raise ValueError('surface point separation differs')
            if p['body_index']==row['body_index']:
                sign=1
            elif p['other_index']==row['body_index']:
                sign=-1;point=opposite
            else:
                raise ValueError('unrelated point')
            normal_force=sign*n*jn/.002;tangent_force=sign*jt/.002
            lever=point-trace[i,1:4]
            total+=normal_force+tangent_force
            normal_moment+=np.cross(lever,normal_force);tangent_moment+=np.cross(lever,tangent_force)
            count+=int(jn/.002>1e-8)
            weighted+=lever*(jn/.002);weight+=jn/.002
        moment=normal_moment+tangent_moment
        expected=3*(trace[i+1,8:11]-trace[i,8:11])/.002-np.array([17.658,-29.43,0])
        expected_moment=.005*(trace[i+1,11:14]-trace[i,11:14])/.002
        force_errors.append(float(np.abs(total-expected).max()))
        moment_errors.append(float(np.abs(moment-expected_moment).max()))
        # Compare impulses: ECS angular velocity is reconstructed from pose differences.
        if not np.allclose(total*.002,expected*.002,atol=1e-8,rtol=0) or not np.allclose(moment*.002,expected_moment*.002,atol=1e-8,rtol=0):
            raise ValueError('point impulse/momentum balance differs')
        forces.append(total.tolist());moments.append(moment.tolist());loaded.append(count)
        normal_moments.append(normal_moment.tolist());tangent_moments.append(tangent_moment.tolist())
        support.append((weighted/weight).tolist() if weight>0 else None)
    return dict(impulse_balance_tolerance_n_s=1e-8,angular_impulse_balance_tolerance_nm_s=1e-8,
                max_force_balance_error_n=max(force_errors),max_moment_balance_error_nm=max(moment_errors),force_world_n=forces,moment_world_nm=moments,loaded_contacts=loaded,
                normal_moment_world_nm=normal_moments,tangential_moment_world_nm=tangent_moments,
                normal_weighted_support_offset_world_m=support,
                max_spin_rad_s=float(np.linalg.norm(trace[:,11:14],axis=1).max()),
                max_contact_moment_nm=float(np.linalg.norm(moments,axis=1).max()))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary',type=Path);parser.add_argument('mujoco_directory',type=Path);parser.add_argument('output',type=Path)
    parser.add_argument('--verify',action='store_true')
    args=parser.parse_args()
    binary=args.binary.resolve()
    if args.verify:
        report=json.loads(args.output.read_text())
        if report['binary_sha256']!=hashlib.sha256(binary.read_bytes()).hexdigest() or report['source_sha256']!=hashlib.sha256(Path(__file__).read_bytes()).hexdigest():
            raise ValueError('source/binary differs')
        if report['protocol']!='native_contact_diagnostics_v1' or not report['exact_replay'] or not report['observation_preserves_trace']:
            raise ValueError('protocol/replay differs')
        if summarize(report['native_audit'])!=report['native_summary']:
            raise ValueError('native summary differs')
        repeated=run(binary,True)
        if any(repeated[k]!=report['native_audit'][k] for k in ['trace','contact_diagnostics','settings','load_mode']):
            raise ValueError('replay differs')
        prior_path=args.mujoco_directory/'report.json'
        if hashlib.sha256(prior_path.read_bytes()).hexdigest()!=report['mujoco_report_sha256']:
            raise ValueError('MuJoCo report differs')
        prior=json.loads(prior_path.read_text())
        if len(prior['results'])!=len(report['mujoco']):
            raise ValueError('MuJoCo record count differs')
        for original,saved in zip(prior['results'],report['mujoco']):
            path=args.mujoco_directory/original['file']
            if hashlib.sha256(path.read_bytes()).hexdigest()!=saved['trace_sha256'] or original['setting']!=saved['setting']:
                raise ValueError('MuJoCo provenance differs')
            with np.load(path,allow_pickle=False) as data:
                if mj.summarize(data['steps'],data['points'])!=saved['summary']:
                    raise ValueError('MuJoCo summary differs')
        print('Verified Native per-point balance and replay')
        return
    if args.output.exists():
        parser.error('new output required')
    audit=run(binary,True);repeat=run(binary,True);disabled=run(binary,False)
    if audit['trace']!=repeat['trace'] or audit['contact_diagnostics']!=repeat['contact_diagnostics']:
        raise ValueError('diagnostic replay differs')
    if audit['trace']!=disabled['trace'][:26] or disabled['contact_diagnostics']:
        raise ValueError('observation changed simulation')
    prior=json.loads((args.mujoco_directory/'report.json').read_text());mj_rows=[]
    for r in prior['results']:
        path=args.mujoco_directory/r['file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=r['sha256']:
            raise ValueError('MuJoCo trace hash differs')
        with np.load(path,allow_pickle=False) as data:
            summary=mj.summarize(data['steps'],data['points'])
        if summary!=r['summary']:
            raise ValueError('MuJoCo summary differs')
        mj_rows.append(dict(setting=r['setting'],summary=summary,trace_sha256=r['sha256']))
    report=dict(protocol='native_contact_diagnostics_v1',binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                native_audit=audit,native_summary=summarize(audit),exact_replay=True,
                observation_preserves_trace=True,mujoco=mj_rows,
                mujoco_report_sha256=hashlib.sha256((args.mujoco_directory/'report.json').read_bytes()).hexdigest())
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report['native_summary'],indent=2))


if __name__=='__main__':
    main()
