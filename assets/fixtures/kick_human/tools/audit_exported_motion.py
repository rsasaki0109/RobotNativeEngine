"""Independently audit exported GLB node interpolation and linear blend skinning."""
from pathlib import Path
import sys, struct, json, hashlib, math
import numpy as np
P=Path(sys.argv[1]); out=Path(sys.argv[2]); blob=P.read_bytes(); cursor=12
while cursor<len(blob):
    length,kind=struct.unpack_from('<II',blob,cursor);chunk=blob[cursor+8:cursor+8+length];cursor+=8+length
    if kind==0x4e4f534a: doc=json.loads(chunk)
    elif kind==0x004e4942: binary=chunk
sizes={'SCALAR':1,'VEC2':2,'VEC3':3,'VEC4':4,'MAT4':16}; formats={5126:('f',4),5123:('H',2),5121:('B',1),5125:('I',4)}; cache={}
def accessor(index):
    if index not in cache:
        a=doc['accessors'][index]; v=doc['bufferViews'][a['bufferView']]; n=sizes[a['type']]; f,width=formats[a['componentType']]; offset=v.get('byteOffset',0)+a.get('byteOffset',0);stride=v.get('byteStride',width*n)
        cache[index]=np.array([struct.unpack_from('<'+f*n,binary,offset+i*stride)for i in range(a['count'])])
    return cache[index]
nodes=doc['nodes']; names={n.get('name',str(i)):i for i,n in enumerate(nodes)}; parents={c:i for i,n in enumerate(nodes)for c in n.get('children',[])}
base=[{k:np.array(n.get(k,d),dtype=float) for k,d in [('translation',[0,0,0]),('rotation',[0,0,0,1]),('scale',[1,1,1])]}for n in nodes]
def slerp(a,b,t):
    a=a/np.linalg.norm(a);b=b/np.linalg.norm(b);d=float(np.dot(a,b))
    if d<0:b=-b;d=-d
    if d>1-np.finfo(float).eps:q=a*(1-t)+b*t
    else:
        angle=math.acos(min(1.,d));q=(a*math.sin((1-t)*angle)+b*math.sin(t*angle))/math.sin(angle)
    return q/np.linalg.norm(q)
def matrix(v):
    x,y,z,w=v['rotation']/np.linalg.norm(v['rotation']);R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    M=np.eye(4);M[:3,:3]=R@np.diag(v['scale']);M[:3,3]=v['translation'];return M
def prepared(anim):
    result=[]
    for c in anim['channels']:
        s=anim['samplers'][c['sampler']];mode=s.get('interpolation','LINEAR');assert mode in ['LINEAR','STEP'],mode
        result.append((c['target']['node'],c['target']['path'],accessor(s['input'])[:,0],accessor(s['output']),mode))
    return result
def sample(channels,t):
    values=[dict(v)for v in base]
    for i,path,ts,vs,mode in channels:
        j=int(np.searchsorted(ts,t,side='right'))
        if j==0:v=vs[0]
        elif j==len(ts):v=vs[-1]
        elif mode=='STEP':v=vs[j-1]
        else:
            alpha=(t-ts[j-1])/(ts[j]-ts[j-1]);v=slerp(vs[j-1],vs[j],alpha)if path=='rotation'else vs[j-1]+alpha*(vs[j]-vs[j-1])
        values[i][path]=v
    local=[matrix(v)for v in values]; world={}
    def calc(i):
        if i not in world:world[i]=calc(parents[i])@local[i]if i in parents else local[i]
        return world[i]
    for i in range(len(nodes)):calc(i)
    return values,world
shoe_nodes=[(i,n['name'])for i,n in enumerate(nodes)if n.get('name')in ['Shoe_l','Sole_l','Shoe_r','Sole_r']]
shoe_data={}
for i,name in shoe_nodes:
    skin=doc['skins'][nodes[i]['skin']];ib=accessor(skin['inverseBindMatrices']).reshape((-1,4,4)).transpose(0,2,1);parts=[]
    for p in doc['meshes'][nodes[i]['mesh']]['primitives']:
        a=p['attributes'];xyz=accessor(a['POSITION']);parts.append((np.column_stack((xyz,np.ones(len(xyz)))),accessor(a['JOINTS_0']).astype(int),accessor(a['WEIGHTS_0'])))
    shoe_data[name]=(skin['joints'],ib,parts)
def skin_shoe(name,world):
    joints,ib,parts=shoe_data[name];mats=np.array([world[j]@ib[k]for k,j in enumerate(joints)]);vertices=[]
    for xyz,indices,weights in parts:
        v=np.zeros_like(xyz)
        for k in range(4):v+=np.einsum('nij,nj->ni',mats[indices[:,k]],xyz)*weights[:,k,None]
        vertices.append(v[:,:3]/weights.sum(axis=1)[:,None])
    return np.concatenate(vertices)
def angle(a,b):return math.degrees(math.acos(float(np.clip(np.dot(a,b)/np.linalg.norm(a)/np.linalg.norm(b),-1,1))))
pairs=[('thigh_l','calf_l'),('calf_l','foot_l'),('thigh_r','calf_r'),('calf_r','foot_r'),('upperarm_l','lowerarm_l'),('lowerarm_l','hand_l'),('upperarm_r','lowerarm_r'),('lowerarm_r','hand_r')]
reports=[]
for anim in doc['animations']:
    ch=prepared(anim);end=max(ts[-1]for _,_,ts,_,_ in ch);times=np.unique(np.concatenate((np.arange(0,end+1e-8,1/240),*[ts for _,_,ts,_,_ in ch],np.array([1.65,2.,2.1,2.2])))); times=times[times<=end+1e-8]
    neg=[];raw_vel=[];translation=[];scales=[];norm_error=0
    for i,path,ts,vs,mode in ch:
        name=nodes[i].get('name',str(i))
        if path=='translation':translation.append({'node':name,'axis_range_m':np.ptp(vs,axis=0).tolist(),'max_delta_from_initial_m':float(np.max(np.linalg.norm(vs-vs[0],axis=1)))})
        elif path=='scale':scales.append({'node':name,'max_delta_from_initial':float(np.max(np.abs(vs-vs[0]))),'max_delta_from_one':float(np.max(np.abs(vs-1)))})
        elif path=='rotation':
            n=np.linalg.norm(vs,axis=1);norm_error=max(norm_error,float(np.max(np.abs(n-1))));qs=vs/n[:,None];dots=np.sum(qs[:-1]*qs[1:],axis=1);neg.extend([{'node':name,'time_s':float(ts[k]),'dot':float(d)}for k,d in enumerate(dots)if d<0]);speed=2*np.arccos(np.clip(np.abs(dots),0,1))/np.diff(ts);k=int(np.argmax(speed));raw_vel.append({'node':name,'peak_rad_s':float(speed[k]),'interval_s':[float(ts[k]),float(ts[k+1])],'max_angle_from_initial_deg':float(np.max(2*np.arccos(np.clip(np.abs(qs@qs[0]),0,1)))*180/math.pi)})
    rows=[];lengths=[];floor=[];support=[];local_t=[];local_s=[];world_rot={name:[]for name in ['upperarm_l','upperarm_r','lowerarm_l','lowerarm_r','pelvis','head','foot_l','foot_r']}
    for t in times:
        values,world=sample(ch,float(t)); xyz=lambda name:world[names[name]][:3,3];hip=xyz('thigh_r');knee=xyz('calf_r');ankle=xyz('foot_r');u=ankle-hip;u/=np.linalg.norm(u);pole=knee-hip-u*np.dot(knee-hip,u);hint=np.array([0,.75,.65]);hint-=u*np.dot(hint,u)
        elbow={side:angle(xyz('lowerarm_'+side)-xyz('upperarm_'+side),xyz('hand_'+side)-xyz('lowerarm_'+side))for side in ['l','r']}
        shoulder={side:angle(xyz('lowerarm_'+side)-xyz('upperarm_'+side),np.array([0,-1,0]))for side in ['l','r']}
        row={'time_s':float(t),'hip_yup_m':hip.tolist(),'knee_yup_m':knee.tolist(),'ankle_yup_m':ankle.tolist(),'pelvis_yup_m':xyz('pelvis').tolist(),'right_knee_flexion_deg':angle(knee-hip,ankle-knee),'knee_height_minus_ankle_height_m':float(knee[1]-ankle[1]),'knee_ahead_of_hip_m':float(knee[2]-hip[2]),'forward_up_pole_cosine':float(np.dot(pole,hint)/np.linalg.norm(pole)/np.linalg.norm(hint)),'elbow_flexion_deg':elbow,'shoulder_world_angle_from_down_deg':shoulder}
        meshes={name:skin_shoe(name,world)for _,name in shoe_nodes};row['shoe_min_y_m']={name:float(np.min(v[:,1]))for name,v in meshes.items()};rows.append(row)
        lengths.append([np.linalg.norm(xyz(b)-xyz(a))for a,b in pairs]);support.append(xyz('foot_l'));local_t.append([v['translation']for v in values]);local_s.append([v['scale']for v in values])
        for name in world_rot:
            R=world[names[name]][:3,:3];world_rot[name].append(R/np.linalg.norm(R,axis=0))
    world_vel=[]
    for name,Rs in world_rot.items():
        Rs=np.array(Rs);ds=np.einsum('nij,nij->n',Rs[:-1],Rs[1:]);a=np.arccos(np.clip((ds-1)/2,-1,1));dt=np.diff(times);valid=dt>1e-5;speed=a[valid]/dt[valid];indices=np.flatnonzero(valid);k=int(np.argmax(speed));world_vel.append({'node':name,'peak_rad_s':float(speed[k]),'interval_s':[float(times[indices[k]]),float(times[indices[k]+1])]})
    knee_rows=[r for r in rows if 1.2<=r['time_s']<=2.5];lt=np.array(local_t);ls=np.array(local_s);nonroot=[i for i,n in enumerate(nodes)if n.get('name')!='Root'];stage=[min(rows,key=lambda r:abs(r['time_s']-t))for t in [0,1.2,1.65,2,2.1,2.2,2.5,2.95,3.5,6]]
    reports.append({'animation':anim['name'],'clip_end_s':float(end),'sampling_hz':240,'samples_including_original_keys':len(times),'negative_adjacent_quaternion_dots':neg,'max_raw_quaternion_norm_error':norm_error,'local_joint_angular_velocity':sorted(raw_vel,key=lambda r:-r['peak_rad_s']),'world_joint_angular_velocity':sorted(world_vel,key=lambda r:-r['peak_rad_s']),'local_translation_channels':translation,'scale_channels':scales,'max_nonroot_local_translation_change_m':float(np.max(np.linalg.norm(lt[:,nonroot]-lt[0,nonroot],axis=2))),'max_local_scale_change':float(np.max(np.abs(ls-ls[0]))),'connected_segment_length_range_m':dict(zip(['-'.join(p)for p in pairs],np.ptp(np.array(lengths),axis=0).tolist())),'support_ankle_axis_range_m':np.ptp(np.array(support),axis=0).tolist(),'shoe_floor_minima_m':{name:min(r['shoe_min_y_m'][name]for r in rows)for _,name in shoe_nodes},'floor_minimum_times_s':{name:min(rows,key=lambda r:r['shoe_min_y_m'][name])['time_s']for _,name in shoe_nodes},'support_foot_world_rotation_change_deg':max(math.degrees(math.acos(float(np.clip((np.trace(world_rot['foot_l'][0].T@R)-1)/2,-1,1))))for R in world_rot['foot_l']), 'right_ankle_max_height_m':max(r['ankle_yup_m'][1]for r in rows),'left_sole_max_abs_floor_error_m':max(abs(r['shoe_min_y_m']['Sole_l'])for r in rows),'minimum_knee_forward_up_pole_cosine':min(r['forward_up_pole_cosine']for r in rows),'maximum_knee_flexion_deg':max(r['right_knee_flexion_deg']for r in rows),'minimum_knee_above_ankle_during_active_kick_m':min(r['knee_height_minus_ankle_height_m']for r in knee_rows),'elbow_flexion_ranges_deg':{side:[min(r['elbow_flexion_deg'][side]for r in rows),max(r['elbow_flexion_deg'][side]for r in rows)]for side in ['l','r']},'shoulder_world_from_down_ranges_deg':{side:[min(r['shoulder_world_angle_from_down_deg'][side]for r in rows),max(r['shoulder_world_angle_from_down_deg'][side]for r in rows)]for side in ['l','r']},'stage_samples':stage})
assert hashlib.sha256(P.read_bytes()).hexdigest()==hashlib.sha256(blob).hexdigest(),'GLB changed during audit'
result={'source_glb':str(P),'source_glb_sha256':hashlib.sha256(blob).hexdigest(),'audit_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'method':'Independent exported GLB accessors, normalized shortest-arc SLERP, full hierarchy, CPU LBS shoe vertices; 240 Hz plus original keys; not Blender in-memory poses; not a physical human simulation','reports':reports};out.write_text(json.dumps(result,indent=2)+'\n')
for r in reports:
    print(r['animation'],'samples',r['samples_including_original_keys'],'worst local omega',r['local_joint_angular_velocity'][:3],'supportrange',r['support_ankle_axis_range_m'],'floor',r['shoe_floor_minima_m'],'knee pole',r['minimum_knee_forward_up_pole_cosine'],'knee max',r['maximum_knee_flexion_deg'],'elbow',r['elbow_flexion_ranges_deg'])
    for s in r['stage_samples']:
        if s['time_s']in [1.65,2.]:print('STAGE',s)
print('saved',out,'SHA',result['source_glb_sha256'])
