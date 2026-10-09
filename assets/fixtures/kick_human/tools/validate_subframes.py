"""Measure the saved reference rig at 120 Hz, including between exported keys."""
import bpy, json, pathlib, math
from mathutils import Vector
p = pathlib.Path(bpy.data.filepath).resolve().parent
arm = bpy.data.objects['HumanRig']
scene = bpy.context.scene
for track in arm.animation_data.nla_tracks:
    track.mute = True
result = {}
chains = [('thigh_l', 'calf_l'), ('calf_l', 'foot_l'), ('thigh_r', 'calf_r'), ('calf_r', 'foot_r'), ('upperarm_l', 'lowerarm_l'), ('lowerarm_l', 'hand_l'), ('upperarm_r', 'lowerarm_r'), ('lowerarm_r', 'hand_r')]
for name in ['low_kick', 'mid_kick']:
    arm.animation_data.action = bpy.data.actions[name]
    scene.frame_set(0)
    bpy.context.view_layer.update()
    left = arm.pose.bones['foot_l'].matrix.copy()
    maxdrift = maxrot = maxgap = maxlength = maxloc = maxscale = 0.0
    minfloor = 1.0
    mintoe = 1.0
    worst = None
    rows = []
    for n in range(721):
        f = n / 2
        scene.frame_set(int(f), subframe=f - int(f))
        bpy.context.view_layer.update()
        leftnow = arm.pose.bones['foot_l'].matrix
        maxdrift = max(maxdrift, (leftnow.translation - left.translation).length)
        dot = abs(leftnow.to_quaternion().dot(left.to_quaternion()))
        maxrot = max(maxrot, 2 * math.acos(min(1, dot)))
        for parent, child in chains:
            maxgap = max(maxgap, (arm.pose.bones[parent].tail - arm.pose.bones[child].head).length)
        for pb in arm.pose.bones:
            maxlength = max(maxlength, abs((pb.tail - pb.head).length - arm.data.bones[pb.name].length))
            if pb.name != 'Root':
                maxloc = max(maxloc, pb.location.length)
            maxscale = max(maxscale, max((abs(v - 1) for v in pb.scale)))
        dg = bpy.context.evaluated_depsgraph_get()
        floor = {}
        for key in ['Shoe_l', 'Sole_l', 'Shoe_r', 'Sole_r']:
            ob = bpy.data.objects[key].evaluated_get(dg)
            me = ob.to_mesh()
            floor[key] = min(((ob.matrix_world @ v.co).z for v in me.vertices))
            ob.to_mesh_clear()
        minimum = min(floor.values())
        if minimum < minfloor:
            minfloor = minimum
            worst = {'time_s': n / 120, 'objects': floor}
        rows.append({'time_s': n / 120, 'floor_min_y_m': minimum})
    result[name] = {'sample_count': 721, 'sample_rate_hz': 120, 'max_support_ankle_drift_m': maxdrift, 'max_support_foot_rotation_drift_rad': maxrot, 'maximum_connected_joint_gap_m': maxgap, 'maximum_bone_length_error_m': maxlength, 'maximum_nonroot_local_translation_m': maxloc, 'maximum_scale_error': maxscale, 'minimum_floor_y_m': minfloor, 'minimum_floor_at': worst, 'floor_samples': rows}
    assert maxgap < 2e-05 and maxlength < 2e-05 and (maxloc == 0) and (maxscale == 0)
    assert maxdrift < 0.0005 and maxrot < 0.005 and (minfloor > -0.0005)
(p / 'subframe-anatomy-audit.json').write_text(json.dumps(result, indent=2) + '\n')
print('SUBFRAME ANATOMY AUDIT PASS', json.dumps({k: {a: b for a, b in v.items() if a != 'floor_samples'} for k, v in result.items()}))
