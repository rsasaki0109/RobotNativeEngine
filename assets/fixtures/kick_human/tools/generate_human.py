"""Generate the CC0 visual kicker from pinned inputs; requires Blender 4.3.2.

Run: blender -b --python generate_human.py -- --source-dir source --manifest
source-manifest.json --output-dir generated. No preexisting blend is required.
The torso sportswear and sneaker textures/geometry are original CC0 modifications.
The motion is a visual animation, not a physical human collision model.
"""
import argparse, sys, pathlib, json, hashlib, bpy
parser = argparse.ArgumentParser()
parser.add_argument('--source-dir', type=pathlib.Path, required=True)
parser.add_argument('--manifest', type=pathlib.Path, required=True)
parser.add_argument('--output-dir', type=pathlib.Path, required=True)
args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:])
S = args.source_dir.resolve()
O = args.output_dir.resolve()
O.mkdir(parents=True, exist_ok=True)
P = O
if bpy.app.version != (4, 3, 2):
    raise RuntimeError('The reference generator requires Blender 4.3.2')
for entry in json.loads(args.manifest.read_text()):
    path = S / entry.get('local_filename', pathlib.Path(entry['path']).name)
    data = path.read_bytes()
    if len(data) != entry['size_bytes'] or hashlib.sha256(data).hexdigest() != entry['sha256']:
        raise RuntimeError('Pinned source verification failed: ' + path.name)
import bpy, json, pathlib, gzip, math, random, hashlib
from mathutils import Vector, Matrix, Quaternion
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
verts = []
uv = []
faces = []
faceuv = []
groups = {}
group = ''
for line in (S / 'base.obj').read_text().splitlines():
    a = line.split()
    if not a:
        continue
    if a[0] == 'v':
        verts.append(Vector(tuple(map(float, a[1:4]))))
    elif a[0] == 'vt':
        uv.append(tuple(map(float, a[1:3])))
    elif a[0] == 'g':
        group = ' '.join(a[1:])
        groups.setdefault(group, set())
    elif a[0] == 'f':
        ids = [int(x.split('/')[0]) - 1 for x in a[1:]]
        groups.setdefault(group, set()).update(ids)
        if group == 'body':
            faces.append(ids)
            faceuv.append([int(x.split('/')[1]) - 1 if '/' in x else 0 for x in a[1:]])
for line in gzip.decompress((S / 'caucasian-male-young.target.gz').read_bytes()).decode().splitlines():
    a = line.split()
    if len(a) == 4:
        verts[int(a[0])] += Vector(tuple(map(float, a[1:])))
body_indices = sorted(set((x for face in faces for x in face)))
minimum = min((verts[i].y for i in body_indices))
maximum = max((verts[i].y for i in body_indices))
scale = 1.75 / (maximum - minimum)

def cv(v):
    return Vector((v.x * scale, -v.z * scale, (v.y - minimum) * scale))
world = [cv(v) for v in verts]
mapping = {old: new for new, old in enumerate(body_indices)}
mesh = bpy.data.meshes.new('CC0 anatomical adult mesh')
mesh.from_pydata([world[i] for i in body_indices], [], [[mapping[i] for i in face] for face in faces])
mesh.update()
body = bpy.data.objects.new('Adult_sportswear', mesh)
bpy.context.collection.objects.link(body)
layer = mesh.uv_layers.new(name='MakeHumanUV')
for poly, ids in zip(mesh.polygons, faceuv):
    for loop, u in zip(poly.loop_indices, ids):
        layer.data[loop].uv = uv[u]
    poly.use_smooth = True
random.seed(4217)

def texmat(name, color, noise=0.025, roughness=0.8):
    image = bpy.data.images.new(name + '_own_texture', width=128, height=128, alpha=True)
    pixels = []
    for y in range(128):
        for x in range(128):
            n = (random.random() - 0.5) * noise + (0.004 if (x + y) % 3 == 0 else 0)
            pixels.extend((min(1, max(0, color[0] + n)), min(1, max(0, color[1] + n)), min(1, max(0, color[2] + n)), 1))
    image.pixels.foreach_set(pixels)
    image.filepath_raw = str(O / (name + '.png'))
    image.file_format = 'PNG'
    image.save()
    image.pack()
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get('Principled BSDF')
    bsdf.inputs['Roughness'].default_value = roughness
    node = mat.node_tree.nodes.new('ShaderNodeTexImage')
    node.image = image
    mat.node_tree.links.new(node.outputs['Color'], bsdf.inputs['Base Color'])
    return mat
materials = [texmat('Skin', (0.56, 0.36, 0.25), 0.04, 0.65), texmat('Blue_sport_shirt', (0.055, 0.15, 0.31), 0.02), texmat('Charcoal_trousers', (0.05, 0.065, 0.08), 0.018), texmat('Black_shoes', (0.023, 0.028, 0.037), 0.014), texmat('Hair', (0.04, 0.025, 0.017), 0.012)]
for mat in materials:
    body.data.materials.append(mat)
weights_data = json.loads((S / 'weights.game_engine.json').read_text())['weights']
vertex_categories = {i: [0.0, 0.0, 0.0, 0.0] for i in body_indices}
for bone, entries in weights_data.items():
    category = 3 if bone.startswith(('foot_', 'ball_')) else 2 if bone.startswith(('pelvis', 'thigh_', 'calf_')) else 1 if bone.startswith(('spine_', 'clavicle_', 'upperarm_', 'lowerarm_')) else 0
    for old, w in entries:
        if old in vertex_categories:
            vertex_categories[old][category] += w
for poly in mesh.polygons:
    category = [sum((vertex_categories[body_indices[i]][j] for i in poly.vertices)) for j in range(4)]
    poly.material_index = max(range(4), key=lambda j: category[j])
rig = json.loads((S / 'rig.game_engine.json').read_text())
weights = json.loads((S / 'weights.game_engine.json').read_text())['weights']
armdata = bpy.data.armatures.new('CC0_game_engine_skeleton')
arm = bpy.data.objects.new('HumanRig', armdata)
bpy.context.collection.objects.link(arm)
bpy.context.view_layer.objects.active = arm
arm.select_set(True)
bpy.ops.object.mode_set(mode='EDIT')

def point(desc):
    if desc['strategy'] == 'CUBE':
        ids = groups[desc['cube_name']]
    else:
        ids = desc['vertex_indices']
    return sum((world[i] for i in ids), Vector()) / len(ids)
for name, definition in rig.items():
    bone = armdata.edit_bones.new(name)
    bone.head = point(definition['head'])
    bone.tail = point(definition['tail'])
    bone.roll = definition['roll']
    if (bone.tail - bone.head).length < 1e-05:
        bone.tail = bone.head + Vector((0, 0, 0.05))
for name, d in rig.items():
    if d['parent']:
        armdata.edit_bones[name].parent = armdata.edit_bones[d['parent']]
bpy.ops.object.mode_set(mode='OBJECT')
for bone, entries in weights.items():
    vg = body.vertex_groups.new(name=bone)
    for old, w in entries:
        if old in mapping and w > 0:
            vg.add([mapping[old]], w, 'REPLACE')
mod = body.modifiers.new('Skinning', 'ARMATURE')
mod.object = arm
body.parent = arm
eye_material = texmat('Eye_white', (0.75, 0.78, 0.77), 0.01, 0.25)
for side, group in [('l', 'joint-l-eye'), ('r', 'joint-r-eye')]:
    pos = point({'strategy': 'CUBE', 'cube_name': group})
    for kind, offset, size, mat in [('Eye', 0, 0.009, eye_material), ('Iris', -0.007, 0.003, materials[4])]:
        bpy.ops.mesh.primitive_uv_sphere_add(segments=16, ring_count=8, radius=size, location=pos + Vector((0, offset, 0)))
        obj = bpy.context.object
        obj.name = kind + '_' + side
        obj.data.materials.append(mat)
        for poly in obj.data.polygons:
            poly.use_smooth = True
        groupweight = obj.vertex_groups.new(name='head')
        groupweight.add(list(range(len(obj.data.vertices))), 1, 'REPLACE')
        m = obj.modifiers.new('head_skinning', 'ARMATURE')
        m.object = arm
        obj.parent = arm
for side in ['l', 'r']:
    ankle = armdata.bones['foot_' + side].head_local
    for kind, z, sc, color in [('Shoe', 0.058, (0.062, 0.136, 0.058), materials[3]), ('Sole', 0.014, (0.064, 0.138, 0.014), texmat('Sole_' + side, (0.52, 0.55, 0.58), 0.008, 0.8))]:
        bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=12, radius=1, location=(ankle.x, ankle.y - 0.085, z))
        obj = bpy.context.object
        obj.name = kind + '_' + side
        obj.scale = sc
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        obj.data.materials.append(color)
        for poly in obj.data.polygons:
            poly.use_smooth = True
        vg = obj.vertex_groups.new(name='foot_' + side)
        vg.add(list(range(len(obj.data.vertices))), 1, 'REPLACE')
        m = obj.modifiers.new('shoe_skinning', 'ARMATURE')
        m.object = arm
        obj.parent = arm
arm = bpy.data.objects['HumanRig']
rig = arm.data
scene = bpy.context.scene
names = sorted((o.name for o in scene.objects if o.parent == arm and o.type == 'MESH'))
arm.animation_data_create()
arm.animation_data.action = None
for tr in list(arm.animation_data.nla_tracks):
    arm.animation_data.nla_tracks.remove(tr)
for name in ['low_kick', 'mid_kick']:
    if name in bpy.data.actions:
        bpy.data.actions.remove(bpy.data.actions[name])
scene.render.fps = 60
scene.frame_start = 0
scene.frame_end = 360
REST = {b.name: b.matrix_local.to_quaternion() for b in rig.bones}
ANKLE = {s: rig.bones['foot_' + s].head_local.copy() for s in ['l', 'r']}

def smooth(x):
    x = min(1.0, max(0.0, x))
    return x * x * x * (10 + x * (-15 + 6 * x))

def seg(t, start, end):
    return smooth((t - start) / (end - start))

def blend(a, b, s):
    return a * (1 - s) + b * s

def mixdir(a, b, s):
    a = a.normalized()
    b = b.normalized()
    delta = a.rotation_difference(b)
    return Quaternion((1, 0, 0, 0)).slerp(delta, s) @ a

def rotation(name, desired):
    """Set only the local rotation, preserving rest joint offsets and lengths."""
    pb = arm.pose.bones[name]
    b = rig.bones[name]
    if b.parent:
        base = arm.pose.bones[b.parent.name].matrix.to_quaternion() @ REST[b.parent.name].inverted() @ REST[name]
    else:
        base = REST[name]
    q = base.inverted() @ desired
    q.normalize()
    pb.rotation_quaternion = q
    bpy.context.view_layer.update()

def direction(name, desired):
    b = rig.bones[name]
    pb = arm.pose.bones[name]
    base = pb.matrix.to_quaternion()
    old = base @ Vector((0, 1, 0))
    q = old.rotation_difference(desired.normalized()) @ base
    rotation(name, q)

def solve_leg(side, ankle):
    a = rig.bones['thigh_' + side].length
    b = rig.bones['calf_' + side].length
    hip = arm.pose.bones['thigh_' + side].head.copy()
    d = ankle - hip
    distance = d.length
    assert abs(a - b) + 1e-05 < distance < a + b - 1e-05, ('unreachable', side, distance, a + b)
    u = d.normalized()
    along = (a * a - b * b + distance * distance) / (2 * distance)
    height = max(0, a * a - along * along) ** 0.5
    hint = Vector((0, -0.65, 0.75))
    pole = (hint - u * hint.dot(u)).normalized()
    knee = hip + u * along + pole * height
    direction('thigh_' + side, knee - hip)
    actual_knee = arm.pose.bones['calf_' + side].head.copy()
    direction('calf_' + side, ankle - actual_knee)
    actual = arm.pose.bones['foot_' + side].head.copy()
    assert (actual - ankle).length < 2e-05, ('FK ankle mismatch', side, list(actual), list(ankle))
    flex = math.pi - math.acos(max(-1, min(1, (hip - knee).normalized().dot((ankle - knee).normalized()))))
    return {'side': side, 'reach_ratio': distance / (a + b), 'knee_flexion_rad': flex, 'knee': list(actual_knee), 'hip': list(hip), 'ankle': list(actual), 'pole_forward_up_dot': pole.dot(hint)}

def floor_probe():
    dg = bpy.context.evaluated_depsgraph_get()
    row = {}
    for name in ['Shoe_l', 'Sole_l', 'Shoe_r', 'Sole_r']:
        obj = bpy.data.objects[name].evaluated_get(dg)
        mesh = obj.to_mesh()
        points = [obj.matrix_world @ v.co for v in mesh.vertices]
        obj.to_mesh_clear()
        row[name] = {'min_y_m': min((v.z for v in points))}
    return row

def toe_probe():
    dg = bpy.context.evaluated_depsgraph_get()
    points = []
    for name in ['Shoe_r', 'Sole_r']:
        obj = bpy.data.objects[name].evaluated_get(dg)
        mesh = obj.to_mesh()
        points.extend([obj.matrix_world @ v.co for v in mesh.vertices])
        obj.to_mesh_clear()
    maxz = max((-v.y for v in points))
    band = [v for v in points if -v.y > maxz - 0.008]
    return {'front_band_center_yup_m': [sum((v.x for v in band)) / len(band), sum((v.z for v in band)) / len(band), sum((-v.y for v in band)) / len(band)], 'max_z_m': maxz, 'bbox_yup_m': {'min': [min((v.x for v in points)), min((v.z for v in points)), min((-v.y for v in points))], 'max': [max((v.x for v in points)), max((v.z for v in points)), max((-v.y for v in points))]}}
strike_angle = math.radians(10)
strike_delta = Quaternion(Vector((1, 0, 0)), -strike_angle)
restpoints = []
for name in ['Shoe_r', 'Sole_r']:
    obj = bpy.data.objects[name]
    restpoints.extend([strike_delta @ (obj.matrix_world @ v.co - ANKLE['r']) for v in obj.data.vertices])
front = max((-v.y for v in restpoints))
band = [v for v in restpoints if -v.y > front - 0.008]
toe_offset_y = sum((v.z for v in band)) / len(band)
toe_offset_x = sum((v.x for v in band)) / len(band)
TARGETS = {name: Vector((-0.1965676 - toe_offset_x, -(0.92 - front), height - toe_offset_y)) for name, height in [('low_kick', 0.284789), ('mid_kick', 0.8468078)]}

def path(t, name):
    impact_shift = Vector((0.16, 0.07 if name == "mid_kick" else -0.26, -0.08))
    weight = seg(t, 0.65, 1.2)
    chamber = seg(t, 1.2, 1.65)
    extend = seg(t, 1.65, 2.0)
    retract = seg(t, 2.2, 2.5)
    plant = seg(t, 2.5, 2.95)
    recover = seg(t, 2.95, 3.5)
    if t < 1.2:
        shift = blend(Vector((0, 0, -0.025)), Vector((0.15, -0.055, -0.06)), weight)
    elif t < 1.65:
        shift = blend(Vector((0.15, -0.055, -0.06)), Vector((0.16, -0.14, -0.07)), chamber)
    elif t < 2:
        shift = blend(Vector((0.16, -0.14, -0.07)), impact_shift, extend)
    elif t <= 2.2:
        shift = impact_shift + Vector((0, -0.012 * math.sin(math.pi * (t - 2) / 0.2) ** 2, 0))
    elif t < 2.5:
        shift = blend(impact_shift, Vector((0.16, -0.13, -0.065)), retract)
    elif t < 2.95:
        shift = blend(Vector((0.16, -0.13, -0.065)), Vector((0.15, -0.055, -0.055)), plant)
    else:
        shift = blend(Vector((0.15, -0.055, -0.055)), Vector((0, 0, -0.025)), recover)
    guard = weight * (1 - recover)
    power = extend * (1 - retract)
    chamber_point = Vector((-0.12, -0.2, 0.44 if name == 'low_kick' else 0.59))
    if t < 1.2:
        ankle = ANKLE['r'].copy()
        angle = 0
    elif t < 1.65:
        ankle = blend(ANKLE['r'], chamber_point, chamber)
        angle = math.radians(-4) * chamber
    elif t < 2:
        ankle = blend(chamber_point, TARGETS[name], extend)
        angle = blend(math.radians(-4), strike_angle, extend)
    elif t <= 2.2:
        ankle = TARGETS[name] + Vector((0, -0.025 * math.sin(math.pi * (t - 2) / 0.2) ** 2, 0))
        angle = strike_angle
    elif t < 2.5:
        ankle = blend(TARGETS[name], chamber_point, retract)
        angle = blend(strike_angle, math.radians(-4), retract)
    elif t < 2.95:
        ankle = blend(chamber_point, ANKLE['r'], plant)
        angle = math.radians(-4) * (1 - plant)
    else:
        ankle = ANKLE['r'].copy()
        angle = 0
    return (shift, ankle, angle, guard, power)

def pose(t, name):
    for pb in arm.pose.bones:
        pb.rotation_mode = 'QUATERNION'
        pb.location = (0, 0, 0)
        pb.scale = (1, 1, 1)
        pb.rotation_quaternion = (1, 0, 0, 0)
    shift, ankle, angle, guard, power = path(t, name)
    arm.pose.bones['Root'].location = shift
    bpy.context.view_layer.update()
    lean_x = (0.035 if name == 'low_kick' else -0.12) * power
    torso = Quaternion(Vector((0, 1, 0)), 0.07 * guard) @ Quaternion(Vector((1, 0, 0)), lean_x)
    rotation('spine_01', torso @ REST['spine_01'])
    twist = Quaternion(Vector((0, 0, 1)), -0.08 * power)
    rotation('spine_03', twist @ arm.pose.bones['spine_03'].matrix.to_quaternion())
    legs = [solve_leg('l', ANKLE['l']), solve_leg('r', ankle)]
    rotation('foot_l', REST['foot_l'])
    rotation('foot_r', Quaternion(Vector((1, 0, 0)), -angle) @ REST['foot_r'])
    for side, sign in [('l', 1), ('r', -1)]:
        neutral_upper = Vector((sign * 0.03, -0.03, -0.251))
        neutral_lower = Vector((sign * 0.01, -0.065, -0.254))
        if side == 'l':
            guard_upper = Vector((sign * 0.055, -0.19, -0.159))
            guard_lower = Vector((sign * -0.015, -0.15, 0.2))
            strike_upper = Vector((sign * 0.055, -0.19, -0.159))
            strike_lower = Vector((sign * -0.03, -0.13, 0.22))
        else:
            guard_upper = Vector((sign * 0.095, 0.085, -0.218))
            guard_lower = Vector((sign * -0.03, -0.17, -0.18))
            strike_upper = Vector((sign * 0.1, 0.18, -0.15))
            strike_lower = Vector((sign * 0.03, 0.03, -0.25))
        upper = mixdir(neutral_upper, mixdir(guard_upper, strike_upper, power), guard)
        lower = mixdir(neutral_lower, mixdir(guard_lower, strike_lower, power), guard)
        direction('upperarm_' + side, upper)
        direction('lowerarm_' + side, lower)
        for finger in ['index', 'middle', 'ring', 'pinky']:
            for segment in ['01', '02', '03']:
                key = finger + '_' + segment + '_' + side
                if key in arm.pose.bones:
                    arm.pose.bones[key].rotation_quaternion = Quaternion(Vector((1, 0, 0)), 0.2 * guard)
    gaze = 0.08 + 0.12 * power if name == 'low_kick' else 0.08 + 0.05 * power
    rotation('head', Quaternion(Vector((1, 0, 0)), gaze) @ arm.pose.bones['head'].matrix.to_quaternion())
    bpy.context.view_layer.update()
    return legs
allrows = {}
contacts = []
connected = [('thigh_l', 'calf_l'), ('calf_l', 'foot_l'), ('thigh_r', 'calf_r'), ('calf_r', 'foot_r'), ('upperarm_l', 'lowerarm_l'), ('lowerarm_l', 'hand_l'), ('upperarm_r', 'lowerarm_r'), ('lowerarm_r', 'hand_r')]
max_gap = 0.0
max_length_error = 0.0
max_nonroot_location = 0.0
max_scale_error = 0.0
min_floor = 1.0
max_reach = 0.0
max_flex = 0.0
minimum_quat_dot = 1.0
for name in ['low_kick', 'mid_kick']:
    action = bpy.data.actions.new(name)
    arm.animation_data.action = action
    previous = {}
    rows = []
    for f in range(361):
        t = f / 60
        scene.frame_set(f)
        legs = pose(t, name)
        floor = floor_probe()
        minimum = min((v['min_y_m'] for v in floor.values()))
        min_floor = min(min_floor, minimum)
        assert minimum >= -2e-05, ('floor penetration', name, t, floor)
        assert abs(floor['Sole_l']['min_y_m']) < 2e-05
        for pb in arm.pose.bones:
            q = pb.rotation_quaternion.copy()
            q.normalize()
            if pb.name in previous and q.dot(previous[pb.name]) < 0:
                q.negate()
            if pb.name in previous:
                minimum_quat_dot = min(minimum_quat_dot, q.dot(previous[pb.name]))
            pb.rotation_quaternion = q
            previous[pb.name] = q.copy()
            pb.keyframe_insert('rotation_quaternion', frame=f, group=pb.name)
            if pb.name == 'Root':
                pb.keyframe_insert('location', frame=f, group=pb.name)
            else:
                max_nonroot_location = max(max_nonroot_location, pb.location.length)
            max_scale_error = max(max_scale_error, max((abs(x - 1) for x in pb.scale)))
            max_length_error = max(max_length_error, abs((pb.tail - pb.head).length - rig.bones[pb.name].length))
        for parent, child in connected:
            max_gap = max(max_gap, (arm.pose.bones[parent].tail - arm.pose.bones[child].head).length)
        max_reach = max(max_reach, max((x['reach_ratio'] for x in legs)))
        max_flex = max(max_flex, max((x['knee_flexion_rad'] for x in legs)))
        rows.append({'time_s': t, 'root_location_m': list(arm.pose.bones['Root'].location), 'bone_quaternions': {pb.name: list(pb.rotation_quaternion) for pb in arm.pose.bones}, 'legs': legs, 'floor': floor})
        if f in (0, 39, 72, 99, 120, 126, 132, 150, 177, 210, 360):
            contacts.append({'animation': name, 'time_s': t, 'pelvis_yup_m': [arm.pose.bones['pelvis'].head.x, arm.pose.bones['pelvis'].head.z, -arm.pose.bones['pelvis'].head.y], 'toe': toe_probe(), 'legs': legs})
    for fc in action.fcurves:
        for key in fc.keyframe_points:
            key.interpolation = 'LINEAR'
    track = arm.animation_data.nla_tracks.new()
    track.name = name
    track.strips.new(name, 0, action)
    track.mute = True
    arm.animation_data.action = None
    allrows[name] = rows
assert max_gap < 2e-05 and max_length_error < 2e-05 and (max_nonroot_location == 0) and (max_scale_error == 0)
(P / 'v5-pose-samples.json').write_text(json.dumps(allrows, indent=2) + '\n')
(P / 'v5-pre-export-checks.json').write_text(json.dumps({'contact_and_stage_probes': contacts, 'max_connected_joint_gap_m': max_gap, 'max_bone_length_error_m': max_length_error, 'max_nonroot_local_translation_m': max_nonroot_location, 'max_scale_error': max_scale_error, 'minimum_floor_y_m': min_floor, 'maximum_reach_ratio': max_reach, 'maximum_knee_flexion_rad': max_flex, 'minimum_adjacent_quaternion_dot': minimum_quat_dot, 'shoe_strike_angle_deg': 10.0, 'target_ankles_blender_m': {k: list(v) for k, v in TARGETS.items()}, 'motion_fps': 60, 'time_origin_s': 0.0, 'visual_animation_only': True}, indent=2) + '\n')
arm.animation_data.action = None
for tr in arm.animation_data.nla_tracks:
    tr.mute = False
scene.frame_set(0)
bpy.context.view_layer.update()
bpy.ops.object.select_all(action='DESELECT')
arm.select_set(True)
for name in names:
    bpy.data.objects[name].select_set(True)
bpy.context.view_layer.objects.active = arm
bpy.ops.export_scene.gltf(filepath=str(O / 'cc0_sport_human.glb'), export_format='GLB', use_selection=True, export_animations=True, export_animation_mode='NLA_TRACKS', export_nla_strips=True)
for tr in arm.animation_data.nla_tracks:
    tr.mute = True
arm.animation_data.action = bpy.data.actions['mid_kick']
scene.frame_set(120)
bpy.ops.wm.save_as_mainfile(filepath=str(O / 'cc0_sport_human.blend'))
print('V5 rotation-only generation and per-frame anatomy checks completed')
