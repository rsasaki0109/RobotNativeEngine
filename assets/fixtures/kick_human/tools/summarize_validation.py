"""Summarize generated GLB and the generator's finite anatomy reports."""
import argparse
import hashlib
import json
from pathlib import Path
import struct


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('generated_dir', type=Path)
    args = parser.parse_args()
    root = args.generated_dir
    path = root / 'cc0_sport_human.glb'
    data = path.read_bytes()
    size, kind = struct.unpack_from('<II', data, 12)
    assert kind == 0x4e4f534a
    doc = json.loads(data[20:20 + size])
    base = 28 + size

    def values(index):
        accessor = doc['accessors'][index]
        assert accessor['componentType'] == 5126
        dimensions = {'SCALAR': 1, 'VEC3': 3, 'VEC4': 4}[accessor['type']]
        view = doc['bufferViews'][accessor['bufferView']]
        offset = base + view.get('byteOffset', 0) + accessor.get('byteOffset', 0)
        stride = view.get('byteStride', 4 * dimensions)
        return [struct.unpack_from('<' + 'f' * dimensions, data, offset + stride * n)
                for n in range(accessor['count'])]

    clips = []
    for animation in doc['animations']:
        maximum_norm_error = 0.0
        minimum_dot = 1.0
        nonroot_translation_change = 0.0
        scale_error = 0.0
        scale_variation = 0.0
        start = 1e10
        end = -1e10
        for channel in animation['channels']:
            sampler = animation['samplers'][channel['sampler']]
            assert sampler.get('interpolation', 'LINEAR') in ('LINEAR', 'STEP')
            times = values(sampler['input'])
            start = min(start, times[0][0])
            end = max(end, times[-1][0])
            samples = values(sampler['output'])
            target = channel['target']
            node_name = doc['nodes'][target['node']].get('name')
            if target['path'] == 'rotation':
                maximum_norm_error = max(maximum_norm_error,
                    max(abs(sum(x*x for x in q) - 1) for q in samples))
                minimum_dot = min(minimum_dot,
                    min((sum(x*y for x, y in zip(a, b)) for a, b in zip(samples, samples[1:])), default=1.0))
            elif target['path'] == 'translation' and node_name != 'Root':
                nonroot_translation_change = max(nonroot_translation_change,
                    max(abs(x-y) for sample in samples for x, y in zip(sample, samples[0])))
            elif target['path'] == 'scale':
                scale_error = max(scale_error, max(abs(x-1) for q in samples for x in q))
                scale_variation = max(scale_variation, max(abs(x-y) for q in samples for x,y in zip(q,samples[0])))
        assert maximum_norm_error < 2e-5 and minimum_dot >= 0
        assert nonroot_translation_change < 2e-6 and scale_error < 5e-6 and scale_variation == 0
        assert start == 0 and end == 6
        clips.append({'name': animation['name'], 'start_s': start, 'end_s': end,
                      'maximum_quaternion_norm_error': maximum_norm_error,
                      'minimum_adjacent_quaternion_dot': minimum_dot,
                      'maximum_nonroot_translation_change_m': nonroot_translation_change,
                      'maximum_scale_error': scale_error, 'maximum_scale_variation': scale_variation})
    generation = json.loads((root / 'v5-pre-export-checks.json').read_text())
    anatomy = json.loads((root / 'subframe-anatomy-audit.json').read_text())
    for clip in anatomy.values():
        assert clip['minimum_floor_y_m'] > -0.0005
        assert clip['max_support_ankle_drift_m'] < 0.0005
        assert clip['maximum_connected_joint_gap_m'] < 2e-5
        assert clip['maximum_bone_length_error_m'] < 2e-5
        assert clip['maximum_nonroot_local_translation_m'] == 0
        assert clip['maximum_scale_error'] == 0
    print(json.dumps({'verdict': 'pass', 'asset': path.name,
                      'asset_sha256': hashlib.sha256(data).hexdigest(),
                      'asset_size_bytes': len(data), 'joints': len(doc['skins'][0]['joints']),
                      'clips': clips, 'generation': {k: v for k, v in generation.items()
                          if k != 'contact_and_stage_probes'},
                      'subframe_anatomy': {k: {a: b for a, b in v.items()
                          if a != 'floor_samples'} for k, v in anatomy.items()},
                      'visual_animation_only': True}, indent=2))


if __name__ == '__main__':
    main()
