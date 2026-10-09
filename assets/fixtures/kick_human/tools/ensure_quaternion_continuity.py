"""Keep GLB LINEAR quaternion keys in one continuous hemisphere."""
import json, struct, pathlib, hashlib, sys, math
p = pathlib.Path(sys.argv[1])
data = bytearray(p.read_bytes())
size, kind = struct.unpack_from('<II', data, 12)
assert kind == 1313821514
g = json.loads(data[20:20 + size])
bin_header = 20 + size
bin_size, bin_kind = struct.unpack_from('<II', data, bin_header)
assert bin_kind == 5130562
base = bin_header + 8
fixed = 0
minimum = 1.0
seen = set()
for anim in g.get('animations', []):
    for channel in anim['channels']:
        if channel['target']['path'] != 'rotation':
            continue
        sampler = anim['samplers'][channel['sampler']]
        assert sampler.get('interpolation', 'LINEAR') in ['LINEAR', 'STEP']
        idx = sampler['output']
        if idx in seen:
            continue
        seen.add(idx)
        acc = g['accessors'][idx]
        assert acc['type'] == 'VEC4' and acc['componentType'] == 5126
        assert 'min' not in acc and 'max' not in acc
        view = g['bufferViews'][acc['bufferView']]
        offset = base + view.get('byteOffset', 0) + acc.get('byteOffset', 0)
        stride = view.get('byteStride', 16)
        prev = None
        for n in range(acc['count']):
            at = offset + n * stride
            q = struct.unpack_from('<4f', data, at)
            assert abs(sum((v * v for v in q)) - 1) < 2e-05
            if prev is not None:
                dot = sum((a * b for a, b in zip(prev, q)))
                if dot < 0:
                    q = tuple((-v for v in q))
                    struct.pack_into('<4f', data, at, *q)
                    fixed += 1
                    dot = -dot
                minimum = min(minimum, dot)
            prev = q
p.write_bytes(data)
report = {'asset': p.name, 'sha256': hashlib.sha256(data).hexdigest(), 'hemisphere_key_corrections': fixed, 'minimum_adjacent_quaternion_dot': minimum, 'rotation_accessors': len(seen), 'same_physical_rotations': True}
print(json.dumps(report, indent=2))
