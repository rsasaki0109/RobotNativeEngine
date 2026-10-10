# CC0 anatomical kick actor

`cc0_sport_human.glb` is a 1.75 m anatomical adult character with 53 skin
joints, eight embedded procedural textures, sportswear and modeled sneakers.
Its two six-second clips, `low_kick` and `mid_kick`, visualize the prescribed
disturbances in [example 139](../../../examples/139_kick_comparison/README.md).
The character has no physics body or human-foot contact model.

The source mesh, rig, weights and adult target are CC0 assets from
MakeHuman/MPFB2 revision `d0a32e57a7f915cb2f2b95410e2117648c7bbb7e`.
Upstream distinguishes its GPLv3 program code from CC0 graphical assets and
exports in [LICENSE.UPSTREAM.md](LICENSE.UPSTREAM.md). The full CC0 text is
[LICENSE.CC0.md](LICENSE.CC0.md). Only the graphical inputs were used;
the procedural textures, shoes and generation/animation code are original.
The generated graphical asset is distributed under CC0. This is a textured
anatomical character, not photogrammetry or motion capture.
The upstream explanatory Markdown's heading and trailing spaces are normalized
here, with its original bytes pinned in the source manifest.

## Motion checks

The actor transfers weight toward its left support leg, chambers the right
knee forward, extends, retracts and plants the foot before returning to idle.
Only `Root` translates. All other bone-local offsets remain fixed and pose
scales are one; exported matrix decomposition adds a few f32 ULPs. Limb
lengths and connected joints therefore remain fixed within export precision.
All quaternion keys have consistent signs, and the runtime samples LINEAR
rotations on the shortest spherical arc for both CPU and GPU skinning.

The low and mid kicks retain about 68.93 and 44.58 degrees of knee flexion
at impact. The left sole is planted throughout; subframe interpolation
produces less than 0.08 mm of vertical drift. These are animation/geometry
checks, not a human musculoskeletal dynamics validation. The toe target
heights at impact are 0.322176 m and 0.826094 m, rounded from the first
recorded world wrench points of the robots' wider recovery stances. The
exported toe heights are within 2 micrometers of the animation targets;
the final nominal wrench points differ from those targets by less than
0.01 mm after G1 feedback tuning. Their
local front extent is 0.920001 m or less. This retargeting rotates joints
while preserving the rig, bone lengths and planted support sole.
Example 139 matches the toe's height and lateral position to the recorded
wrench point, with the toe tip about 8 cm behind it as a visual surface
offset. This positioning does not calculate a foot collision or contact
force.

Example 139 continuously retimes these clips along the same checked pose
path. Extension takes 175 ms, the impact hold lasts 80 ms to match the
prescribed wrench pulse, and retraction takes 220 ms. Impact still starts
at simulation time 2.0 s. The renderer and headless animation checker use
the same time map; these faster visual motions do not determine the robot
force or simulate human dynamics.

## Regenerate from verified inputs

Use Blender 4.3.2. Fetching inputs requires HTTPS access to
`raw.githubusercontent.com`; every input is pinned by byte size and SHA-256
in [source-manifest.json](source-manifest.json). Existing verified files are
reused. No Blender add-on or pre-existing `.blend` file is required.

```bash
cd assets/fixtures/kick_human
python3 fetch_source.py
blender -b --python tools/generate_human.py -- \
  --source-dir source --manifest source-manifest.json --output-dir generated \
  --low-strike-height-m 0.322176 --mid-strike-height-m 0.826094
python3 tools/ensure_quaternion_continuity.py generated/cc0_sport_human.glb
blender -b generated/cc0_sport_human.blend --python tools/validate_subframes.py
python3 tools/summarize_validation.py generated > generated/validation-summary.json
```

The generator uses the explicit procedural texture seed 4217 and validates
joint connections, fixed bone lengths, non-root translations, scales, reach
and shoe clearance at all source keys. The subframe checker verifies the
interpolated Blender animation; example 139 additionally checks the exported
asset at 120 Hz without a GPU. The committed asset was independently checked
from glTF accessors at 240 Hz and reviewed in actual wgpu frames.

[validation.json](validation.json) records measurements of the committed GLB.
For the separate accessor/hierarchy/skinning calculation, install NumPy and run
`python3 tools/audit_exported_motion.py cc0_sport_human.glb exported-motion-audit.json`.
This reports geometry and interpolation measurements; it is separate from
Blender's in-memory pose checks and does not simulate a human body.

Fresh generation reproduces the rig, motion, vertex positions, UVs, weights
and texture pixels. Blender's sphere triangulation can change index order,
diagonals and derived normals, so byte-identical GLB regeneration is not
claimed. Review the generated model, replace the committed GLB and regenerate
the GIF together when changing motion. Preserve the manifest and licenses.
