# Unitree G1 model provenance

- Source: `unitreerobotics/unitree_ros`
- Commit: `d96d8f63ae17a7108d4f7229c00ef875ba7129c9`
- Upstream paths:
  - `robots/g1_description/g1_23dof.urdf`
  - `robots/g1_description/g1_29dof_with_hand.urdf`
- License: BSD-3-Clause; see `LICENSE.unitree_ros`

The 23-DoF and 29-DoF-with-Dex3 URDFs and the STL files they reference are
vendored unchanged. Shared meshes are stored once; the integrated model adds
the waist/wrist links and seven actuated Dex3 joints per hand.
RNE applies the Z-up to Y-up conversion in its G1 robot asset descriptors. The
29-DoF + Dex3 task keeps the upstream model unchanged, disables mesh-derived
collision approximations, and adds backend-neutral fingertip sensor bodies in
the Episode for deterministic two-sided contact gating.

## 12-DoF walking model (unitree_rl_gym)

- Source: `unitreerobotics/unitree_rl_gym`
- Commit: `276801e46c5d433564f24658bac64f254b7d2d4b`
- Upstream path: `resources/robots/g1_description/g1_12dof.urdf`
  (SHA-256 `52a0f729252cba764f610136f0772573e55694024b888da93abd5ad23aba0152`)
- License: BSD-3-Clause; see `LICENSE.unitree_rl_gym`

`g1_12dof.rne.urdf` is generated from that file by
`examples/144_g1_sim2sim/prepare_model.py`. The 18 links fixed to the pelvis
(torso, head, logo, arms, hands, pelvis contour and four inertia-less sensor
frames) are merged into the pelvis, as the upstream MJCF does: masses add, the
inertia uses the parallel-axis theorem, and the script stops unless the result
matches the MJCF's pelvis inertial. Their visuals and collisions move onto the
pelvis with composed transforms. The torso mesh `torso_link_23dof_rev_1_0.STL`
(7.8 MB, not vendored) is replaced by the vendored 23-DoF `torso_link.STL`,
placed where the 23-DoF waist joint puts it. Leg links, joints, limits and the
four 5 mm foot spheres per ankle are unchanged. The walking descriptor
`../unitree_g1_12dof_walk.rne.robot.toml` imports only primitive collision
geometry, so the feet are the only colliders, matching the upstream MuJoCo
scene during walking. The pretrained policy that drives it is under
`assets/policies/unitree_g1_walk`.
