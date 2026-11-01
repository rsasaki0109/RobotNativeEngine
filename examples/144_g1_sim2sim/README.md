# 144 · G1 and H1 sim2sim: pretrained walking policies on RNE

![The G1 walks a 0.6 m circle in RNE; its pelvis path (blue) lies on MuJoCo's (orange)](../../docs/media/g1-sim2sim.png)

[GIF](../../docs/media/g1-sim2sim.gif) · [metadata](../../docs/media/g1-sim2sim.json)

Unitree's [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)
ships a G1 walking policy trained in Isaac Gym and a script that deploys it in
MuJoCo. This example runs the same policy, unchanged, on RNE's Rapier backend
and measures how far RNE's walking lands from MuJoCo's. The same policy and
loop also run in the [browser playground](../../web/rne_web_viewer/README.md#physics-walking),
driven from the keyboard in real time.

```bash
cargo run --release -p g1_sim2sim --example 144_g1_sim2sim                  # 4 commands vs MuJoCo
cargo run --release -p g1_sim2sim --example 144_g1_sim2sim -- --timing      # solver-iteration sweep
cargo run --release -p g1_sim2sim --example 144_g1_sim2sim -- --ablation    # plant details
cargo run --release -p g1_sim2sim --example 144_g1_sim2sim -- --command 0.5,0,0 --seconds 20
cargo run --release -p g1_sim2sim --example 144_g1_sim2sim -- --robot h1     # the H1 policy
```

Reports go to `target/g1-sim2sim/g1-sim2sim-report.json`; a single
`--command` also writes every policy step to `g1-sim2sim-records.json`.

## What runs

- **Policy.** `motion.pt`'s actor (LSTM 47→64, Linear 64→32, ELU,
  Linear 32→12) evaluated by `rne_ai::LstmPolicy` from
  [`assets/policies/unitree_g1_walk`](../../assets/policies/unitree_g1_walk).
  The weights were copied byte for byte out of the TorchScript archive; nothing
  in it was unpickled or executed. On MuJoCo's recorded observations the Rust
  policy reproduces MuJoCo's actions within 1e-4 (`policy.rs` test).
- **Loop.** The deployment in `deploy/deploy_mujoco/configs/g1.yaml`: PD torque
  `kp (target − q) − kd q̇` recomputed every 2 ms physics step and clamped to the
  URDF effort limits, policy at 50 Hz, target = 0.25 · action + default pose,
  gait phase period 0.8 s. `rne_ai::UnitreeRlGymWalker` evaluates the same law
  on cached joint handles.
- **Observation.** Pelvis angular velocity in the pelvis frame, gravity
  projected into the pelvis frame, scaled command, joint positions and
  velocities, previous action, gait phase. RNE is Y-up; the example converts
  to the policy's Z-up frame.
- **Plant.** The 12-DoF model the policy was trained on: legs actuated,
  32.107 kg, four 5 mm foot spheres per ankle as the only colliders (as in the
  MuJoCo scene while walking). The 18 fixed-joint links of the upper body are
  merged into the pelvis, as the MJCF does; `prepare_model.py` checks the merged
  mass, centre of mass and inertia against the MJCF's pelvis. MJCF joint
  `armature` (0.01 kg·m²), `damping` and `frictionloss`, and MuJoCo's default
  friction of 1 are reproduced; see `PlantConfig`.

## Results

Ten seconds per command, second-half averages, same weights and loop on both
engines (MuJoCo 3.9.0, `fixtures/mujoco_reference.json`):

| Command (vx, yaw) | Forward speed RNE / MuJoCo (m/s) | Yaw rate RNE / MuJoCo (rad/s) | Mean pelvis height RNE / MuJoCo (m) |
| --- | --- | --- | --- |
| 0.5, 0 | 0.463 / 0.476 | −0.011 / −0.017 | 0.770 / 0.770 |
| 0, 0 | 0.010 / 0.014 | −0.010 / −0.010 | 0.777 / 0.778 |
| 0.3, 0.5 | 0.278 / 0.280 | 0.455 / 0.452 | 0.771 / 0.772 |
| 0.8, 0 | 0.741 / 0.758 | −0.024 / −0.020 | 0.764 / 0.765 |

No run falls on either engine. The mean leg posture at 0.5 m/s matches within
0.004 rad (knee 0.342 / 0.338 rad from the default pose). Removing armature or
joint friction, or halving floor friction, changes forward speed by under
0.01 m/s; the policy is robust to those details.

**The gap that mattered was mass.** The upstream URDF has four sensor frames
(two IMUs, the D435 and the MID-360) without `<inertial>`. URDF gives them no
mass, but RNE's importer gives such links 1 kg each, so the first port weighed
36.1 kg with the extra mass high on the torso. The policy compensated by
crouching: at 0.5 m/s the knee bent 0.510 rad instead of 0.338 and the pelvis
rode 19 mm low (0.751 m vs 0.770 m). The derived model now merges those frames
into the pelvis with no mass, and a test pins the plant at 32.107 kg.
`unitree_g1_backflip_probe` and
`unitree_g1_29dof_dex3_fixed`, which also import declared masses, still carry
the 1 kg frames.

## H1

`--robot h1` runs unitree_rl_gym's H1 policy, which has the same network with
ten leg actions, on the H1 the same way. Its model merges the 11 fixed-joint
links into the pelvis (51.650 kg, matching the MJCF's pelvis inertial to
5.8e-7 kg·m²). Every link collides through the convex hull of its mesh, as the
MJCF's mesh geoms do, so the feet are hulls of the ankle meshes rather than
spheres. Same metrics and engines as above (MuJoCo reference in
`fixtures/h1_mujoco_reference.json`):

| Command (vx, yaw) | Forward speed RNE / MuJoCo (m/s) | Yaw rate RNE / MuJoCo (rad/s) | Mean pelvis height RNE / MuJoCo (m) |
| --- | --- | --- | --- |
| 0.5, 0 | 0.458 / 0.431 | −0.015 / −0.010 | 1.030 / 1.030 |
| 0, 0 | −0.054 / −0.078 | −0.023 / −0.018 | 1.033 / 1.032 |
| 0.3, 0.5 | 0.236 / 0.213 | 0.433 / 0.433 | 1.024 / 1.026 |
| 0.8, 0 | 0.751 / 0.731 | 0.020 / 0.015 | 1.024 / 1.024 |

No run falls on either engine and the pelvis height matches within 2 mm. RNE
walks 0.02–0.03 m/s faster than MuJoCo at every command, about twice the G1's
gap. Armature, joint friction and floor friction each move it by under
0.01 m/s, which leaves the contact model as the likely cause: the same hull
feet are rigid in Rapier and soft in MuJoCo. One H1 runs at 4.5–4.8× real time
with four solver iterations.

## Real time

Simulated seconds per wall second for G1 walkers at 0.5 m/s, 10 s each, in
the 4-core development container. RNE times its whole loop (physics, PD and
policy, all in Rust), one walker per thread. MuJoCo 3.9.0 runs the same loop
with PD and policy in numpy, one process per walker; its first row counts only
the time inside `mj_step`. Medians of five runs, alternating with a build of
the previous engine ("before"):

| Engine and setting | 1 walker | 2 | 4 | 8 |
| --- | --- | --- | --- | --- |
| MuJoCo, `mj_step` only | 19.1 | 36.3 | 72.4 | 69.9 |
| MuJoCo, loop with numpy PD and policy | 15.1 | 28.4 | 56.4 | 54.4 |
| RNE, 1 solver iteration | 16.4 | 27.9 | 45.8 | 54.9 |
| RNE, 1 solver iteration, before | 11.6 | 21.3 | 40.0 | 38.7 |
| RNE, 4 solver iterations (default) | 7.1 | 13.4 | 25.7 | 25.3 |
| RNE, 4 solver iterations, before | 4.5 | 8.7 | 14.6 | 15.5 |

- **The walk is unchanged.** The speedups reproduce every floating-point
  result, so the walk records are byte-identical before and after. Every
  setting walks: 0.457 m/s at one iteration, 0.464 m/s at four, against
  MuJoCo's 0.476 m/s.
- **One walker.** At one solver iteration RNE runs at 86% of MuJoCo's physics
  alone, and slightly faster than MuJoCo's loop once numpy computes PD and
  policy. At four iterations it reaches 37% of MuJoCo's physics.
- **Several walkers.** MuJoCo processes scale almost linearly to the four
  cores; RNE threads scale less well, so four walkers reach 63% of MuJoCo's
  physics throughput at one iteration and 35% at four. Runs vary by up to 40%
  between repeats on this shared container.
- **Where the time went.** Callgrind counts the instructions for 2 s of
  walking. At four iterations they fall from 2.88 G to 1.92 G (−33%); at one
  iteration from 1.09 G to 0.83 G (−25%). Rapier rebuilds and refactorizes
  the multibody mass matrices in every solver substep. That took 61% of the
  step and now takes 50%: each leg link's Jacobian is zero outside its
  ancestors' 7–12 coordinates, and the vendored Rapier skips those columns
  while computing the rest in nalgebra's exact order
  ([`RNE_PATCH.md`](../../third_party/rapier3d/RNE_PATCH.md)). A joint's limit
  and motor constraints now share one `M⁻¹e` solve. The backend sorts the ECS
  entities once per synchronization instead of six times, and reads a joint's
  position and velocity with one decode.

```bash
cargo run --release -p g1_sim2sim --example 144_g1_sim2sim -- --throughput --output target/g1-sim2sim
python3 examples/144_g1_sim2sim/mujoco_throughput.py UNITREE_RL_GYM target/g1-sim2sim/mujoco-throughput.json
```

[Medians and individual runs](../../docs/media/g1-sim2sim-throughput.json).
`--timing` still sweeps 1–32 solver iterations for one walker. Before #502
merged the fixed upper body (31 links to 13) and cached joint handles, the
first port ran at 2.6× real time with four iterations.

In the browser the same code compiled to WebAssembly keeps real time: headless
Chromium in this container (WebGPU on SwiftShader, no GPU) held a real-time
factor of 0.99–1.01 over a 21 s session of standing, walking, turning and a
side push, with physics and policy using 27–32% of each simulated second.

## Reproducing the reference

The MuJoCo side needs `mujoco` and `numpy` and an unitree_rl_gym checkout
(commit `276801e`) for its MJCF scene:

```bash
python3 examples/144_g1_sim2sim/extract_policy.py UNITREE_RL_GYM assets/policies/unitree_g1_walk
python3 examples/144_g1_sim2sim/prepare_model.py UNITREE_RL_GYM
python3 examples/144_g1_sim2sim/mujoco_reference.py UNITREE_RL_GYM \
  examples/144_g1_sim2sim/fixtures/mujoco_reference.json \
  --fixture examples/144_g1_sim2sim/fixtures/mujoco_policy_io.json
# The H1: the same three scripts with --robot h1, writing
# assets/policies/unitree_h1_walk, assets/robots/h1_description/h1.rne.urdf and
# fixtures/h1_mujoco_reference.json / h1_mujoco_policy_io.json.
cargo run --release -p g1_sim2sim --example 144_g1_sim2sim -- --render target/g1-sim2sim-render
python3 examples/144_g1_sim2sim/encode.py target/g1-sim2sim-render docs/media/g1-sim2sim
```

`mujoco_reference.py` runs the policy from the same weight blob in numpy and
shares its walking metrics with Rust through `metrics.py`.

## Limits

- Rapier contact is rigid; MuJoCo's is soft. The foot spheres are 5 mm, so
  the comparison is sensitive to contact modelling, and these four commands on
  flat ground are the whole test.
- RNE regularizes `frictionloss` as `0.1 · tanh(q̇ / 0.05)`; MuJoCo treats it
  as a constraint.
- Observations are ideal simulator state. There is no sensor noise, latency or
  domain randomization, and nothing here says the policy works on hardware.
