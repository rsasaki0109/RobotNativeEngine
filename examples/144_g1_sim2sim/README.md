# 144 · G1 sim2sim: a pretrained walking policy on RNE

![The G1 walks a 0.6 m circle in RNE; its pelvis path (blue) lies on MuJoCo's (orange)](../../docs/media/g1-sim2sim.png)

[GIF](../../docs/media/g1-sim2sim.gif) · [metadata](../../docs/media/g1-sim2sim.json)

Unitree's [unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)
ships a G1 walking policy trained in Isaac Gym and a script that deploys it in
MuJoCo. This example runs the same policy, unchanged, on RNE's Rapier backend
and measures how far RNE's walking lands from MuJoCo's.

```bash
cargo run --release -p g1_sim2sim --example 144_g1_sim2sim                  # 4 commands vs MuJoCo
cargo run --release -p g1_sim2sim --example 144_g1_sim2sim -- --timing      # solver-iteration sweep
cargo run --release -p g1_sim2sim --example 144_g1_sim2sim -- --ablation    # plant details
cargo run --release -p g1_sim2sim --example 144_g1_sim2sim -- --command 0.5,0,0 --seconds 20
```

Reports go to `target/g1-sim2sim/g1-sim2sim-report.json`; a single
`--command` also writes every policy step to `g1-sim2sim-records.json`.

## What runs

- **Policy.** `motion.pt`'s actor (LSTM 47→64, Linear 64→32, ELU,
  Linear 32→12) evaluated in Rust from
  [`assets/policies/unitree_g1_walk`](../../assets/policies/unitree_g1_walk).
  The weights were copied byte for byte out of the TorchScript archive; nothing
  in it was unpickled or executed. On MuJoCo's recorded observations the Rust
  policy reproduces MuJoCo's actions within 1e-4 (`policy.rs` test).
- **Loop.** The deployment in `deploy/deploy_mujoco/configs/g1.yaml`: PD torque
  `kp (target − q) − kd q̇` recomputed every 2 ms physics step and clamped to the
  URDF effort limits, policy at 50 Hz, target = 0.25 · action + default pose,
  gait phase period 0.8 s. RNE's portable PD
  (`UrdfSceneSim::step_joint_pd_effort_targets_substeps`) evaluates the same law.
- **Observation.** Pelvis angular velocity in the pelvis frame, gravity
  projected into the pelvis frame, scaled command, joint positions and
  velocities, previous action, gait phase. RNE is Y-up; the example converts
  to the policy's Z-up frame.
- **Plant.** The 12-DoF model the policy was trained on: legs actuated, upper
  body welded, 32.107 kg, four 5 mm foot spheres per ankle as the only
  colliders (as in the MuJoCo scene while walking). MJCF joint `armature`
  (0.01 kg·m²), `damping` and `frictionloss`, and MuJoCo's default friction of 1
  are reproduced; see `PlantConfig`.

## Results

Ten seconds per command, second-half averages, same weights and loop on both
engines (MuJoCo 3.9.0, `fixtures/mujoco_reference.json`):

| Command (vx, yaw) | Forward speed RNE / MuJoCo (m/s) | Yaw rate RNE / MuJoCo (rad/s) | Mean pelvis height RNE / MuJoCo (m) |
| --- | --- | --- | --- |
| 0.5, 0 | 0.463 / 0.476 | −0.011 / −0.017 | 0.770 / 0.770 |
| 0, 0 | 0.010 / 0.014 | −0.010 / −0.010 | 0.777 / 0.778 |
| 0.3, 0.5 | 0.278 / 0.280 | 0.455 / 0.452 | 0.771 / 0.772 |
| 0.8, 0 | 0.740 / 0.758 | −0.024 / −0.020 | 0.764 / 0.765 |

No run falls on either engine. The mean leg posture at 0.5 m/s matches within
0.004 rad (knee 0.342 / 0.338 rad from the default pose). Removing armature or
joint friction, or halving floor friction, changes forward speed by under
0.01 m/s; the policy is robust to those details.

**The gap that mattered was mass.** The upstream URDF has four sensor frames
(two IMUs, the D435 and the MID-360) without `<inertial>`. URDF gives them no
mass, but RNE's importer gives such links 1 kg each, so the first port weighed
36.1 kg with the extra mass high on the torso. The policy compensated by
crouching: at 0.5 m/s the knee bent 0.510 rad instead of 0.338 and the pelvis
rode 19 mm low (0.751 m vs 0.770 m). `prepare_model.py` gives those frames the
1e-9 kg marker inertials that example 139 already uses, and a test pins the
plant at 32.107 kg. `unitree_g1_backflip_probe` and
`unitree_g1_29dof_dex3_fixed`, which also import declared masses, still carry
the 1 kg frames.

## Real time

One G1 in the 4-core development container, 10 s at 0.5 m/s, wall time of the
whole loop (physics, PD and policy):

| Solver iterations | 1 | 2 | 4 (default) | 8 | 16 | 32 |
| --- | --- | --- | --- | --- | --- | --- |
| Real-time factor | 6.7× | 4.1× | 2.6× | 1.3× | 0.7× | 0.4× |

Every setting walks (forward speed 0.456–0.463 m/s). MuJoCo 3.9.0 steps the
same model in 98 µs (20× real time) on the same machine, about eight times
RNE's 0.77 ms at four iterations. RNE is fast enough for an interactive
single-robot playground, not yet for MuJoCo-class batch throughput.

## Reproducing the reference

The MuJoCo side needs `mujoco` and `numpy` and an unitree_rl_gym checkout
(commit `276801e`) for its MJCF scene:

```bash
python3 examples/144_g1_sim2sim/extract_policy.py UNITREE_RL_GYM assets/policies/unitree_g1_walk
python3 examples/144_g1_sim2sim/prepare_model.py UNITREE_RL_GYM
python3 examples/144_g1_sim2sim/mujoco_reference.py UNITREE_RL_GYM \
  examples/144_g1_sim2sim/fixtures/mujoco_reference.json \
  --fixture examples/144_g1_sim2sim/fixtures/mujoco_policy_io.json
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
