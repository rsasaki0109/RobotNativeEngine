# Unitree H1 walking policy (unitree_rl_gym)

- Source: `unitreerobotics/unitree_rl_gym` commit
  `276801e46c5d433564f24658bac64f254b7d2d4b`, `deploy/pre_train/h1/motion.pt`
  (SHA-256 `44a0fbceb81f3877833ae9a398d039bea1759cb0d3c8188181013885f70589eb`)
- License: BSD-3-Clause; see `LICENSE.unitree_rl_gym`

`h1_walk_lstm.f32` holds the actor's float32 tensors exactly as stored in the
TorchScript archive; `h1_walk_lstm.policy.json` lists their names, shapes and
offsets, the blob's SHA-256, and the deployment contract from
`deploy/deploy_mujoco/configs/h1.yaml` with the joint order of the upstream
MJCF's actuators. `examples/144_g1_sim2sim/extract_policy.py --robot h1`
regenerates both. It reads the archive as a zip of raw storages and never
unpickles or executes it.

The network has the G1 policy's structure with ten actions: a one-layer LSTM
(41 inputs, 64 units, gate order i, f, g, o, zero initial state), then
Linear(64, 32), ELU and Linear(32, 10). `rne_ai::LstmPolicy` evaluates it and
`rne_ai::UnitreeRlGymWalker` runs it on
`assets/robots/unitree_h1_walk.rne.robot.toml`; see `examples/144_g1_sim2sim`.
