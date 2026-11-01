# Unitree G1 walking policy (unitree_rl_gym)

- Source: `unitreerobotics/unitree_rl_gym` commit
  `276801e46c5d433564f24658bac64f254b7d2d4b`, `deploy/pre_train/g1/motion.pt`
  (SHA-256 `cf668f75b90d1abf73d2b87612a6e76bccc61ff7e083b63582d3f6aaa3c1759d`)
- License: BSD-3-Clause; see `LICENSE.unitree_rl_gym`

`g1_walk_lstm.f32` holds the actor's float32 tensors exactly as stored in the
TorchScript archive; `g1_walk_lstm.policy.json` lists their names, shapes and
offsets, the blob's SHA-256, and the deployment contract from
`deploy/deploy_mujoco/configs/g1.yaml` (500 Hz PD with the listed gains, a
50 Hz policy, default pose and observation scales).
`examples/144_g1_sim2sim/extract_policy.py` regenerates both from an upstream
checkout. It reads the archive as a zip of raw storages and never unpickles
or executes it; the storage-to-tensor map was read from a `pickletools`
disassembly and is checked by byte size.

The network is a one-layer LSTM (47 inputs, 64 units, gate order i, f, g, o,
zero initial state) followed by Linear(64, 32), ELU and Linear(32, 12).
`rne_ai::LstmPolicy` evaluates it and `rne_ai::UnitreeRlGymWalker` runs it on
`assets/robots/unitree_g1_12dof_walk.rne.robot.toml`; see
`examples/144_g1_sim2sim` and the browser playground's physics walking mode.
