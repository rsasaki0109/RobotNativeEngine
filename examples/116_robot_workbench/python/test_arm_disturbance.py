"""External force transport and precondition/pulse/recovery phase semantics."""
import unittest

import numpy as np

from audit_arm_disturbance import run_case
from rne_gym import ArmPoseVectorEnv
from test_rne_gym import BINARY, CONFIG


class FakePolicy:
    def reset(self, mask):
        pass

    def __call__(self, obs, names, caps, gains, hold):
        return np.zeros((2, 3))


class FakeEnv:
    target_joints = ("shoulder_pan", "shoulder_lift", "elbow_flex")
    joint_names = target_joints
    agent_ids = ["alice", "alice"]
    _caps = np.ones((2, 3))

    def __init__(self, horizon, bad_steps=(), never_stable=False):
        self.horizon, self.bad_steps, self.never_stable = horizon, bad_steps, never_stable

    def reset(self, seed):
        self.steps = 0
        return np.zeros((2, 21)), {}

    def step(self, actions, *, forces_n):
        self.steps += 1
        pulse = bool(forces_n[1])
        error = np.zeros((2, 3))
        if pulse or self.steps in self.bad_steps or self.never_stable:
            error[1, 0] = 0.1
        return np.zeros((2, 21)), np.zeros(2), np.zeros(2, dtype=bool), np.full(2, self.steps >= self.horizon), {
            "angle_error_rad": error, "fallen": np.zeros(2, dtype=bool),
            "target_rad": np.zeros((2, 3)), "state_hash": ["same", "same"]}


class ForceRecoveryTests(unittest.TestCase):
    def test_external_force_does_not_change_motor_command(self):
        with ArmPoseVectorEnv(BINARY, CONFIG, horizon=20) as env:
            env.reset(seed=[42, 42])
            _, _, _, _, before = env.step(np.zeros((2, 9)))
            self.assertEqual(before["state_hash"][0], before["state_hash"][1])
            _, _, _, _, after = env.step(np.zeros((2, 9)), forces_n=[{}, {"alice": {"wrist_flex": [0, 0, 1]}}])
            self.assertNotEqual(after["state_hash"][0], after["state_hash"][1])
            self.assertTrue(all(j["commanded_effort_nm"] == 0 for j in env._agent(1)["joints"].values()))

    def test_recovery_requires_a_new_uninterrupted_hold_after_release(self):
        protocol = {"horizon": 15, "hold_steps": 3, "pulse_steps": 2,
                    "target_joint": "wrist_flex", "force_world_n": [0, 0, 2], "dt_s": 0.004166667}
        result = run_case(FakeEnv(15, bad_steps={7}), FakePolicy(), np.zeros((2, 6)), {}, 42, protocol)
        self.assertEqual(result["pulse_start_step"], 3)
        self.assertEqual(result["delivered_force_steps"], 2)
        self.assertEqual(result["steps"], 10)
        self.assertTrue(result["lanes"][1]["success"])
        self.assertTrue(result["lanes"][1]["displaced"])
        result = run_case(FakeEnv(15, never_stable=True), FakePolicy(), np.zeros((2, 6)), {}, 42, protocol)
        self.assertFalse(result["precondition_met"])
        self.assertEqual(result["delivered_force_steps"], 0)
        self.assertFalse(result["lanes"][1]["success"])


if __name__ == "__main__":
    unittest.main()
