"""Per-lane bounded-reference PID controller; simulation-time state only."""
import copy
import numpy as np

from rne_gym import ArmPoseVectorEnv

DEFAULTS = {"dt_s": 0.004166667, "settle_steps": 120,
            "reference_speed_rad_s": 0.5, "integral_gain_nm_per_rad_s": [0.4, 4.0, 3.0],
            "integral_limit_nm": [0.3, 1.0, 0.7]}


class ArmPoseController:
    """Slew-limit joint references, then apply capped PID with conditional integration."""

    def __init__(self, num_envs, settings):
        settings = copy.deepcopy(settings)
        if not isinstance(num_envs, int) or num_envs < 1:
            raise ValueError("num_envs must be positive")
        if (not isinstance(settings["settle_steps"], int) or settings["settle_steps"] < 0
                or not np.isfinite(settings["dt_s"]) or settings["dt_s"] <= 0
                or not np.isfinite(settings["reference_speed_rad_s"])
                or settings["reference_speed_rad_s"] <= 0):
            raise ValueError("invalid controller timing")
        for key in ("integral_gain_nm_per_rad_s", "integral_limit_nm"):
            values = np.asarray(settings[key])
            if values.shape != (3,) or not np.isfinite(values).all() or (values < 0).any():
                raise ValueError("integral parameters must be three finite nonnegative values")
        self.settings = settings
        self.reference = np.zeros((num_envs, 3))
        self.integral = np.zeros((num_envs, 3))
        self.steps = np.zeros(num_envs, dtype=np.int64)

    def reset(self, mask):
        """Clear only reconstructed lanes, including their reference and integrator."""
        mask = np.asarray(mask)
        if mask.shape != self.steps.shape or mask.dtype != np.bool_:
            raise ValueError("reset mask must be a boolean array of num_envs")
        self.reference[mask] = 0
        self.integral[mask] = 0
        self.steps[mask] = 0

    def __call__(self, observations, names, caps, log_gains, hold):
        dt = self.settings["dt_s"]
        moving = self.steps >= self.settings["settle_steps"]
        delta = self.settings["reference_speed_rad_s"] * dt
        self.reference[moving] += np.clip(
            observations[moving, -5:-2] - self.reference[moving], -delta, delta)
        gains = np.exp(log_gains).reshape(len(observations), 3, 2)
        actions = np.zeros((len(observations), len(names)))
        for j, name in enumerate(names):
            if name not in hold:
                continue
            g = hold[name]
            cap = g["effort_limit_nm"]
            if name in ArmPoseVectorEnv.target_joints:
                k = ArmPoseVectorEnv.target_joints.index(name)
                error = self.reference[:, k] - observations[:, j]
                pd = gains[:, k, 0] * error - gains[:, k, 1] * observations[:, j + len(names)]
                increment = self.settings["integral_gain_nm_per_rad_s"][k] * error * dt
                limit = self.settings["integral_limit_nm"][k]
                candidate = np.clip(self.integral[:, k] + increment, -limit, limit)
                # Integrate only when unsaturated or when the change unwinds saturation.
                proposed = pd + candidate
                accept = ((np.abs(proposed) <= cap)
                          | ((proposed > cap) & (increment < 0))
                          | ((proposed < -cap) & (increment > 0)))
                self.integral[:, k] = np.where(accept, candidate, self.integral[:, k])
                torque = pd + self.integral[:, k]
            else:
                torque = -g["stiffness_nm_per_rad"] * observations[:, j] - g["damping_nm_s_per_rad"] * observations[:, j + len(names)]
            actions[:, j] = np.clip(torque, -cap, cap) / caps[:, j]
        self.steps += 1
        return actions
