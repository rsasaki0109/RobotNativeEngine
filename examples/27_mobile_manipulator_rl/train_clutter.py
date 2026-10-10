"""IK-assisted CEM smoke for the fixed-base clutter pick-and-place task.

The Rust IK policy supplies the staged approach and phase timing. CEM
learns bounded shoulder/elbow tracking scales while free or grasped, plus the
contact-gated gripper velocity. All candidates are sampled; no scripted winning
candidate or success bonus is inserted. Episodes use friction grasping.

    .venv/bin/maturin develop -m crates/rne_py/Cargo.toml
    .venv/bin/python examples/27_mobile_manipulator_rl/train_clutter.py --smoke
"""

import hashlib
import random
import struct
import sys

try:
    import rne_py
except ImportError:
    sys.exit(
        "rne_py is not installed. Build it with:\n"
        "  .venv/bin/maturin develop -m crates/rne_py/Cargo.toml"
    )

ACTION_LIMIT = 6.0
# Keep the original rollout budget; the reference policy owns phase transitions.
EPISODE_STEPS = 950
ARM_LIMIT_RAD_S = 0.3  # Center-cube IK tracking limit in crates/rne_ai/src/policy.rs.
MAX_TRACKING_SCALE = 1.5
PARAM_DIM = 5  # free shoulder/elbow scales, gripper speed, grasped arm scales
TASK = "clutter_place_center"

GRIPPER_OPEN_RAD_S = 0.5
# Same IK controller as the candidates, with the fingers kept open throughout.
WEAK_BASELINE = [1.0, 1.0, GRIPPER_OPEN_RAD_S, 1.0, 1.0]
OBSERVED_FLOAT_FIELDS = (
    "base_x",
    "base_y",
    "base_z",
    "base_yaw",
    "ee_x",
    "ee_y",
    "ee_z",
    "shoulder_position",
    "elbow_position",
    "wrist_yaw_position",
    "gripper_position",
    "gripper_position_m",
    "lift_position_m",
    "target_dx",
    "target_dy",
    "target_dz",
)


def clamp(value, limit=ACTION_LIMIT):
    return max(-limit, min(limit, value))


class IkAssistedClutterPolicy:
    """Tune the IK controller without mapping Cartesian errors to joint rates."""

    def __init__(self, params):
        self.params = params
        self.reference = rne_py.IkClutterPickPlacePolicy()

    def act(self, obs):
        action = list(self.reference.act(obs))
        scales = self.params[3:5] if obs.is_grasping else self.params[:2]
        for joint, scale in zip((2, 3), scales):
            scale = max(0.0, min(MAX_TRACKING_SCALE, scale))
            action[joint] = clamp(action[joint] * scale, ARM_LIMIT_RAD_S)
        # Reuse the mount-distance gate and hysteresis from the Rust policy.
        # Apply the learned speed to every close command, including carry/hold,
        # so an open-gripper candidate cannot acquire a scripted closing phase.
        if action[4] < 0.0:
            action[4] = clamp(self.params[2])
        return action


def hash_observed_step(hasher, step, total_reward, action):
    """Hash emitted actions, observed poses, rewards, and episode end flags."""
    values = [getattr(step.observation, field) for field in OBSERVED_FLOAT_FIELDS]
    values.extend((step.reward, total_reward, *action))
    hasher.update(
        struct.pack(
            f"<{len(values)}d???",
            *values,
            step.observation.is_grasping,
            step.terminated,
            step.truncated,
        )
    )


def rollout_metrics(params, replay_hash=None):
    episode = rne_py.MobileManipulatorEpisode(TASK)
    step = episode.reset()
    episode.set_grasp_mode("friction")
    policy = IkAssistedClutterPolicy(params)
    grasped = False
    placed = False
    if replay_hash is not None:
        hash_observed_step(replay_hash, step, episode.total_reward, [0.0] * 6)
    for _ in range(EPISODE_STEPS):
        action = policy.act(step.observation)
        step = episode.step(*action)
        if replay_hash is not None:
            hash_observed_step(replay_hash, step, episode.total_reward, action)
        if episode.is_grasping:
            grasped = True
        if step.terminated:
            placed = True
        if step.done:
            break
    return episode.total_reward, grasped, placed


def cem_smoke():
    population = 16
    elite = 4
    iterations = 8
    # Scales start at nominal IK tracking; gripper sampling covers open and close.
    mean = [1.0, 1.0, 0.0, 1.0, 1.0]
    std = [0.2, 0.2, 1.5, 0.2, 0.2]
    history = []
    best_reward = float("-inf")
    best_params = mean
    best_grasped = False
    best_placed = False

    for _ in range(iterations):
        candidates = []
        for _ in range(population):
            params = [random.gauss(mean[i], std[i]) for i in range(PARAM_DIM)]
            reward, grasped, placed = rollout_metrics(params)
            candidates.append((reward, grasped, placed, params))
            if reward > best_reward:
                best_reward = reward
                best_params = params
                best_grasped = grasped
                best_placed = placed
        candidates.sort(key=lambda item: item[0], reverse=True)
        elites = candidates[:elite]
        history.append(elites[0][0])
        mean = [sum(item[3][i] for item in elites) / elite for i in range(PARAM_DIM)]
        std = [max(0.12, s * 0.9) for s in std]

    return history, best_params, best_reward, best_grasped, best_placed


def replay_best(params):
    first_hash = hashlib.sha256()
    second_hash = hashlib.sha256()
    first = rollout_metrics(params, first_hash)
    second = rollout_metrics(params, second_hash)
    if first != second or first_hash.digest() != second_hash.digest():
        sys.exit(f"replay failed: observed trajectories differ ({first} vs {second})")
    return first, first_hash.hexdigest()


def ik_policy_grasps() -> bool:
    """Reference grasp check using the Rust IK clutter policy in friction mode."""
    policy = rne_py.IkClutterPickPlacePolicy()
    episode = rne_py.MobileManipulatorEpisode(TASK)
    step = episode.reset()
    episode.set_grasp_mode("friction")
    for _ in range(policy.total_steps()):
        left, right, shoulder, elbow, gripper, lift = policy.act(step.observation)
        step = episode.step(left, right, shoulder, elbow, gripper, lift)
        if episode.is_grasping:
            return True
        if step.done:
            break
    return False


def main():
    random.seed(0)
    smoke = "--smoke" in sys.argv
    baseline, baseline_grasped, _ = rollout_metrics(WEAK_BASELINE)
    scripted_grasped = ik_policy_grasps()
    history, best_params, best_reward, best_grasped, best_placed = cem_smoke()
    (replay_reward, replay_grasped, replay_placed), replay_digest = replay_best(best_params)
    open_params = best_params.copy()
    open_params[2] = GRIPPER_OPEN_RAD_S
    _, open_grasped, _ = rollout_metrics(open_params)
    stopped_params = best_params.copy()
    stopped_params[0:2] = [0.0, 0.0]
    _, stopped_grasped, _ = rollout_metrics(stopped_params)
    print(
        "clutter CEM: "
        f"baseline={baseline:.2f} best={best_reward:.2f} "
        f"grasped={best_grasped} placed={best_placed} "
        f"scripted_grasped={scripted_grasped} "
        f"replay={replay_reward:.2f}/{replay_grasped}/{replay_placed} "
        f"open_grasped={open_grasped} stopped_grasped={stopped_grasped} "
        f"observed_sha256={replay_digest} "
        f"history={[round(x, 2) for x in history]}"
    )
    if smoke:
        if (
            best_reward > baseline + 0.5
            and best_grasped
            and scripted_grasped
            and (best_reward, best_grasped, best_placed)
            == (replay_reward, replay_grasped, replay_placed)
            and not baseline_grasped
            and not open_grasped
            and not stopped_grasped
        ):
            print("clutter smoke ok: CEM beat baseline, grasped, replay stable")
            return
        sys.exit(
            "smoke failed: clutter CEM "
            f"(baseline={baseline:.2f}, best={best_reward:.2f}, "
            f"best_grasped={best_grasped}, scripted_grasped={scripted_grasped}, "
            f"baseline_grasped={baseline_grasped}, open_grasped={open_grasped}, "
            f"stopped_grasped={stopped_grasped})"
        )


if __name__ == "__main__":
    main()
