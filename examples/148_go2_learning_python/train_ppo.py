"""Train example 147's Go2 walk with Stable-Baselines3 PPO.

The environments are `rne_py_locomotion.Go2LearningBatch`, stepped in parallel in one
Rust call per control period. Training runs in stages: on gaps,
`--curriculum` gives each stage its range of gap widths and optionally its
depth (easy first), and without it every stage is the task itself. After
each stage the deterministic policy walks one episode in each of
`--eval-envs` held-out environments, on the task itself and on the stage's
own gaps when they differ. The results file records the return, how far
it walked, how many gaps it crossed, how often it fell, and how its
episodes ended: with a foot in a gap, or stopped short of one.

With `--depth-levels N` the gaps deepen in N levels along y instead, up
to the task's 30 cm, and each environment moves up a level when an episode
crosses `--promote-after-gaps` gaps and down one when it crosses none.
Each evaluation then also walks the policy at every level (`by_level`) and
records how many environments trained at each (`training_levels`).
`--smoke` trains a few hundred steps on every path for CI.

    .venv/bin/python examples/148_go2_learning_python/train_ppo.py --smoke
    .venv/bin/python examples/148_go2_learning_python/train_ppo.py \\
        --floor gaps --height-scan --timesteps 20000000 --n-envs 64 \\
        --curriculum 0.25-0.35@0.03,0.25-0.35@0.08,0.25-0.35@0.15,0.25-0.35@0.3
    .venv/bin/python examples/148_go2_learning_python/train_ppo.py \\
        --floor gaps --height-scan --timesteps 50000000 --n-envs 64 --depth-levels 8
"""

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import VecNormalize
except ImportError:
    sys.exit(
        "stable-baselines3 is required. Install with: "
        "pip install 'stable-baselines3==2.6.0' --extra-index-url "
        "https://download.pytorch.org/whl/cpu"
    )

import numpy as np

from go2_vec_env import NativeGo2VecEnv
from run import task_arguments

# The gaps of example 147's gap task, in meters.
TASK_GAP_WIDTH_M = (0.25, 0.35)
TASK_GAP_DEPTH_M = 0.3
# A foot whose center is this far below the floor is in a gap: resting on
# the floor, it is a foot radius (2 cm) above it.
IN_GAP_BELOW_M = 0.02
# An episode that ends this close before a gap, upright and with no foot in
# it, stopped short of the gap.
SHORT_OF_GAP_M = 0.6
# A gap counts as crossed once the base is this far past its far edge, so
# the hind feet (0.19 m behind the base) are across too.
CROSSED_PAST_M = 0.25
# Held-out environments draw their starts from seeds this far from training's.
EVALUATION_SEED_OFFSET = 10_000


def parse_curriculum(text):
    """`0.05-0.10,0.25-0.35@0.08` → [((0.05, 0.10), 0.3), ((0.25, 0.35), 0.08)]:
    each stage's gap widths and depth (the task's 0.3 m unless given)."""
    stages = []
    for stage in text.split(","):
        widths, _, depth = stage.partition("@")
        low, high = (float(value) for value in widths.split("-"))
        stages.append(((low, high), float(depth) if depth else TASK_GAP_DEPTH_M))
    return stages


def walk_once(model, env, normalize):
    """The deterministic policy's first episode in every environment of
    `env`: (return, start x, last x, last feet, fell) each."""
    observations = env.reset()
    count = env.num_envs
    start_x = env.base_positions()[:, 0].copy()
    last_x = start_x.copy()
    last_feet = env.foot_positions()
    totals, fell, done = np.zeros(count), np.zeros(count, bool), np.zeros(count, bool)
    for _ in range(env._batch.episode_steps):
        actions, _ = model.predict(normalize(observations), deterministic=True)
        before_x, before_feet = env.base_positions()[:, 0], env.foot_positions()
        observations, rewards, dones, infos = env.step(actions)
        after_x, after_feet = env.base_positions()[:, 0], env.foot_positions()
        for index in range(count):
            if done[index]:
                continue
            totals[index] += float(rewards[index])
            if dones[index]:
                # The batch has reset this environment already: its last
                # state is the one before the final period.
                done[index] = True
                last_x[index], last_feet[index] = before_x[index], before_feet[index]
                fell[index] = not infos[index].get("TimeLimit.truncated", False)
            else:
                last_x[index], last_feet[index] = after_x[index], after_feet[index]
        if done.all():
            break
    return totals, start_x, last_x, last_feet, fell


def evaluate(model, env, normalize):
    """Return, distance, gaps crossed and falls of the deterministic policy
    over one episode in each environment of `env`."""
    totals, start_x, last_x, last_feet, fell = walk_once(model, env, normalize)
    gaps = env.gap_intervals()
    crossed, reached, fell_in_gap = [], [], 0
    front_in_gap = rear_in_gap = short_of_gap = 0
    for begin, end, feet, did_fall in zip(start_x, last_x, last_feet, fell):
        ahead = [gap for gap in gaps if gap[0] > begin]
        crossed.append(sum(1 for gap in ahead if end > gap[1] + CROSSED_PAST_M))
        # Its front feet (0.19 m ahead of the base) at the gap.
        reached.append(sum(1 for gap in ahead if end > gap[0] - 0.2))
        if did_fall and any(gap[0] - 0.3 < end < gap[1] + 0.3 for gap in gaps):
            fell_in_gap += 1
        in_gap = feet[:, 2] < -IN_GAP_BELOW_M
        front_in_gap += int(in_gap[:2].any())
        rear_in_gap += int(in_gap[2:].any() and not in_gap[:2].any())
        next_gap = min((gap[0] for gap in gaps if gap[0] > end), default=None)
        if (not did_fall and not in_gap.any() and next_gap is not None
                and next_gap - feet[:2, 0].max() < SHORT_OF_GAP_M):
            short_of_gap += 1
    return {
        "episodes": len(totals),
        "mean_return": float(totals.mean()),
        "mean_distance_m": float((last_x - start_x).mean()),
        "falls": int(fell.sum()),
        "falls_at_gaps": fell_in_gap,
        "gaps_reached": int(sum(reached)),
        "gaps_crossed": int(sum(crossed)),
        "episodes_crossing_a_gap": int(sum(1 for count in crossed if count > 0)),
        # How the episodes ended: a front foot in a gap, only a rear foot in
        # one, or upright with the front feet within SHORT_OF_GAP_M of the
        # next gap and no foot in any.
        "ended_front_foot_in_gap": front_in_gap,
        "ended_rear_foot_in_gap": rear_in_gap,
        "ended_short_of_gap": short_of_gap,
    }


def evaluate_levels(model, env, normalize, depths):
    """`evaluate` at each gap depth level of `env`, every environment
    starting at that level."""
    results = []
    for level, depth in enumerate(depths):
        env.set_levels([level] * env.num_envs)
        result = evaluate(model, env, normalize)
        result.update(level=level, gap_depth_m=depth)
        results.append(result)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--floor", choices=["flat", "terrain", "gaps"], default="flat")
    parser.add_argument("--terrain-seed", type=int, default=7)
    parser.add_argument("--height-scan", action="store_true")
    parser.add_argument("--n-envs", type=int, default=16)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--timesteps", type=int, default=2_000_000)
    parser.add_argument("--stages", type=int, default=4,
                        help="training stages without --curriculum (one evaluation each)")
    parser.add_argument("--curriculum", default="",
                        help="gap widths (and @depth) per stage, e.g. "
                             "0.05-0.10,0.25-0.35@0.08 (gaps only)")
    parser.add_argument("--depth-levels", type=int, default=1,
                        help="gaps deepen in this many levels along y, each environment "
                             "moving up and down them as it crosses gaps (gaps only)")
    parser.add_argument("--promote-after-gaps", type=int, default=2,
                        help="gaps an episode must cross to move up a depth level")
    parser.add_argument("--n-steps", type=int, default=256, help="rollout periods per env")
    parser.add_argument("--batch-size", type=int, default=4096)
    parser.add_argument("--n-epochs", type=int, default=5)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--ent-coef", type=float, default=0.0)
    parser.add_argument("--log-std-init", type=float, default=-0.5)
    parser.add_argument("--net", default="256,256", help="hidden layers of both networks")
    parser.add_argument("--no-normalize", action="store_true",
                        help="train on raw observations and rewards")
    parser.add_argument("--eval-envs", type=int, default=64)
    parser.add_argument("--out", default="target/go2_learning_ppo")
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    if not args.smoke:
        train(args, parser)
        return
    # Gaps with the scan and normalization, a few hundred steps on each
    # path: a two-stage curriculum, then depth levels.
    args.floor, args.height_scan = "gaps", True
    args.n_envs, args.n_steps, args.batch_size, args.timesteps = 2, 64, 64, 256
    args.eval_envs, args.n_epochs = 2, 1
    for curriculum, levels in (("0.05-0.10@0.1,0.25-0.35", 1), ("", 3)):
        args.curriculum, args.depth_levels, args.stages = curriculum, levels, 2
        history = train(args, parser)
        assert len(history) == 2
        assert ("on_stage_gaps" in history[0]) == bool(curriculum)
        assert ("by_level" in history[0]) == (levels > 1)
    print("smoke ok")


def train(args, parser):
    """Trains and evaluates as `args` ask; returns each stage's results."""
    task = task_arguments(args)
    levels = args.depth_levels > 1
    if levels:
        if args.floor != "gaps" or args.curriculum:
            parser.error("--depth-levels needs --floor gaps and no --curriculum")
        task["gap_levels"] = args.depth_levels
    if args.curriculum:
        if args.floor != "gaps":
            parser.error("--curriculum needs --floor gaps")
        stages = parse_curriculum(args.curriculum)
    else:
        target = (TASK_GAP_WIDTH_M, TASK_GAP_DEPTH_M) if args.floor == "gaps" else None
        stages = [target] * args.stages
    final = (TASK_GAP_WIDTH_M, TASK_GAP_DEPTH_M) if args.floor == "gaps" else None

    def stage_task(stage):
        if stage is None:
            return task
        width, depth = stage
        return {**task, "gap_width": width, "gap_depth": depth}

    def held_out(stage):
        return NativeGo2VecEnv(n_envs=args.eval_envs, seed=args.seed + EVALUATION_SEED_OFFSET,
                               **stage_task(stage))

    raw = NativeGo2VecEnv(n_envs=args.n_envs, seed=args.seed, **stage_task(stages[0]))
    if levels:
        raw.set_depth_curriculum(args.promote_after_gaps)
    env = raw if args.no_normalize else VecNormalize(raw, gamma=args.gamma)
    task_held_out = held_out(final)
    if levels:
        # The task's gaps are the deepest level's.
        depths = task_held_out.level_depths()
        task_held_out.set_levels([len(depths) - 1] * args.eval_envs)
    normalize = (lambda observations: observations) if args.no_normalize else env.normalize_obs

    layers = [int(size) for size in args.net.split(",")]
    model = PPO(
        "MlpPolicy", env,
        n_steps=args.n_steps, batch_size=args.batch_size, n_epochs=args.n_epochs,
        learning_rate=args.learning_rate, gamma=args.gamma, ent_coef=args.ent_coef,
        policy_kwargs={"net_arch": {"pi": layers, "vf": layers},
                       "activation_fn": torch.nn.ELU, "log_std_init": args.log_std_init},
        seed=args.seed, verbose=0,
    )

    stage_timesteps = args.timesteps // len(stages)
    history, started = [], time.monotonic()
    for index, stage in enumerate(stages):
        if index > 0 and stage != stages[index - 1]:
            raw.set_task(**stage_task(stage))
            model._last_obs = env.reset()
            model._last_episode_starts = np.ones((args.n_envs,), dtype=bool)
        model.learn(total_timesteps=stage_timesteps, reset_num_timesteps=False)
        result = evaluate(model, task_held_out, normalize)
        result.update(stage=index, gap_width_m=None if stage is None else stage[0],
                      gap_depth_m=None if stage is None else stage[1],
                      timesteps=model.num_timesteps,
                      seconds=round(time.monotonic() - started, 1))
        if stage != final:
            # The same policy on this stage's own gaps.
            result["on_stage_gaps"] = evaluate(model, held_out(stage), normalize)
        if levels:
            reached = raw.levels()
            result["training_levels"] = [reached.count(level) for level in range(len(depths))]
            result["by_level"] = evaluate_levels(model, task_held_out, normalize, depths)
            task_held_out.set_levels([len(depths) - 1] * args.eval_envs)
        history.append(result)
        print(json.dumps(result), flush=True)

    os.makedirs(args.out, exist_ok=True)
    summary = {
        "floor": args.floor,
        "height_scan": args.height_scan,
        "seed": args.seed,
        "n_envs": args.n_envs,
        "curriculum": args.curriculum or None,
        "depth_levels": args.depth_levels,
        "promote_after_gaps": args.promote_after_gaps if levels else None,
        "normalize": not args.no_normalize,
        "hyperparameters": {
            "n_steps": args.n_steps, "batch_size": args.batch_size,
            "n_epochs": args.n_epochs, "learning_rate": args.learning_rate,
            "gamma": args.gamma, "ent_coef": args.ent_coef,
            "log_std_init": args.log_std_init, "net": layers,
        },
        "timesteps": model.num_timesteps,
        "steps_per_second": round(model.num_timesteps / (time.monotonic() - started), 1),
        "stages": history,
        "final": history[-1],
    }
    with open(os.path.join(args.out, "results.json"), "w") as file:
        json.dump(summary, file, indent=2)
    if not args.smoke:
        model.save(os.path.join(args.out, "ppo_go2"))
        if not args.no_normalize:
            env.save(os.path.join(args.out, "vec_normalize.pkl"))
    print(json.dumps({key: summary[key] for key in ("floor", "height_scan", "timesteps",
                                                     "steps_per_second", "final")}))
    return history


if __name__ == "__main__":
    main()
