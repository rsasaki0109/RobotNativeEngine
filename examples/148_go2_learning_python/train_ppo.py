"""Train example 147's Go2 walk with Stable-Baselines3 PPO.

The environments are `rne_py_locomotion.Go2LearningBatch`, stepped in parallel in one
Rust call per control period. Training runs in stages: on gaps,
`--curriculum` gives each stage its range of gap widths (narrow first),
and without it every stage is the task itself. After each stage the
deterministic policy walks one episode in each of `--eval-envs` held-out
environments on the task itself, and the results file records its return,
how far it walked, how many gaps it crossed and how often it fell.
`--smoke` trains a few hundred steps on every path for CI.

    .venv/bin/python examples/148_go2_learning_python/train_ppo.py --smoke
    .venv/bin/python examples/148_go2_learning_python/train_ppo.py \\
        --floor gaps --height-scan --timesteps 20000000 --n-envs 64 \\
        --curriculum 0.05-0.10,0.10-0.20,0.18-0.28,0.25-0.35
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
# A gap counts as crossed once the base is this far past its far edge, so
# the hind feet (0.19 m behind the base) are across too.
CROSSED_PAST_M = 0.25
# Held-out environments draw their starts from seeds this far from training's.
EVALUATION_SEED_OFFSET = 10_000


def parse_widths(text):
    """`0.05-0.10,0.25-0.35` → [(0.05, 0.10), (0.25, 0.35)]."""
    stages = []
    for stage in text.split(","):
        low, high = (float(value) for value in stage.split("-"))
        stages.append((low, high))
    return stages


def walk_once(model, env, normalize):
    """The deterministic policy's first episode in every environment of
    `env`: (return, start x, last x, fell) each."""
    observations = env.reset()
    count = env.num_envs
    start_x = env.base_positions()[:, 0].copy()
    last_x = start_x.copy()
    totals, fell, done = np.zeros(count), np.zeros(count, bool), np.zeros(count, bool)
    for _ in range(env._batch.episode_steps):
        actions, _ = model.predict(normalize(observations), deterministic=True)
        before_x = env.base_positions()[:, 0]
        observations, rewards, dones, infos = env.step(actions)
        after_x = env.base_positions()[:, 0]
        for index in range(count):
            if done[index]:
                continue
            totals[index] += float(rewards[index])
            if dones[index]:
                # The batch has reset this environment already: its last
                # position is the one before the final period.
                done[index] = True
                last_x[index] = before_x[index]
                fell[index] = not infos[index].get("TimeLimit.truncated", False)
            else:
                last_x[index] = after_x[index]
        if done.all():
            break
    return totals, start_x, last_x, fell


def evaluate(model, env, normalize):
    """Return, distance, gaps crossed and falls of the deterministic policy
    over one episode in each environment of `env`."""
    totals, start_x, last_x, fell = walk_once(model, env, normalize)
    gaps = env.gap_intervals()
    crossed, reached, fell_in_gap = [], [], 0
    for begin, end, did_fall in zip(start_x, last_x, fell):
        ahead = [gap for gap in gaps if gap[0] > begin]
        crossed.append(sum(1 for gap in ahead if end > gap[1] + CROSSED_PAST_M))
        # Its front feet (0.19 m ahead of the base) at the gap.
        reached.append(sum(1 for gap in ahead if end > gap[0] - 0.2))
        if did_fall and any(gap[0] - 0.3 < end < gap[1] + 0.3 for gap in gaps):
            fell_in_gap += 1
    return {
        "episodes": len(totals),
        "mean_return": float(totals.mean()),
        "mean_distance_m": float((last_x - start_x).mean()),
        "falls": int(fell.sum()),
        "falls_at_gaps": fell_in_gap,
        "gaps_reached": int(sum(reached)),
        "gaps_crossed": int(sum(crossed)),
        "episodes_crossing_a_gap": int(sum(1 for count in crossed if count > 0)),
    }


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
                        help="gap widths per stage, e.g. 0.05-0.10,0.25-0.35 (gaps only)")
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
    if args.smoke:
        # Gaps with the scan, a two-stage curriculum and normalization: every
        # path, a few hundred steps.
        args.floor, args.height_scan = "gaps", True
        args.curriculum = "0.05-0.10,0.25-0.35"
        args.n_envs, args.n_steps, args.batch_size, args.timesteps = 2, 64, 64, 256
        args.eval_envs, args.n_epochs = 2, 1

    task = task_arguments(args)
    if args.curriculum:
        if args.floor != "gaps":
            parser.error("--curriculum needs --floor gaps")
        widths = parse_widths(args.curriculum)
    else:
        widths = [TASK_GAP_WIDTH_M if args.floor == "gaps" else None] * args.stages

    def stage_task(width):
        return task if width is None else {**task, "gap_width": width}

    raw = NativeGo2VecEnv(n_envs=args.n_envs, seed=args.seed, **stage_task(widths[0]))
    env = raw if args.no_normalize else VecNormalize(raw, gamma=args.gamma)
    held_out = NativeGo2VecEnv(n_envs=args.eval_envs, seed=args.seed + EVALUATION_SEED_OFFSET,
                               **stage_task(TASK_GAP_WIDTH_M if args.floor == "gaps" else None))
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

    stage_timesteps = args.timesteps // len(widths)
    history, started = [], time.monotonic()
    for stage, width in enumerate(widths):
        if stage > 0 and width != widths[stage - 1]:
            raw.set_task(**stage_task(width))
            model._last_obs = env.reset()
            model._last_episode_starts = np.ones((args.n_envs,), dtype=bool)
        model.learn(total_timesteps=stage_timesteps, reset_num_timesteps=False)
        result = evaluate(model, held_out, normalize)
        result.update(stage=stage, gap_width_m=width, timesteps=model.num_timesteps,
                      seconds=round(time.monotonic() - started, 1))
        history.append(result)
        print(json.dumps(result), flush=True)

    os.makedirs(args.out, exist_ok=True)
    summary = {
        "floor": args.floor,
        "height_scan": args.height_scan,
        "seed": args.seed,
        "n_envs": args.n_envs,
        "curriculum": args.curriculum or None,
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
    if args.smoke:
        assert len(history) == len(widths)
        print("smoke ok")


if __name__ == "__main__":
    main()
