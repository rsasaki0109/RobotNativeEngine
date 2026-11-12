"""Train example 147's Go2 walk with Stable-Baselines3 PPO.

The environments are `rne_py_locomotion.Go2LearningBatch`, stepped in parallel in one
Rust call per control period. `--smoke` trains a few thousand steps for CI.

    .venv/bin/python examples/148_go2_learning_python/train_ppo.py --smoke
    .venv/bin/python examples/148_go2_learning_python/train_ppo.py \
        --floor gaps --height-scan --timesteps 5000000 --n-envs 16
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from stable_baselines3 import PPO
except ImportError:
    sys.exit(
        "stable-baselines3 is required. Install with: "
        "pip install 'stable-baselines3==2.6.0' --extra-index-url "
        "https://download.pytorch.org/whl/cpu"
    )

from go2_vec_env import NativeGo2VecEnv
from run import task_arguments


def evaluate(model, env, periods):
    """Mean return of the deterministic policy over the first episode of
    every environment."""
    observations = env.reset()
    totals, done = [0.0] * env.num_envs, [False] * env.num_envs
    for _ in range(periods):
        actions, _ = model.predict(observations, deterministic=True)
        observations, rewards, dones, _ = env.step(actions)
        for index in range(env.num_envs):
            if not done[index]:
                totals[index] += float(rewards[index])
                done[index] = bool(dones[index])
        if all(done):
            break
    return sum(totals) / len(totals)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--floor", choices=["flat", "terrain", "gaps"], default="flat")
    parser.add_argument("--terrain-seed", type=int, default=7)
    parser.add_argument("--height-scan", action="store_true")
    parser.add_argument("--n-envs", type=int, default=16)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--timesteps", type=int, default=2_000_000)
    parser.add_argument("--out", default="target/go2_learning_ppo")
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    if args.smoke:
        args.n_envs, args.timesteps = 2, 1024
    env = NativeGo2VecEnv(n_envs=args.n_envs, seed=args.seed, **task_arguments(args))
    model = PPO("MlpPolicy", env, n_steps=256 if args.smoke else 1024, batch_size=256,
                seed=args.seed, verbose=0)
    model.learn(total_timesteps=args.timesteps)
    score = evaluate(model, env, env._batch.episode_steps)
    os.makedirs(args.out, exist_ok=True)
    if not args.smoke:
        model.save(os.path.join(args.out, "ppo_go2"))
    print(json.dumps({"floor": args.floor, "height_scan": args.height_scan,
                      "timesteps": args.timesteps, "mean_return": score}))
    if args.smoke:
        print("smoke ok")


if __name__ == "__main__":
    main()
