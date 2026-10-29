"""Collect seeded vector transitions; demonstrates explicit reset after done."""
import argparse
import json
from pathlib import Path

import numpy as np

from rne_gym import ShoulderVectorEnv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("--config", type=Path, default=Path(__file__).parents[1] / "scenarios/vector_arena.json")
    parser.add_argument("--steps", type=int, default=60)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be positive")
    config = json.loads(args.config.read_text())
    with ShoulderVectorEnv(args.executable.resolve(), config, horizon=30) as env:
        env.reset(seed=args.seed)
        rng = np.random.default_rng(args.seed)
        total = np.zeros(env.num_envs)
        episodes = np.zeros(env.num_envs, dtype=int)
        successes = np.zeros(env.num_envs, dtype=int)
        for _ in range(args.steps):
            # Small exploratory torques; this is a protocol example, not a trained policy.
            action = rng.uniform(-0.01, 0.01, env.action_space.shape)
            obs, reward, terminated, truncated, info = env.step(action)
            assert env.observation_space.contains(obs)
            total += reward
            successes += info["is_success"]
            done = terminated | truncated
            if done.any():
                episodes += done
                env.reset(options={"reset_mask": done})
        print(json.dumps({"transitions": args.steps * env.num_envs,
                          "reward_sum": total.tolist(), "episodes": episodes.tolist(),
                          "successes": successes.tolist()}))


if __name__ == "__main__":
    main()
