"""Step example 147's Go2 walking task from Python, with no RL library.

`rne_py_locomotion.Go2LearningBatch` is `rne_locomotion::go2_learning` as a native
parallel batch. This script walks a few environments with a fixed open-loop
trot-like action, checks the contract the batch keeps, and prints what it
saw. `--smoke` keeps it short for CI.

    .venv/bin/maturin develop -m crates/rne_py_locomotion/Cargo.toml --release
    .venv/bin/python examples/148_go2_learning_python/run.py --smoke
"""

import argparse
import json
import math
import sys

try:
    import rne_py_locomotion
except ImportError:
    sys.exit(
        "rne_py_locomotion is not installed. Build it with: "
        ".venv/bin/maturin develop -m crates/rne_py_locomotion/Cargo.toml --release"
    )


def task_arguments(args):
    if args.floor == "flat":
        return {}
    task = {"terrain_seed": args.terrain_seed, "random_starts": True,
            "height_scan": args.height_scan, "terrain_size": 48.0}
    if args.floor == "gaps":
        task.update(terrain_amplitude=0.0, terrain_cell=0.05, gap_depth=0.3)
    return task


def trot(period, joint):
    """An open-loop action: diagonal legs swing in antiphase."""
    leg = joint // 3
    phase = 0.0 if leg in (0, 3) else math.pi
    return 0.3 * math.sin(2.0 * math.pi * period * 0.02 / 0.5 + phase)


def walk(args):
    batch = rne_py_locomotion.Go2LearningBatch(args.n_envs, args.seed, 0, **task_arguments(args))
    spec = json.loads(batch.task_spec_json())
    width = batch.observation_dim
    first = batch.reset()
    assert len(first.observations) == args.n_envs
    assert all(len(observation) == width for observation in first.observations)
    returns, episode_return, ended = [], [0.0] * args.n_envs, 0
    for period in range(args.periods):
        actions = [[trot(period, joint) for joint in range(12)] for _ in range(args.n_envs)]
        step = batch.step(actions)
        for env in range(args.n_envs):
            episode_return[env] += step.rewards[env]
            done = step.terminated[env] or step.truncated[env]
            assert done == (step.terminal_observations[env] is not None)
            if done:
                ended += 1
                returns.append(episode_return[env])
                episode_return[env] = 0.0
    print(json.dumps({
        "task": spec["task_id"], "observation_dim": width, "envs": args.n_envs,
        "periods": args.periods, "episodes_ended": ended,
        "mean_return": sum(returns) / len(returns) if returns else None,
        "base_positions_m": batch.base_positions(),
    }))
    return batch


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--floor", choices=["flat", "terrain", "gaps"], default="flat")
    parser.add_argument("--terrain-seed", type=int, default=7)
    parser.add_argument("--height-scan", action="store_true")
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--periods", type=int, default=600)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    if args.smoke:
        # Past one episode's 250 periods, on the flat floor and on gaps with
        # the scan, so a reset and the scan both run.
        for floor, scan in (("flat", False), ("gaps", True)):
            args.floor, args.height_scan, args.n_envs, args.periods = floor, scan, 2, 260
            walk(args)
        # Same seed, same steps.
        args.floor, args.height_scan, args.periods = "flat", False, 20
        first = walk(args).base_positions()
        assert first == walk(args).base_positions(), "a batch must be deterministic"
        print("smoke ok")
        return
    walk(args)


if __name__ == "__main__":
    main()
