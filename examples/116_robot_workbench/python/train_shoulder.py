"""Parallel cross-entropy policy search for two shoulder PD gains (no extra deps)."""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import numpy as np

from rne_gym import ShoulderVectorEnv

SCENARIOS = Path(__file__).parents[1] / "scenarios"
INITIAL = np.log([0.3, 0.02])
LOW = np.log([0.05, 0.002])
HIGH = np.log([10.0, 0.5])


def policy_actions(observations, names, caps, log_gains, hold):
    """Explicit policy: learned shoulder gains, fixed other-arm PD, coasting wheels."""
    actions = np.zeros((len(observations), len(names)))
    for j, name in enumerate(names):
        if name not in hold:
            continue
        g = hold[name]
        kp, kd = (np.exp(log_gains).T if name == "shoulder_pan"
                  else (g["stiffness_nm_per_rad"], g["damping_nm_s_per_rad"]))
        target = observations[:, -3] if name == "shoulder_pan" else 0.0
        torque = kp * (target - observations[:, j]) - kd * observations[:, j + len(names)]
        actions[:, j] = np.clip(torque, -g["effort_limit_nm"], g["effort_limit_nm"]) / caps[:, j]
    return actions


def evaluate(env, candidates, seeds, hold, *, action_fn=policy_actions, reset_fn=None):
    """Each candidate sees the same target seeds; score only its first episode."""
    records = [[] for _ in candidates]
    for seed in seeds:
        obs, _ = env.reset(seed=[seed] * env.num_envs)
        if reset_fn is not None:
            reset_fn(np.ones(env.num_envs, dtype=bool))
        active = np.ones(env.num_envs, dtype=bool)
        returns = np.zeros(env.num_envs)
        lengths = np.zeros(env.num_envs, dtype=int)
        for _ in range(env.horizon):
            action = action_fn(obs, env.joint_names, env._caps, candidates, hold)
            obs, reward, terminated, truncated, info = env.step(action)
            returns[active] += reward[active]
            lengths[active] += 1
            done = terminated | truncated
            for i in np.flatnonzero(active & done):
                records[i].append({"seed": seed, "return": float(returns[i]),
                                   "steps": int(lengths[i]),
                                   "success": bool(info["is_success"][i]),
                                   "fallen": bool(info["fallen"][i]),
                                   "truncated": bool(truncated[i]),
                                   "final_error_rad": np.asarray(info["angle_error_rad"][i]).tolist(),
                                   "state_hash": str(info["state_hash"][i])})
            active &= ~done
            if not active.any():
                break
            if done.any():
                # Finished lanes remain physical peers in a synchronous batch but
                # subsequent episodes never contribute to candidate fitness.
                obs, _ = env.reset(options={"reset_mask": done})
                if reset_fn is not None:
                    reset_fn(done)
    return records


def score(records):
    return np.array([np.mean([r["return"] for r in rows]) for rows in records])


def summary(rows):
    return {"mean_return": float(np.mean([r["return"] for r in rows])),
            "mean_steps": float(np.mean([r["steps"] for r in rows])),
            "success_rate": float(np.mean([r["success"] for r in rows])),
            "mean_abs_final_error_rad": float(np.mean([np.abs(r["final_error_rad"]) for r in rows]))}


def search(env, hold, *, seed, generations, train_seeds, initial=INITIAL,
           low=LOW, high=HIGH, initial_std=1.2, action_fn=policy_actions, reset_fn=None, score_fn=score):
    rng = np.random.default_rng(seed)
    mean, std = initial.copy(), np.full(len(initial), initial_std)
    best = mean.copy()
    best_score = None
    history = []
    for generation in range(generations):
        candidates = np.clip(rng.normal(mean, std, (env.num_envs, len(initial))), low, high)
        candidates[0] = best  # Never discard the incumbent based on sampling alone.
        records = evaluate(env, candidates, train_seeds, hold, action_fn=action_fn, reset_fn=reset_fn)
        scores = score_fn(records)
        order = sorted(range(env.num_envs), key=lambda i: scores[i], reverse=True)
        if best_score is None or scores[order[0]] > best_score:
            best, best_score = candidates[order[0]].copy(), scores[order[0]]
        elite = candidates[order[:max(2, env.num_envs // 2)]]
        mean = elite.mean(axis=0)
        std = np.maximum(elite.std(axis=0), 0.15)
        row = {"generation": generation, "gains": np.exp(candidates).tolist(),
               "scores": np.asarray(scores).tolist(), "episodes": records,
               "best_score": best_score}
        history.append(row)
        print(json.dumps({"generation": generation, "best_training_score": best_score,
                          "best_gains": np.exp(best).tolist()}), flush=True)
    return best, history


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("output", type=Path, help="new result JSON; existing files are not overwritten")
    parser.add_argument("--generations", type=int, default=4)
    parser.add_argument("--population", type=int, default=6)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if not 2 <= args.population <= 16 or args.generations < 1 or args.seed < 0:
        parser.error("population must be 2..16, generations positive, seed nonnegative")
    if args.output.exists():
        parser.error("output already exists")
    # Same physical world in every lane: candidate comparisons must be paired.
    world = json.loads((SCENARIOS / "vector_arena.json").read_text())["environments"][0]
    hold = json.loads((SCENARIOS / "wheel_arm_hold.json").read_text())["arm_hold"]["alice"]
    config = {"workers": min(args.population, 8),
              "environments": [copy.deepcopy(world) for _ in range(args.population)]}
    train_seeds, eval_seeds = [11, 29], [101, 202, 303, 404]
    with ShoulderVectorEnv(args.executable.resolve(), config, horizon=300, physics_steps=1) as env:
        best, history = search(env, hold, seed=args.seed, generations=args.generations, train_seeds=train_seeds)
    # Freeze best before touching held-out targets. Evaluate initial and final paired.
    config["environments"] = config["environments"][:2]
    config["workers"] = 2
    with ShoulderVectorEnv(args.executable.resolve(), config, horizon=300, physics_steps=1) as env:
        evaluation = evaluate(env, np.stack([INITIAL, best]), eval_seeds, hold)
    report = {"schema_version": 1, "algorithm": "cross_entropy_pd_gain_search",
              "seed": args.seed, "train_seeds": train_seeds, "evaluation_seeds": eval_seeds,
              "world": world, "horizon": 300, "physics_steps": 1,
              "fixed_arm_hold": hold, "initial_gains": np.exp(INITIAL).tolist(),
              "policy_log_gains": {"initial": INITIAL.tolist(), "learned": best.tolist()},
              "learned_gains": np.exp(best).tolist(), "training": history,
              "evaluation": {"initial": evaluation[0], "learned": evaluation[1]},
              "summary": {"initial": summary(evaluation[0]), "learned": summary(evaluation[1])}}
    with args.output.open("x", encoding="utf-8") as out:
        json.dump(report, out, indent=2, allow_nan=False)
        out.write("\n")
    print(json.dumps(report["summary"]), flush=True)


if __name__ == "__main__":
    main()
