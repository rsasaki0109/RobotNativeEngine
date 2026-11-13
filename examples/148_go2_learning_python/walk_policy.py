"""Walk a policy exported by train_ppo.py in RNE from example 147's held-out
evaluation starts: the points `mujoco_go2_learning.py evaluate --starts`
walks from, so the two simulators' walks compare episode by episode.

It needs `rne_py_locomotion` and numpy only. The policy runs here as plain
numbers, as it does in MuJoCo's script. `--against NAME=FILE` adds a
comparison with a MuJoCo evaluation of the same policy and starts.

    .venv/bin/python examples/148_go2_learning_python/walk_policy.py policy.json \\
        --starts 64 --against stiff=mujoco-stiff.json > rne.json
"""

import argparse
import json
import sys

import numpy as np

try:
    import rne_py_locomotion
except ImportError:
    sys.exit(
        "rne_py_locomotion is not installed. Build it with: "
        ".venv/bin/maturin develop -m crates/rne_py_locomotion/Cargo.toml --release"
    )

# As GAP_CROSSED_PAST_M in crates/rne_locomotion/src/go2_learning.rs.
GAP_CROSSED_PAST_M = 0.25
# A foot whose center is this far below the floor is in a gap.
IN_GAP_BELOW_M = 0.02


class Policy:
    """train_ppo.py's `policy_json`: normalized observations, ELU layers, a
    linear last layer clipped to [-1, 1]."""

    def __init__(self, value):
        if value.get("model") != "sb3_mlp" or value.get("activation") != "elu":
            sys.exit("expected a policy exported by train_ppo.py")
        self.layers = [(np.asarray(layer["w"]), np.asarray(layer["b"]))
                       for layer in value["layers"]]
        self.mean, self.var = np.asarray(value["obs_mean"]), np.asarray(value["obs_var"])
        self.clip_obs, self.epsilon = value.get("clip_obs"), value.get("epsilon", 1e-8)

    def act(self, observations):
        """Actions for a batch of observations, one per row."""
        x = (observations - self.mean) / np.sqrt(self.var + self.epsilon)
        if self.clip_obs is not None:
            x = np.clip(x, -self.clip_obs, self.clip_obs)
        for w, b in self.layers[:-1]:
            x = x @ w.T + b
            x = np.where(x > 0.0, x, np.exp(np.minimum(x, 0.0)) - 1.0)
        w, b = self.layers[-1]
        return np.clip(x @ w.T + b, -1.0, 1.0)


def task_arguments(task):
    """The batch arguments of train_ppo.py's `--floor` (run.py's
    `task_arguments`)."""
    if task["floor"] == "flat":
        return {}
    arguments = {"terrain_seed": 7, "random_starts": True,
                 "height_scan": task["height_scan"], "terrain_size": 48.0}
    if task["floor"] == "gaps":
        arguments.update(terrain_amplitude=0.0, terrain_cell=0.05, gap_depth=0.3)
    return arguments


def walk(policy, task, count):
    """One episode from each of the first `count` evaluation starts."""
    batch = rne_py_locomotion.Go2LearningBatch(count, 1, 0, **task_arguments(task))
    starts = batch.evaluation_starts(count)
    batch.set_starts(starts)
    gaps = batch.gap_intervals()
    observations = np.asarray(batch.reset().observations)
    start_x = np.asarray(batch.base_positions())[:, 0]
    final_x, feet = start_x.copy(), np.asarray(batch.foot_positions())
    totals, fell, done = np.zeros(count), np.zeros(count, bool), np.zeros(count, bool)
    for _ in range(batch.episode_steps):
        actions = policy.act(observations)
        before_x, before_feet = np.asarray(batch.base_positions())[:, 0], \
            np.asarray(batch.foot_positions())
        step = batch.step(actions.tolist())
        after_x, after_feet = np.asarray(batch.base_positions())[:, 0], \
            np.asarray(batch.foot_positions())
        observations = np.asarray(step.observations)
        for index in range(count):
            if done[index]:
                continue
            totals[index] += step.rewards[index]
            if step.terminated[index] or step.truncated[index]:
                # The batch has reset this environment already: its last
                # state is the one before the final period.
                done[index], fell[index] = True, step.terminated[index]
                final_x[index], feet[index] = before_x[index], before_feet[index]
            else:
                final_x[index], feet[index] = after_x[index], after_feet[index]
        if done.all():
            break
    episodes = []
    for index in range(count):
        crossed = sum(1 for begin, end in gaps
                      if begin > start_x[index] and final_x[index] > end + GAP_CROSSED_PAST_M)
        in_gap = feet[index][:, 2] < -IN_GAP_BELOW_M
        episodes.append({
            "start_m": list(starts[index]), "start_x_m": float(start_x[index]),
            "final_x_m": float(final_x[index]), "gaps_crossed": crossed,
            "fitness": float(totals[index]), "fell": bool(fell[index]),
            "front_foot_in_gap": bool(in_gap[:2].any()),
            "rear_foot_in_gap": bool(in_gap[2:].any()),
        })
    return episodes


def trace(policy, task, index, path):
    """Walks the episode from evaluation start `index` alone and writes
    each control period: the base, the four foot centers (front-left,
    front-right, rear-left, rear-right) and each link's ground contact
    over the period (normal impulse in N·s, physics steps in contact)."""
    batch = rne_py_locomotion.Go2LearningBatch(1, 1, 1, **task_arguments(task))
    start = batch.evaluation_starts(index + 1)[index]
    batch.set_starts([start])
    observations = np.asarray(batch.reset().observations)
    load = {name: (impulse, steps) for name, impulse, steps in batch.contact_load(0)}
    periods, fell = [], False
    for _ in range(batch.episode_steps):
        actions = policy.act(observations)
        period = {"position_m": batch.base_positions()[0],
                  "feet_m": batch.foot_positions()[0], "action": actions[0].tolist()}
        step = batch.step(actions.tolist())
        observations = np.asarray(step.observations)
        if step.terminated[0] or step.truncated[0]:
            # The batch has reset the environment: the last period's
            # contacts are lost with it.
            fell = bool(step.terminated[0])
            periods.append(period)
            break
        after = {name: (impulse, steps) for name, impulse, steps in batch.contact_load(0)}
        period["contacts"] = {name: {"impulse_n_s": after[name][0] - load[name][0],
                                     "steps": after[name][1] - load[name][1]}
                              for name in after if after[name][1] > load[name][1]}
        load = after
        periods.append(period)
    with open(path, "w") as file:
        json.dump({"engine": "rne", "start_m": list(start), "episode": index, "fell": fell,
                   "gaps_m": batch.gap_intervals(), "periods": periods}, file)


def summary(episodes):
    return {
        "fitness": sum(e["fitness"] for e in episodes) / len(episodes),
        "forward_m": sum(e["final_x_m"] - e["start_x_m"] for e in episodes) / len(episodes),
        "falls": sum(e["fell"] for e in episodes),
        "gaps_crossed": sum(e["gaps_crossed"] for e in episodes),
        "episodes_crossing_a_gap": sum(1 for e in episodes if e["gaps_crossed"] > 0),
        "ended_front_foot_in_gap": sum(e["front_foot_in_gap"] for e in episodes),
    }


def compare(episodes, path):
    """Episode by episode against a `mujoco_go2_learning.py evaluate
    --starts` report of the same policy."""
    theirs = json.load(open(path))["episodes"]
    if len(theirs) != len(episodes):
        sys.exit(f"{path} has {len(theirs)} episodes, not {len(episodes)}")
    pairs = []
    for ours, other in zip(episodes, theirs):
        if max(abs(a - b) for a, b in zip(ours["start_m"], other["start_m"])) > 1e-6:
            sys.exit(f"{path} walks from other starts")
        pairs.append({
            "start_m": ours["start_m"],
            "rne": {"start_x_m": ours["start_x_m"], "final_x_m": ours["final_x_m"],
                    "gaps_crossed": ours["gaps_crossed"], "fell": ours["fell"],
                    "front_foot_in_gap": ours["front_foot_in_gap"]},
            "mujoco": {"start_x_m": other["start_x_m"], "final_x_m": other["final_x_m"],
                       "gaps_crossed": other["gaps_crossed"],
                       "fell": bool(other["outcome"]["fell"])},
        })
    more = sum(1 for p in pairs if p["rne"]["gaps_crossed"] > p["mujoco"]["gaps_crossed"])
    fewer = sum(1 for p in pairs if p["rne"]["gaps_crossed"] < p["mujoco"]["gaps_crossed"])
    # Where RNE crossed fewer gaps: how its episode ended.
    behind = [p["rne"] for p in pairs if p["rne"]["gaps_crossed"] < p["mujoco"]["gaps_crossed"]]
    return {
        "rne_gaps_crossed": sum(p["rne"]["gaps_crossed"] for p in pairs),
        "mujoco_gaps_crossed": sum(p["mujoco"]["gaps_crossed"] for p in pairs),
        "episodes_rne_crossed_more": more,
        "episodes_rne_crossed_fewer": fewer,
        "episodes_equal": len(pairs) - more - fewer,
        "where_rne_crossed_fewer": {
            "rne_fell": sum(e["fell"] for e in behind),
            "rne_front_foot_in_gap": sum(e["front_foot_in_gap"] and not e["fell"]
                                         for e in behind),
            "rne_walking": sum(not e["fell"] and not e["front_foot_in_gap"] for e in behind),
        },
        "mean_start_x_shift_m": sum(p["rne"]["start_x_m"] - p["mujoco"]["start_x_m"]
                                    for p in pairs) / len(pairs),
        "episodes": pairs,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("policy", help="policy.json written by train_ppo.py")
    parser.add_argument("--starts", type=int, default=64)
    parser.add_argument("--against", action="append", default=[], metavar="NAME=FILE",
                        help="a MuJoCo evaluation of the same policy and starts")
    parser.add_argument("--trace", nargs=2, metavar=("EPISODE", "FILE"), default=None,
                        help="write the trace of one episode of --starts")
    args = parser.parse_args()
    exported = json.load(open(args.policy))
    if args.trace is not None:
        trace(Policy(exported["policy"]), exported["task"], int(args.trace[0]), args.trace[1])
    episodes = walk(Policy(exported["policy"]), exported["task"], args.starts)
    report = {"task": exported["task"], "outcome": summary(episodes), "episodes": episodes,
              "against": {}}
    for entry in args.against:
        name, _, path = entry.partition("=")
        report["against"][name] = compare(episodes, path)
    print(json.dumps(report))


if __name__ == "__main__":
    main()
