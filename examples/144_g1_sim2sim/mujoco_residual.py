#!/usr/bin/env python3
"""Train example 144's push-recovery residual in MuJoCo instead of RNE.

Repeats `--train-residual` exactly: the residual model (linear or MLP), the
push scenarios, the evolution-strategies settings and the seed come from RNE's
training report and the starting parameters are drawn as RNE draws them. The
fitness is the same (1 per survived push plus up to 0.5 for how upright the
pelvis stayed, half the share of the window stood through for a fall, minus
the mean squared target offset times the penalty, the mean squared offset
while walking before the push times its own penalty and, for each candidate,
the squared change in undisturbed walking speed from the plain policy's times
the speed penalty) and so is the update
(antithetic pairs, centred ranks, Adam, SplitMix64 normal deviates). Only the
simulator differs, so the two runs compare training cost and, evaluated with
mujoco_robustness.py and `--robustness --residual`, transfer both ways.

Usage: mujoco_residual.py UNITREE_RL_GYM RNE_TRAINING_REPORT.json OUTPUT.json
       [--processes 4] [--generations N]
"""
import argparse
import json
import math
import multiprocessing
import time
from pathlib import Path

import mujoco
import numpy as np

from mujoco_robustness import walk

DIRECTIONS = {"forward": (1.0, 0.0), "backward": (-1.0, 0.0), "left": (0.0, 1.0),
              "right": (0.0, -1.0)}
PHASE_STEPS, PUSH_STEPS = 5, 5


class Normal:
    """SplitMix64 normal deviates (Box-Muller), as residual.rs draws them."""

    def __init__(self, seed):
        self.state = seed

    def uniform(self):
        self.state = (self.state + 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
        z = self.state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & 0xFFFFFFFFFFFFFFFF
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & 0xFFFFFFFFFFFFFFFF
        z ^= z >> 31
        return ((z >> 11) + 0.5) / float(1 << 53)

    def sample(self):
        u, v = self.uniform(), self.uniform()
        return math.sqrt(-2.0 * math.log(u)) * math.cos(2.0 * math.pi * v)


def initial_parameters(spec, normal):
    """The starting parameter vector, drawn as residual.rs draws it."""
    if spec.get("model") == "mlp":
        inputs, hidden = len(spec["observation_indices"]), spec["hidden"]
        joints = len(spec["w2"])
        scale = 1.0 / math.sqrt(inputs)
        w1 = [scale * normal.sample() for _ in range(hidden * inputs)]
        return np.array(w1 + [0.0] * (hidden + joints * hidden))
    return np.zeros(len(spec["weights_rad"]) * len(spec["features"]))


def residual_spec(spec, theta):
    """`spec` (an RNE residual) with its parameters replaced by `theta`."""
    if spec.get("model") == "mlp":
        inputs, hidden = len(spec["observation_indices"]), spec["hidden"]
        w1, rest = theta[:hidden * inputs], theta[hidden * inputs:]
        return {**spec, "w1": w1.reshape(hidden, inputs).tolist(), "b1": rest[:hidden].tolist(),
                "w2": rest[hidden:].reshape(-1, hidden).tolist()}
    return {**spec, "weights_rad": theta.reshape(len(spec["weights_rad"]), -1).tolist()}


def scenario(job):
    upstream, robot, training, residual, case, self_collision = job
    walk_steps = int(round(training["walk_before_push_s"] / 0.02))
    recovery_steps = int(round(training["recovery_s"] / 0.02))
    start = walk_steps + case["phase"] * PHASE_STEPS
    end = start + PUSH_STEPS + recovery_steps
    x, y = DIRECTIONS[case["direction"]]
    force = np.array([x * case["force_n"], y * case["force_n"], 0.0])
    squares, tilts = [], []
    protocol = {"command": [0.5, 0.0, 0.0]}
    fall, _ = walk(upstream, robot, protocol, end, push=(start, end - recovery_steps, force),
                   fall_m=0.55, self_collision=self_collision, residual=residual,
                   offset_squares=squares, tilts=tilts)
    walk_square = float(np.mean(squares[:start]))
    if fall is None:
        upright = 0.5 * (1.0 - min(max(tilts[start:]) / training["tilt_reference_rad"], 1.0))
        return 1.0 + upright, float(np.mean(squares)), walk_square, end
    taken = int(round(fall / 0.02))
    stood = max(taken - start, 0) / (end - start)
    return 0.5 * stood, float(np.mean(squares)), walk_square, taken


def speed_check(upstream, robot, training, residual, self_collision):
    """Mean forward speed over the second half of an undisturbed walk (zero
    if it falls) and the policy steps taken, as residual.rs's walk_speed."""
    steps = int(round(training["speed_check_s"] / 0.02))
    fall, records = walk(upstream, robot, {"command": [0.5, 0.0, 0.0]}, steps, fall_m=0.55,
                         self_collision=self_collision, residual=residual)
    if fall is not None:
        return 0.0, int(round(fall / 0.02))
    half = steps // 2
    x = [r["base_position_m"][0] for r in records]
    return (x[-1] - x[half - 1]) / ((steps - half) * 0.02), steps


def job(arguments):
    """One push scenario, or a speed check when the scenario is None."""
    upstream, robot, training, residual, case, self_collision = arguments
    if case is None:
        return speed_check(upstream, robot, training, residual, self_collision)
    return scenario(arguments)


def centred_ranks(values):
    order = sorted(range(len(values)), key=lambda i: (values[i], i))
    ranks = [0.0] * len(values)
    for rank, index in enumerate(order):
        ranks[index] = rank / max(len(values) - 1, 1) - 0.5
    return ranks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path)
    parser.add_argument("rne_training", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--processes", type=int, default=multiprocessing.cpu_count())
    parser.add_argument("--generations", type=int)
    arguments = parser.parse_args()
    rne = json.loads(arguments.rne_training.read_text())
    robot, training = rne["robot"], rne["training"]
    spec = rne["residual"]
    generations = training["generations"] if arguments.generations is None else arguments.generations
    pairs, sigma, rate = training["pairs"], training["sigma"], training["learning_rate"]
    penalty, scenarios = training["offset_penalty_per_rad2"], training["scenarios"]
    walk_penalty = training.get("walk_offset_penalty_per_rad2", 0.0)
    speed_penalty = training.get("speed_penalty_per_m2_s2", 0.0)
    cases = scenarios + ([None] if speed_penalty > 0 else [])
    self_collision = rne["self_collision"]
    normal = Normal(training["seed"])
    theta = initial_parameters(spec, normal)
    dimension = len(theta)
    m, v = np.zeros(dimension), np.zeros(dimension)
    upstream = str(arguments.upstream)
    log, simulated_steps, begin = [], 0, time.perf_counter()
    checkpoint_every, checkpoints = training.get("checkpoint_every", 0), []
    reference_speed = None
    if speed_penalty > 0:
        # The plain policy's undisturbed speed in MuJoCo, which candidates keep.
        reference_speed, steps = speed_check(upstream, robot, training,
                                             residual_spec(spec, np.zeros(dimension)),
                                             self_collision)
        simulated_steps += steps
    with multiprocessing.get_context("fork").Pool(arguments.processes) as pool:
        for generation in range(generations + 1):
            noise = [np.array([normal.sample() for _ in range(dimension)]) for _ in range(pairs)]
            candidates = [theta.copy()]
            if generation < generations:
                for epsilon in noise:
                    candidates += [theta + sigma * epsilon, theta - sigma * epsilon]
            jobs = [(upstream, robot, training, residual_spec(spec, c), s, self_collision)
                    for c in candidates for s in cases]
            results = pool.map(job, jobs)
            fitness = [0.0] * len(candidates)
            survived = [0.0] * len(candidates)
            walk_square = [0.0] * len(candidates)
            speed = [None] * len(candidates)
            for index, result in enumerate(results):
                candidate, case = divmod(index, len(cases))
                if cases[case] is None:
                    speed[candidate], taken = result
                    fitness[candidate] -= speed_penalty * (speed[candidate] - reference_speed) ** 2
                else:
                    score, square, walk_sq, taken = result
                    fitness[candidate] += (score - penalty * square - walk_penalty * walk_sq) \
                        / len(scenarios)
                    survived[candidate] += (score >= 1.0) / len(scenarios)
                    walk_square[candidate] += walk_sq / len(scenarios)
                simulated_steps += taken
            entry = {"generation": generation, "fitness": fitness[0],
                     "survived_share": survived[0],
                     "population_mean_fitness": sum(fitness) / len(fitness),
                     "walk_rms_offset_rad": math.sqrt(walk_square[0]),
                     "walk_speed_m_s": speed[0],
                     "wall_s": time.perf_counter() - begin,
                     "simulated_s": simulated_steps * 0.02}
            print(json.dumps(entry), flush=True)
            log.append(entry)
            if checkpoint_every and (generation % checkpoint_every == 0 or generation == generations):
                checkpoints.append({"generation": generation, "wall_s": entry["wall_s"],
                                    "simulated_s": entry["simulated_s"],
                                    "residual": residual_spec(spec, theta)})
            if generation == generations:
                break
            ranks = centred_ranks(fitness[1:])
            gradient = np.zeros(dimension)
            for pair, epsilon in enumerate(noise):
                gradient += (ranks[2 * pair] - ranks[2 * pair + 1]) * epsilon / (pairs * sigma)
            t = generation + 1
            m = 0.9 * m + 0.1 * gradient
            v = 0.999 * v + 0.001 * gradient * gradient
            theta = theta + rate * (m / (1 - 0.9 ** t)) / (np.sqrt(v / (1 - 0.999 ** t)) + 1e-8)
    report = {**rne, "engine": f"mujoco {mujoco.__version__}",
              "residual": residual_spec(spec, theta),
              "training": {**rne["training"], "generations": generations,
                           "processes": arguments.processes, "log": log,
                           "checkpoints": checkpoints,
                           "reference_speed_m_s": reference_speed}}
    report["training"].pop("threads", None)
    arguments.output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"wrote {arguments.output}")


if __name__ == "__main__":
    main()
