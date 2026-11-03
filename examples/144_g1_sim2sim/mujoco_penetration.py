#!/usr/bin/env python3
"""MuJoCo side of example 144's `--penetration`.

Walks the policy at 0.5 m/s for 6 s on unitree_rl_gym's MuJoCo scene with the
loop of mujoco_robustness.py and, for each policy step after the first 2 s,
takes the deepest penetration of any floor contact (`-mjContact.dist`, zero
if none penetrates). Prints the median and 95th percentile in millimetres,
picked as RNE picks them.

Usage: mujoco_penetration.py UNITREE_RL_GYM [--robot g1]
"""
import argparse
from pathlib import Path

from mujoco_reference import ROBOTS
from mujoco_robustness import walk

STEPS, SKIPPED = 300, 100


def quantile(ordered, q):
    """The sample at rank round((n - 1) q), as walk.rs's foot_penetration_mm."""
    return ordered[int((len(ordered) - 1) * q + 0.5)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("upstream", type=Path)
    parser.add_argument("--robot", choices=sorted(ROBOTS), default="g1")
    arguments = parser.parse_args()
    depths = []
    walk(str(arguments.upstream), arguments.robot, {"command": [0.5, 0.0, 0.0]}, STEPS,
         penetrations=depths)
    ordered = sorted(depths[SKIPPED:])
    print(f"{arguments.robot} foot penetration while walking: median "
          f"{1000 * quantile(ordered, 0.5):.2f} mm, 95th percentile "
          f"{1000 * quantile(ordered, 0.95):.2f} mm")


if __name__ == "__main__":
    main()
