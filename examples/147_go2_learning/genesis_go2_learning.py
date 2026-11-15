#!/usr/bin/env python3
"""Example 147's Go2 in Genesis: a third simulator to walk the same policies in.

RNE and MuJoCo agree on how example 148's gap policies fall (see example
148's README); Genesis, a GPU-first simulator with its own rigid solver,
is a third opinion when two disagree. The episode, observation, height
scan, policies, starts and report are `mujoco_go2_learning.py`'s, so the
walks compare start by start with `walk_policy.py --against`; only the
plant differs:

- the URDF RNE loads, without visuals, its fixed links merged, as MuJoCo's;
- a free base, the terrain as a Genesis height field (cropped to the
  region the episodes walk), friction 1 on both;
- the PD (kp 20, kd 0.5) in Genesis's position control, within the URDF's
  effort limits, and with `--velocity-limits` the torque-speed curve
  (`motor_torque_range`) set before every physics step, as MuJoCo's script;
- no rotor inertia: Genesis gives each joint 0.1 kg·m² of armature by
  default, twenty times a calf's own inertia, which RNE and MuJoCo lack;
- no self-collision, as in RNE and MuJoCo.

Usage: genesis_go2_learning.py stand [--terrain TERRAIN.json]
       genesis_go2_learning.py evaluate POLICY.json --terrain TERRAIN.json --starts N
       genesis_go2_learning.py check-terrain TERRAIN.json
"""
import argparse
import json
import math
import tempfile
from pathlib import Path

import numpy as np

import genesis as gs
import mujoco_go2_learning as mj

EFFORT_LIMIT_NM = np.tile([23.7, 23.7, 45.43], 4)
# The region the evaluation episodes walk: starts lie in x [-8, 2] and
# y [-8, 8], and a 5 s walk covers under 20 m.
CROP_X_M, CROP_Y_M = (-10.0, 24.0), (-10.0, 10.0)


def numpy(value):
    """A Genesis tensor (or array) as a float numpy array."""
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=float)


def rotation_matrix(quaternion):
    """The rotation of a w x y z quaternion."""
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def cropped_field(terrain):
    """The terrain grid over `CROP_X_M` x `CROP_Y_M`, x along the first
    axis as Genesis's height fields lie, and the world position of its
    first sample."""
    grid, size = terrain["grid"], terrain["size_m"]
    rows, columns = grid.shape
    cell = size / (columns - 1)
    column = lambda x: int(round((x + 0.5 * size) / cell))
    row = lambda y: int(round((y + 0.5 * size) / cell))
    c0, c1 = max(column(CROP_X_M[0]), 0), min(column(CROP_X_M[1]), columns - 1)
    r0, r1 = max(row(CROP_Y_M[0]), 0), min(row(CROP_Y_M[1]), rows - 1)
    field = grid[r0:r1 + 1, c0:c1 + 1].T
    return field, cell, (-0.5 * size + c0 * cell, -0.5 * size + r0 * cell)


def joint_dof(robot, name):
    joint = robot.get_joint(name)
    for attribute in ("dofs_idx_local", "dof_idx_local", "dofs_idx", "dof_idx", "dof_start"):
        value = getattr(joint, attribute, None)
        if value is not None:
            return int(np.ravel(value)[0])
    raise SystemExit(f"cannot find the dof of {name}")


class GenesisGo2(mj.Go2):
    """`mujoco_go2_learning.Go2`'s episode on a Genesis plant."""

    def __init__(self, plant, task=None):  # pylint: disable=super-init-not-called
        self.task = task or {}
        self.plant = plant
        self.terrain = mj.load_terrain(plant.get("terrain"))
        self.substeps = plant.get("substeps", 5)
        self.velocity_limits = bool(plant.get("velocity_limits", False))
        self.trace = None
        side = mj.FLAT_FLOOR_SIZE_M if self.terrain is None else self.terrain["size_m"]
        self.arena_half = 0.5 * side - mj.ARENA_EDGE_MARGIN_M
        friction = plant.get("floor_friction", 1.0)
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=mj.DT_S / self.substeps, substeps=1),
            rigid_options=gs.options.RigidOptions(enable_self_collision=False),
            show_viewer=False)
        material = gs.materials.Rigid(friction=friction)
        if self.terrain is None:
            self.scene.add_entity(gs.morphs.Plane(), material=material)
        else:
            field, cell, (x0, y0) = cropped_field(self.terrain)
            self.floor = self.scene.add_entity(
                gs.morphs.Terrain(height_field=field, horizontal_scale=cell, vertical_scale=1.0,
                                  pos=(x0, y0, 0.0)),
                material=material)
        urdf = Path(tempfile.mkdtemp()) / "go2.urdf"
        urdf.write_text(mj.urdf_without_visuals())
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(file=str(urdf), pos=(0.0, 0.0, mj.SPAWN_HEIGHT_M),
                           merge_fixed_links=True, default_armature=None),
            material=material)
        self.scene.build()
        self.dofs = [joint_dof(self.robot, name) for name in mj.JOINTS]
        self.robot.set_dofs_kp(np.full(12, mj.KP_NM_PER_RAD), self.dofs)
        self.robot.set_dofs_kv(np.full(12, mj.KD_NM_S_PER_RAD), self.dofs)
        self.robot.set_dofs_force_range(-EFFORT_LIMIT_NM, EFFORT_LIMIT_NM, self.dofs)

    def mass_kg(self):
        try:
            return float(numpy(self.robot.get_mass()).sum())
        except Exception:  # noqa: BLE001 - diagnostic only
            return None

    def settle(self, start=(0.0, 0.0)):
        x, y = start
        ground = 0.0 if (x, y) == (0.0, 0.0) else mj.ground_height(self.terrain, x, y)
        self.robot.set_pos(np.array([x, y, mj.SPAWN_HEIGHT_M + ground]))
        self.robot.set_quat(np.array([1.0, 0.0, 0.0, 0.0]))
        self.robot.set_dofs_position(mj.DEFAULT_RAD, self.dofs)
        self.robot.zero_all_dofs_velocity()
        for _ in range(mj.SETTLE_STEPS):
            self.step(mj.DEFAULT_RAD)

    def step(self, targets_rad):
        if self.trace is not None:
            self.trace.append({**self.state(), "targets_rad": list(map(float, targets_rad))})
        self.robot.control_dofs_position(np.asarray(targets_rad, dtype=float), self.dofs)
        for _ in range(mj.DECIMATION * self.substeps):
            if self.velocity_limits:
                ratio = self.joint_velocities() / mj.VELOCITY_LIMIT_RAD_S
                lowest = np.clip(EFFORT_LIMIT_NM * (-1.0 - ratio), -EFFORT_LIMIT_NM,
                                 EFFORT_LIMIT_NM)
                highest = np.clip(EFFORT_LIMIT_NM * (1.0 - ratio), -EFFORT_LIMIT_NM,
                                  EFFORT_LIMIT_NM)
                self.robot.set_dofs_force_range(lowest, highest, self.dofs)
            self.scene.step()

    def base_position(self):
        return numpy(self.robot.get_pos())

    def base_quaternion(self):
        return numpy(self.robot.get_quat())

    def body_rotation(self):
        return rotation_matrix(self.base_quaternion())

    def body_spin(self):
        return self.body_rotation().T @ numpy(self.robot.get_ang())

    def joint_positions(self):
        return numpy(self.robot.get_dofs_position(self.dofs))

    def joint_velocities(self):
        return numpy(self.robot.get_dofs_velocity(self.dofs))

    def height_scan(self):
        """The scan's readings from the terrain's heights below each point,
        as RNE reads them."""
        x0, y0, z0 = self.base_position()
        heading = self.heading()
        sin, cos = math.sin(heading), math.cos(heading)
        readings = []
        for dx in mj.HEIGHT_SCAN_X_M:
            for dy in mj.HEIGHT_SCAN_Y_M:
                hit = mj.ground_height(self.terrain, x0 + cos * dx - sin * dy,
                                       y0 + sin * dx + cos * dy)
                reading = (z0 - hit - mj.HEIGHT_SCAN_NOMINAL_M) * mj.HEIGHT_SCAN_GAIN_PER_M
                readings.append(min(max(reading, -1.0), 1.0))
        return readings

    def state(self):
        return {"position_m": self.base_position().tolist(),
                "quaternion_wxyz": self.base_quaternion().tolist(),
                "linear_velocity_m_s": numpy(self.robot.get_vel()).tolist(),
                "angular_velocity_rad_s": numpy(self.robot.get_ang()).tolist(),
                "joints_rad": self.joint_positions().tolist(),
                "joint_velocities_rad_s": self.joint_velocities().tolist()}


def stand_report(robot, start, seconds=1.0):
    robot.settle(start)
    for _ in range(round(seconds / mj.CONTROL_DT_S)):
        robot.step(mj.DEFAULT_RAD)
    rotation = robot.body_rotation()
    position = robot.base_position()
    return {"start_m": list(start), "base_m": position.tolist(),
            "above_ground_m": float(position[2] - mj.ground_height(robot.terrain, *position[:2])),
            "pitch_rad": float(math.asin(max(-1.0, min(1.0, rotation[2, 0])))),
            "joint_rad": robot.joint_positions().tolist()}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    stand = sub.add_parser("stand")
    stand.add_argument("--terrain", type=Path, default=None)
    check = sub.add_parser("check-terrain")
    check.add_argument("terrain", type=Path)
    evaluate = sub.add_parser("evaluate")
    evaluate.add_argument("policy", type=Path)
    evaluate.add_argument("--terrain", type=Path, default=None)
    evaluate.add_argument("--starts", type=int, default=None)
    evaluate.add_argument("--trace", type=Path, default=None)
    evaluate.add_argument("--trace-episode", type=int, default=0)
    for command in (stand, check, evaluate):
        command.add_argument("--substeps", type=int, default=5,
                             help="physics steps per 5 ms step of RNE's (default 5: 1 ms)")
        command.add_argument("--friction", type=float, default=1.0)
        command.add_argument("--precision", choices=["32", "64"], default="64")
    evaluate.add_argument("--velocity-limits", action="store_true")
    arguments = parser.parse_args()
    gs.init(backend=gs.cpu, precision=arguments.precision, logging_level="warning")
    engine = f"genesis {getattr(gs, '__version__', '?')}"
    terrain = getattr(arguments, "terrain", None)
    plant = {"floor_friction": arguments.friction, "substeps": arguments.substeps,
             "terrain": None if terrain is None else str(terrain)}

    if arguments.command == "stand":
        robot = GenesisGo2(plant)
        print(json.dumps({"engine": engine, "plant": plant, "mass_kg": robot.mass_kg(),
                          **stand_report(robot, (0.0, 0.0))}))
    elif arguments.command == "check-terrain":
        # Does the field lie as RNE's? Stand the robot with its hind feet
        # over the first gap past the origin and with all four on flat
        # floor: only the first stands lower or pitched.
        robot = GenesisGo2(plant)
        gaps = [gap for gap in mj.terrain_gaps(plant) if gap[0] > 0.5]
        start, end = gaps[0]
        over = (0.5 * (start + end) + 0.19, 0.0)
        flat = (start - 0.6, 0.0)
        print(json.dumps({"engine": engine, "gap_m": [start, end],
                          "hind_feet_over_gap": stand_report(robot, over),
                          "flat": stand_report(robot, flat)}))
    else:
        policy, task = mj.load_policy(arguments.policy, None)
        trained = json.loads(arguments.policy.read_text()).get("task", {})
        if arguments.velocity_limits or trained.get("velocity_limits"):
            plant["velocity_limits"] = True
        robot = GenesisGo2(plant, task)
        gaps = mj.terrain_gaps(plant)
        points = ([(0.0, 0.0)] if arguments.starts is None
                  else mj.evaluation_starts(arguments.starts, gaps))
        if arguments.trace is not None:
            robot.trace = []
            traced = points[arguments.trace_episode]
            outcome = robot.episode(policy, traced)
            arguments.trace.write_text(json.dumps({
                "engine": engine, "policy": str(arguments.policy), "task": task,
                "start_m": list(traced), "settle_periods": mj.SETTLE_STEPS,
                "outcome": mj.outcome_json(outcome), "periods": robot.trace,
                "last": robot.state()}) + "\n")
            robot.trace = None
        outcomes = [robot.episode(policy, point) for point in points]
        outcome = mj.outcome_json(mj.mean_outcome(outcomes))
        report = {"engine": engine, "plant": plant, "policy": str(arguments.policy),
                  "task": task, "mass_kg": robot.mass_kg()}
        if arguments.starts is not None:
            outcome["falls"] = sum(o["fell"] for o in outcomes)
            report["episodes"] = [{"start_m": list(point), "outcome": mj.outcome_json(o)}
                                  for point, o in zip(points, outcomes)]
        if gaps:
            crossed = [mj.gaps_crossed(gaps, o["start_x_m"], o["final_x_m"]) for o in outcomes]
            if arguments.starts is not None:
                for episode, o, count in zip(report["episodes"], outcomes, crossed):
                    episode.update(start_x_m=o["start_x_m"], final_x_m=o["final_x_m"],
                                   gaps_crossed=count)
            outcome["gaps_crossed"] = sum(crossed)
            outcome["episodes_crossing_a_gap"] = sum(1 for count in crossed if count > 0)
        report["outcome"] = outcome
        print(json.dumps(report))


if __name__ == "__main__":
    main()
