#!/usr/bin/env python3
"""Example 147's Go2 learning to walk, in MuJoCo.

The model is built from the URDF RNE loads
(assets/robots/go2_description/go2_description.rne.urdf). MuJoCo's URDF
importer reads it without its visuals and materials, which leaves the boxes,
cylinders and spheres RNE collides with, and merges its fixed links. On top of
it go:
- a free joint on the base and a floor of the same friction;
- a position actuator per joint: kp (target - q) - kv q' clamped to the URDF
  effort limit at every physics step, the law RNE's PdJointWalker applies;
- collisions between the robot and the floor only, as RNE's scene has no
  self-collision;
- no joint damping, friction or armature, as in RNE;
- the elliptic cone and impratio 100, as MuJoCo Menagerie's Go2 uses
  (`--cone pyramidal --impratio 1` for MuJoCo's defaults).

Training repeats example 147's `--train` exactly:
- the episode (start in the default pose, stand 1 s, walk 5 s), the
  observation, the action and the fitness;
- the MLP and its initialization;
- evolution strategies with the same antithetic pairs, centred ranks, Adam
  and SplitMix64 normal deviates.
The settings and seed come from RNE's training report, so only the simulator
differs. `--evaluate` runs one episode of a policy trained in either engine.

`--terrain TERRAIN.json` replaces the floor with the height field
`147_go2_learning --write-terrain TERRAIN.json --terrain SEED` writes, and
the fall test measures the base's height above the ground below it, as RNE
does; `check-terrain` ray-casts the field at the file's probe points against
RNE's heights.

Usage: mujoco_go2_learning.py train RNE_REPORT.json OUTPUT.json [--processes 4]
       mujoco_go2_learning.py evaluate POLICY.json [--generation N]
       mujoco_go2_learning.py check-terrain TERRAIN.json
"""
import argparse
import json
import math
import multiprocessing
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

REPO = Path(__file__).resolve().parents[2]
URDF = REPO / "assets/robots/go2_description/go2_description.rne.urdf"
JOINTS = [f"{leg}_{part}_joint" for leg in ("FL", "FR", "RL", "RR")
          for part in ("hip", "thigh", "calf")]
DEFAULT_RAD = np.array([0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5])
KP_NM_PER_RAD, KD_NM_S_PER_RAD = 20.0, 0.5
DT_S, DECIMATION = 0.005, 4
CONTROL_DT_S = DT_S * DECIMATION
SPAWN_HEIGHT_M = 0.36
SETTLE_STEPS, EPISODE_STEPS = 50, 250
GAIT_PERIOD_S, ACTION_SCALE_RAD = 0.5, 0.5
FALL_HEIGHT_M, FALL_GRAVITY_Z = 0.10, -0.5
LATERAL_PENALTY, HEADING_PENALTY, FALL_PENALTY, SMOOTHNESS_PENALTY = 0.5, 0.5, 1.0, 0.02
OBSERVATIONS, ACTIONS = 44, 12


def urdf_without_visuals():
    """The URDF without visuals and materials, which the importer would load
    (and whose repeated material names it rejects)."""
    root = ET.parse(URDF).getroot()
    for parent in root.iter():
        for child in [c for c in parent if c.tag in ("visual", "material")]:
            parent.remove(child)
    return ET.tostring(root, encoding="unicode")


def load_terrain(path):
    """A terrain file, or None for the flat floor."""
    if path is None:
        return None
    terrain = json.loads(Path(path).read_text())
    terrain["grid"] = np.array(terrain["heights_m"]).reshape(terrain["rows"], terrain["columns"])
    return terrain


def ground_height(terrain, x, y):
    """The terrain's bilinear height below (x, y), as RNE's height field."""
    if terrain is None:
        return 0.0
    grid, size = terrain["grid"], terrain["size_m"]
    rows, columns = grid.shape
    u = (x + 0.5 * size) / (size / (columns - 1))
    v = (y + 0.5 * size) / (size / (rows - 1))
    if not (0.0 <= u <= columns - 1 and 0.0 <= v <= rows - 1):
        return -math.inf
    column, row = min(int(u), columns - 2), min(int(v), rows - 2)
    fu, fv = u - column, v - row
    return (grid[row, column] * (1 - fu) * (1 - fv) + grid[row, column + 1] * fu * (1 - fv)
            + grid[row + 1, column] * (1 - fu) * fv + grid[row + 1, column + 1] * fu * fv)


def add_floor(spec, floor_friction, terrain):
    """The flat floor, or the terrain as a height field: MuJoCo scales its
    normalized elevation by size[2] above the geom, so the geom sits at the
    lowest height and size[2] spans the range."""
    friction = [floor_friction, 0.005, 0.0001]
    if terrain is None:
        floor = spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[20, 20, 0.1],
                                        friction=friction)
    else:
        grid = terrain["grid"]
        low, high = float(grid.min()), float(grid.max())
        half = 0.5 * terrain["size_m"]
        field = spec.add_hfield(name="terrain", nrow=grid.shape[0], ncol=grid.shape[1],
                                size=[half, half, max(high - low, 1e-6), 0.5])
        field.userdata = ((grid - low) / max(high - low, 1e-6)).flatten().tolist()
        floor = spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_HFIELD, hfieldname="terrain",
                                        pos=[0.0, 0.0, low], friction=friction)
    floor.contype, floor.conaffinity = 0, 1


def build_model(floor_friction, cone, impratio, terrain=None):
    spec = mujoco.MjSpec.from_string(urdf_without_visuals())
    # Links without an <inertial> are massless, as in the URDF specification
    # and in RNE, instead of taking mass from their geoms at 1000 kg/m^3.
    spec.compiler.inertiafromgeom = mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_FALSE
    spec.option.timestep = DT_S
    spec.option.cone = {"pyramidal": mujoco.mjtCone.mjCONE_PYRAMIDAL,
                        "elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC}[cone]
    spec.option.impratio = impratio
    spec.body("base").add_freejoint()
    for geom in spec.geoms:
        geom.contype, geom.conaffinity = 1, 0
        geom.friction = [floor_friction, 0.005, 0.0001]
    add_floor(spec, floor_friction, terrain)
    for name in JOINTS:
        actuator = spec.add_actuator(name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        actuator.set_to_position(kp=KP_NM_PER_RAD, kv=KD_NM_S_PER_RAD)
        actuator.forcelimited = True
        # The URDF effort limit, which the importer stores on the joint.
        actuator.forcerange = spec.joint(name).actfrcrange
    return spec.compile()


class Go2:
    """One Go2 that runs example 147's episode."""

    def __init__(self, plant):
        self.terrain = load_terrain(plant.get("terrain"))
        self.model = build_model(plant["floor_friction"], plant["cone"], plant["impratio"],
                                 self.terrain)
        self.data = mujoco.MjData(self.model)
        ids = [self.model.joint(name).id for name in JOINTS]
        self.qpos = np.array([self.model.jnt_qposadr[i] for i in ids])
        self.qvel = np.array([self.model.jnt_dofadr[i] for i in ids])
        self.rotation = np.zeros(9)

    def settle(self):
        """Start in the default pose at rest, base at 0.36 m, and stand."""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:3] = [0.0, 0.0, SPAWN_HEIGHT_M]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qpos[self.qpos] = DEFAULT_RAD
        mujoco.mj_forward(self.model, self.data)
        for _ in range(SETTLE_STEPS):
            self.step(DEFAULT_RAD)

    def step(self, targets_rad):
        self.data.ctrl[:] = targets_rad
        mujoco.mj_step(self.model, self.data, nstep=DECIMATION)

    def body_rotation(self):
        mujoco.mju_quat2Mat(self.rotation, self.data.qpos[3:7])
        return self.rotation.reshape(3, 3)

    def observe(self, step, action):
        rotation = self.body_rotation()
        spin = self.data.qvel[3:6]  # the free joint's angular velocity is in the body frame
        gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
        phase = (step * CONTROL_DT_S % GAIT_PERIOD_S) / GAIT_PERIOD_S
        angle = 2.0 * math.pi * phase
        return np.concatenate([spin * 0.25, gravity, self.data.qpos[self.qpos] - DEFAULT_RAD,
                               self.data.qvel[self.qvel] * 0.05, action,
                               [math.sin(angle), math.cos(angle)]])

    def heading(self):
        w, x, y, z = self.data.qpos[3:7]
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def episode(self, policy):
        """As go2.rs's episode."""
        self.settle()
        start = self.data.qpos[:3].copy()
        action = np.zeros(ACTIONS)
        change_square, steps, fell = 0.0, 0, False
        for step in range(EPISODE_STEPS):
            following = policy.act(self.observe(step, action))
            change_square += float(np.sum((following - action) ** 2))
            action = following
            self.step(DEFAULT_RAD + ACTION_SCALE_RAD * action)
            steps += 1
            gravity_z = -self.body_rotation()[2, 2]
            x, y, z = self.data.qpos[:3]
            if z - ground_height(self.terrain, x, y) < FALL_HEIGHT_M or gravity_z > FALL_GRAVITY_Z:
                fell = True
                break
        forward, lateral = self.data.qpos[0] - start[0], self.data.qpos[1] - start[1]
        heading = self.heading()
        fitness = (forward - LATERAL_PENALTY * abs(lateral) - HEADING_PENALTY * abs(heading)
                   - SMOOTHNESS_PENALTY * change_square / steps - (FALL_PENALTY if fell else 0.0))
        return {"fitness": fitness, "forward_m": forward, "lateral_m": lateral,
                "heading_rad": heading, "survived_s": steps * CONTROL_DT_S, "fell": fell,
                "control_steps": SETTLE_STEPS + steps}


class Mlp:
    """a = tanh(W2 tanh(W1 x + b1) + b2), parameters flat as policy.rs orders them."""

    def __init__(self, hidden, params):
        self.hidden, self.params = hidden, np.asarray(params, dtype=float)
        p, h = self.params, hidden
        self.w1 = p[:h * OBSERVATIONS].reshape(h, OBSERVATIONS)
        self.b1 = p[h * OBSERVATIONS:h * OBSERVATIONS + h]
        rest = h * OBSERVATIONS + h
        self.w2 = p[rest:rest + ACTIONS * h].reshape(ACTIONS, h)
        self.b2 = p[rest + ACTIONS * h:]

    @staticmethod
    def parameter_count(hidden):
        return hidden * OBSERVATIONS + hidden + ACTIONS * hidden + ACTIONS

    def act(self, observation):
        return np.tanh(self.w2 @ np.tanh(self.w1 @ observation + self.b1) + self.b2)

    def to_json(self):
        return {"model": "mlp", "inputs": OBSERVATIONS, "hidden": self.hidden,
                "outputs": ACTIONS, "params": self.params.tolist()}


class Normal:
    """SplitMix64 normal deviates (Box-Muller), as es.rs draws them."""

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


def centred_ranks(values):
    order = sorted(range(len(values)), key=lambda i: (values[i], i))
    ranks = [0.0] * len(values)
    denominator = max(len(values) - 1, 1)
    for rank, index in enumerate(order):
        ranks[index] = rank / denominator - 0.5
    return ranks


ROBOT = None


def init_worker(plant):
    global ROBOT
    ROBOT = Go2(plant)


def run_candidate(job):
    hidden, params = job
    return ROBOT.episode(Mlp(hidden, params))


def outcome_json(outcome):
    return {key: outcome[key] for key in
            ("fitness", "forward_m", "lateral_m", "heading_rad", "survived_s", "fell")}


def train(rne_report, plant, processes):
    config = rne_report["training"]["config"]
    hidden, pairs, sigma = config["hidden"], config["pairs"], config["sigma"]
    learning_rate, generations = config["learning_rate"], config["generations"]
    normal = Normal(config["seed"])
    dimension = Mlp.parameter_count(hidden)
    theta = np.zeros(dimension)
    for index in range(hidden * OBSERVATIONS):
        theta[index] = normal.sample() / math.sqrt(OBSERVATIONS)
    m, v = np.zeros(dimension), np.zeros(dimension)
    beta1, beta2 = 0.9, 0.999
    log, checkpoints, control_steps = [], [], 0
    with multiprocessing.Pool(processes, initializer=init_worker, initargs=(plant,)) as pool:
        begin = time.perf_counter()
        for generation in range(generations + 1):
            noise = np.array([[normal.sample() for _ in range(dimension)] for _ in range(pairs)])
            candidates = [theta.copy()]
            if generation < generations:
                for epsilon in noise:
                    for sign in (1.0, -1.0):
                        candidates.append(theta + sign * sigma * epsilon)
            outcomes = pool.map(run_candidate, [(hidden, c) for c in candidates])
            control_steps += sum(o["control_steps"] for o in outcomes)
            fitness = [o["fitness"] for o in outcomes]
            entry = {"generation": generation, "mean": outcome_json(outcomes[0]),
                     "population_mean_fitness": sum(fitness) / len(fitness),
                     "population_best_fitness": max(fitness),
                     "wall_s": time.perf_counter() - begin,
                     "simulated_s": control_steps * CONTROL_DT_S}
            print(json.dumps(entry), flush=True)
            every = config["checkpoint_every"]
            if every and (generation % every == 0 or generation == generations):
                checkpoints.append({"generation": generation, "wall_s": entry["wall_s"],
                                    "simulated_s": entry["simulated_s"],
                                    "policy": Mlp(hidden, theta).to_json()})
            log.append(entry)
            if generation == generations:
                break
            ranks = centred_ranks(fitness[1:])
            gradient = np.zeros(dimension)
            for pair, epsilon in enumerate(noise):
                gradient += (ranks[2 * pair] - ranks[2 * pair + 1]) * epsilon / (pairs * sigma)
            t = generation + 1
            m = beta1 * m + (1.0 - beta1) * gradient
            v = beta2 * v + (1.0 - beta2) * gradient * gradient
            theta = theta + learning_rate * (m / (1.0 - beta1 ** t)) / (
                np.sqrt(v / (1.0 - beta2 ** t)) + 1e-8)
    return {"schema_version": 1, "engine": f"mujoco {mujoco.__version__}", "plant": plant,
            "processes": processes,
            "training": {"method": rne_report["training"]["method"], "parameters": dimension,
                         "config": config, "log": log, "checkpoints": checkpoints},
            "policy": Mlp(hidden, theta).to_json()}


def load_policy(path, generation):
    report = json.loads(Path(path).read_text())
    if generation is not None:
        value = next(c["policy"] for c in report["training"]["checkpoints"]
                     if c["generation"] == generation)
    else:
        value = report.get("policy", report)
    return Mlp(value["hidden"], value["params"])


def check_terrain(path):
    """Ray-cast MuJoCo's copy of the terrain at RNE's probe points."""
    terrain = load_terrain(path)
    model = build_model(1.0, "elliptic", 100.0, terrain)
    data = mujoco.MjData(model)
    data.qpos[2] = 100.0  # the robot out of the rays' way
    mujoco.mj_forward(model, data)
    hfield_geoms = [g for g in range(model.ngeom)
                    if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_HFIELD]
    worst, worst_bilinear = 0.0, 0.0
    geomid = np.zeros(1, dtype=np.int32)
    for x, y, height in terrain["probes"]:
        start = np.array([x, y, 5.0])
        distance = mujoco.mj_ray(model, data, start, np.array([0.0, 0.0, -1.0]), None, 1, -1,
                                 geomid)
        if distance < 0 or geomid[0] not in hfield_geoms:
            raise SystemExit(f"no height-field hit at ({x:.3f}, {y:.3f})")
        hit = 5.0 - distance
        worst = max(worst, abs(hit - height))
        worst_bilinear = max(worst_bilinear, abs(ground_height(terrain, x, y) - height))
    print(json.dumps({"probes": len(terrain["probes"]), "worst_ray_error_m": worst,
                      "worst_bilinear_error_m": worst_bilinear}))
    # MuJoCo triangulates each 10 cm cell, RNE interpolates it bilinearly:
    # they differ by at most the cell's twist, well under a millimetre here.
    if worst > 2e-3 or worst_bilinear > 1e-9:
        raise SystemExit("MuJoCo's terrain does not match RNE's")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("train", "evaluate", "bench", "stand"):
        command = sub.add_parser(name)
        command.add_argument("--cone", choices=["pyramidal", "elliptic"], default="elliptic")
        command.add_argument("--impratio", type=float, default=100.0)
        command.add_argument("--friction", type=float, default=None,
                             help="floor and robot friction (default: the RNE report's, or 1)")
        command.add_argument("--terrain", type=Path, default=None,
                             help="height field written by 147_go2_learning --write-terrain")
    sub.add_parser("check-terrain").add_argument("terrain", type=Path)
    sub.choices["train"].add_argument("rne_report", type=Path)
    sub.choices["train"].add_argument("output", type=Path)
    sub.choices["train"].add_argument("--processes", type=int, default=4)
    sub.choices["evaluate"].add_argument("policy", type=Path)
    sub.choices["evaluate"].add_argument("--generation", type=int)
    sub.choices["stand"].add_argument("seconds", type=float)
    sub.choices["bench"].add_argument("episodes", type=int)
    sub.choices["bench"].add_argument("--seed", type=int, default=2026)
    sub.choices["bench"].add_argument("--hidden", type=int, default=32)
    arguments = parser.parse_args()
    if arguments.command == "check-terrain":
        check_terrain(arguments.terrain)
        return
    if arguments.command == "train":
        rne_report = json.loads(arguments.rne_report.read_text())
        friction = (rne_report["plant"]["floor_friction"] if arguments.friction is None
                    else arguments.friction)
        plant = {"floor_friction": friction, "cone": arguments.cone,
                 "impratio": arguments.impratio,
                 "terrain": None if arguments.terrain is None else str(arguments.terrain)}
        if (rne_report["plant"].get("terrain_seed") is None) != (arguments.terrain is None):
            parser.error("train on terrain exactly when the RNE report did")
        report = train(rne_report, plant, arguments.processes)
        arguments.output.write_text(json.dumps(report, indent=2) + "\n")
        print(f"wrote {arguments.output}")
    elif arguments.command == "bench":
        # One process, one model: the initial policy's episodes, as --bench.
        plant = {"floor_friction": 1.0 if arguments.friction is None else arguments.friction,
                 "cone": arguments.cone, "impratio": arguments.impratio,
                 "terrain": None if arguments.terrain is None else str(arguments.terrain)}
        normal = Normal(arguments.seed)
        params = np.zeros(Mlp.parameter_count(arguments.hidden))
        for index in range(arguments.hidden * OBSERVATIONS):
            params[index] = normal.sample() / math.sqrt(OBSERVATIONS)
        robot, policy = Go2(plant), Mlp(arguments.hidden, params)
        begin = time.perf_counter()
        steps = sum(robot.episode(policy)["control_steps"] for _ in range(arguments.episodes))
        wall = time.perf_counter() - begin
        print(json.dumps({"engine": f"mujoco {mujoco.__version__}", "plant": plant,
                          "episodes": arguments.episodes,
                          "simulated_s": steps * CONTROL_DT_S, "wall_s": wall,
                          "simulated_s_per_wall_s": steps * CONTROL_DT_S / wall}))
    elif arguments.command == "stand":
        # Settle as an episode does, then hold the default targets, as --stand.
        plant = {"floor_friction": 1.0 if arguments.friction is None else arguments.friction,
                 "cone": arguments.cone, "impratio": arguments.impratio,
                 "terrain": None if arguments.terrain is None else str(arguments.terrain)}
        robot = Go2(plant)
        robot.settle()
        for _ in range(round(arguments.seconds / CONTROL_DT_S)):
            robot.step(DEFAULT_RAD)
        rotation = robot.body_rotation()
        print(json.dumps({
            "engine": f"mujoco {mujoco.__version__}", "plant": plant,
            "seconds_after_settling": arguments.seconds,
            "base_m": robot.data.qpos[:3].tolist(),
            "base_speed_m_s": float(np.linalg.norm(robot.data.qvel[:3])),
            "pitch_rad": float(math.asin(rotation[2, 0])),
            "roll_rad": float(math.asin(rotation[2, 1])),
            "tilt_rad": float(math.acos(max(-1.0, min(1.0, rotation[2, 2])))),
            "joint_rad": robot.data.qpos[robot.qpos].tolist(),
            "mass_kg": float(robot.model.body_mass.sum())}))
    else:
        plant = {"floor_friction": 1.0 if arguments.friction is None else arguments.friction,
                 "cone": arguments.cone, "impratio": arguments.impratio,
                 "terrain": None if arguments.terrain is None else str(arguments.terrain)}
        outcome = Go2(plant).episode(load_policy(arguments.policy, arguments.generation))
        print(json.dumps({"engine": f"mujoco {mujoco.__version__}", "plant": plant,
                          "policy": str(arguments.policy), "generation": arguments.generation,
                          "outcome": outcome_json(outcome)}))


if __name__ == "__main__":
    main()
