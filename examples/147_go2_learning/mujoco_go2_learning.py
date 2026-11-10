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
RNE's heights, and checks the height scan at the file's scan probes.

Training follows the RNE report's task. With `random_starts`, every
generation starts at its own point of the terrain, from the same SplitMix64
stream as RNE's. With `height_scan`, the policy also reads 15 vertical rays
around the base, cast with `mj_ray` against the floor alone, scaled as RNE
scales them. `evaluate --starts N` walks from RNE's N held-out starts and
reports the mean.

Usage: mujoco_go2_learning.py train RNE_REPORT.json OUTPUT.json [--processes 4]
       mujoco_go2_learning.py evaluate POLICY.json [--generation N] [--starts N]
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
PROPRIOCEPTION, ACTIONS = 44, 12
# The height scan, as go2.rs defines it: vertical rays at these offsets in
# the base's heading frame, x outer and y inner; each reading is
# (base height above the hit - nominal) * gain, clipped to [-1, 1].
HEIGHT_SCAN_X_M = (-0.2, 0.0, 0.2, 0.4, 0.6)
HEIGHT_SCAN_Y_M = (-0.15, 0.0, 0.15)
HEIGHT_SCAN_POINTS = len(HEIGHT_SCAN_X_M) * len(HEIGHT_SCAN_Y_M)
HEIGHT_SCAN_NOMINAL_M, HEIGHT_SCAN_GAIN_PER_M = 0.3, 5.0
# The floor's geom group, which the scan's rays alone see.
FLOOR_GROUP = 3
# An episode ends as a fall when the base comes this close to the floor's
# edge, RNE's flat floor being this wide, as go2.rs and locomotion.rs.
ARENA_EDGE_MARGIN_M, FLAT_FLOOR_SIZE_M = 1.0, 40.0
# Random starts, as go2.rs draws them.
START_X_RANGE_M, START_Y_RANGE_M = (-8.0, 2.0), (-8.0, 8.0)
START_STREAM, EVALUATION_STARTS_SEED = 0x5354415254530147, 0x4556414C0147


def observations(task):
    return PROPRIOCEPTION + (HEIGHT_SCAN_POINTS if task.get("height_scan") else 0)


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
    # Visualization and ray filtering only: the height scan sees the floor alone.
    floor.group = FLOOR_GROUP


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

    def __init__(self, plant, task=None):
        self.task = task or {}
        self.terrain = load_terrain(plant.get("terrain"))
        self.model = build_model(plant["floor_friction"], plant["cone"], plant["impratio"],
                                 self.terrain)
        self.data = mujoco.MjData(self.model)
        ids = [self.model.joint(name).id for name in JOINTS]
        self.qpos = np.array([self.model.jnt_qposadr[i] for i in ids])
        self.qvel = np.array([self.model.jnt_dofadr[i] for i in ids])
        self.rotation = np.zeros(9)
        self.ray_groups = np.zeros(6, dtype=np.uint8)
        self.ray_groups[FLOOR_GROUP] = 1
        self.ray_top = 1.0 + (0.0 if self.terrain is None else float(self.terrain["grid"].max()))
        self.ray_hit = np.zeros(1, dtype=np.int32)
        # The base must stay this far inside the floor's edge, as go2.rs's
        # arena: RNE's flat floor is 40 m square, the terrain its own size.
        side = FLAT_FLOOR_SIZE_M if self.terrain is None else self.terrain["size_m"]
        self.arena_half = 0.5 * side - ARENA_EDGE_MARGIN_M

    def settle(self, start=(0.0, 0.0)):
        """Start in the default pose at rest, base 0.36 m above `start`, and
        stand. The terrain is zero at the origin, so a start there keeps the
        flat floor's height exactly, as go2.rs does."""
        mujoco.mj_resetData(self.model, self.data)
        x, y = start
        ground = 0.0 if (x, y) == (0.0, 0.0) else ground_height(self.terrain, x, y)
        self.data.qpos[:3] = [x, y, SPAWN_HEIGHT_M + ground]
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

    def height_scan(self):
        """The scan's readings from mj_ray hits on the floor."""
        x0, y0, z0 = self.data.qpos[:3]
        heading = self.heading()
        sin, cos = math.sin(heading), math.cos(heading)
        down = np.array([0.0, 0.0, -1.0])
        readings = []
        for dx in HEIGHT_SCAN_X_M:
            for dy in HEIGHT_SCAN_Y_M:
                start = np.array([x0 + cos * dx - sin * dy, y0 + sin * dx + cos * dy,
                                  self.ray_top])
                distance = mujoco.mj_ray(self.model, self.data, start, down, self.ray_groups, 1,
                                         -1, self.ray_hit)
                hit = -math.inf if distance < 0 else self.ray_top - distance
                reading = (z0 - hit - HEIGHT_SCAN_NOMINAL_M) * HEIGHT_SCAN_GAIN_PER_M
                readings.append(min(max(reading, -1.0), 1.0))
        return readings

    def observe(self, step, action):
        rotation = self.body_rotation()
        spin = self.data.qvel[3:6]  # the free joint's angular velocity is in the body frame
        gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
        phase = (step * CONTROL_DT_S % GAIT_PERIOD_S) / GAIT_PERIOD_S
        angle = 2.0 * math.pi * phase
        parts = [spin * 0.25, gravity, self.data.qpos[self.qpos] - DEFAULT_RAD,
                 self.data.qvel[self.qvel] * 0.05, action, [math.sin(angle), math.cos(angle)]]
        if self.task.get("height_scan"):
            parts.append(self.height_scan())
        return np.concatenate(parts)

    def heading(self):
        w, x, y, z = self.data.qpos[3:7]
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def episode(self, policy, start=(0.0, 0.0)):
        """As go2.rs's episode, from `start`."""
        return self.episode_with(policy.act, start)[0]

    def episode_with(self, act, start=(0.0, 0.0), on_step=None):
        """As go2.rs's episode_with: `act` maps an observation to the action
        applied, `on_step(reward, fell)` follows every control period, and the
        observation after the last period comes back with the outcome."""
        self.settle(start)
        start = self.data.qpos[:3].copy()
        action = np.zeros(ACTIONS)
        change_square, steps, fell = 0.0, 0, False
        last_x, last_lateral, last_heading = start[0], 0.0, 0.0
        for step in range(EPISODE_STEPS):
            following = act(self.observe(step, action))
            step_change = float(np.sum((following - action) ** 2))
            change_square += step_change
            action = following
            self.step(DEFAULT_RAD + ACTION_SCALE_RAD * action)
            steps += 1
            gravity_z = -self.body_rotation()[2, 2]
            x, y, z = self.data.qpos[:3]
            outside = abs(x) > self.arena_half or abs(y) > self.arena_half
            fell = bool(z - ground_height(self.terrain, x, y) < FALL_HEIGHT_M
                        or gravity_z > FALL_GRAVITY_Z or outside)
            lateral, heading = abs(y - start[1]), abs(self.heading())
            if on_step is not None:
                on_step((x - last_x) - LATERAL_PENALTY * (lateral - last_lateral)
                        - HEADING_PENALTY * (heading - last_heading)
                        - SMOOTHNESS_PENALTY * step_change / EPISODE_STEPS
                        - (FALL_PENALTY if fell else 0.0), fell)
            last_x, last_lateral, last_heading = x, lateral, heading
            if fell:
                break
        last = self.observe(steps, action)
        forward, lateral = self.data.qpos[0] - start[0], self.data.qpos[1] - start[1]
        heading = self.heading()
        fitness = (forward - LATERAL_PENALTY * abs(lateral) - HEADING_PENALTY * abs(heading)
                   - SMOOTHNESS_PENALTY * change_square / steps - (FALL_PENALTY if fell else 0.0))
        return ({"fitness": fitness, "forward_m": forward, "lateral_m": lateral,
                 "heading_rad": heading, "survived_s": steps * CONTROL_DT_S, "fell": fell,
                 "control_steps": SETTLE_STEPS + steps}, last)


class Mlp:
    """a = tanh(W2 tanh(W1 x + b1) + b2), parameters flat as policy.rs orders them."""

    def __init__(self, inputs, hidden, params):
        self.inputs, self.hidden = inputs, hidden
        self.params = np.asarray(params, dtype=float)
        p, h = self.params, hidden
        self.w1 = p[:h * inputs].reshape(h, inputs)
        self.b1 = p[h * inputs:h * inputs + h]
        rest = h * inputs + h
        self.w2 = p[rest:rest + ACTIONS * h].reshape(ACTIONS, h)
        self.b2 = p[rest + ACTIONS * h:]

    @staticmethod
    def parameter_count(inputs, hidden):
        return hidden * inputs + hidden + ACTIONS * hidden + ACTIONS

    @staticmethod
    def initial_params(inputs, hidden, normal):
        """W1 from `normal`'s first deviates over sqrt(inputs), as policy.rs."""
        params = np.zeros(Mlp.parameter_count(inputs, hidden))
        for index in range(hidden * inputs):
            params[index] = normal.sample() / math.sqrt(inputs)
        return params

    def act(self, observation):
        return np.tanh(self.w2 @ np.tanh(self.w1 @ observation + self.b1) + self.b2)

    def to_json(self):
        return {"model": "mlp", "inputs": self.inputs, "hidden": self.hidden,
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


def start_points(seed, count):
    """`count` start points uniform in the start box, as go2.rs draws them."""
    stream = Normal(seed)
    points = []
    for _ in range(count):
        u, v = stream.uniform(), stream.uniform()
        points.append((START_X_RANGE_M[0] + (START_X_RANGE_M[1] - START_X_RANGE_M[0]) * u,
                       START_Y_RANGE_M[0] + (START_Y_RANGE_M[1] - START_Y_RANGE_M[0]) * v))
    return points


def training_starts(seed, generations):
    return start_points(seed ^ START_STREAM, generations + 1)


def evaluation_starts(count):
    return start_points(EVALUATION_STARTS_SEED, count)


def centred_ranks(values):
    order = sorted(range(len(values)), key=lambda i: (values[i], i))
    ranks = [0.0] * len(values)
    denominator = max(len(values) - 1, 1)
    for rank, index in enumerate(order):
        ranks[index] = rank / denominator - 0.5
    return ranks


ROBOT = None


def init_worker(plant, task):
    global ROBOT
    ROBOT = Go2(plant, task)


def run_candidate(job):
    inputs, hidden, params, start = job
    return ROBOT.episode(Mlp(inputs, hidden, params), start)


def outcome_json(outcome):
    return {key: outcome[key] for key in
            ("fitness", "forward_m", "lateral_m", "heading_rad", "survived_s", "fell")}


def train(rne_report, plant, processes):
    config = rne_report["training"]["config"]
    hidden, pairs, sigma = config["hidden"], config["pairs"], config["sigma"]
    learning_rate, generations = config["learning_rate"], config["generations"]
    task = {"height_scan": False, "random_starts": False, **rne_report.get("task", {})}
    inputs = observations(task)
    starts = training_starts(config["seed"], generations)
    normal = Normal(config["seed"])
    dimension = Mlp.parameter_count(inputs, hidden)
    theta = Mlp.initial_params(inputs, hidden, normal)
    m, v = np.zeros(dimension), np.zeros(dimension)
    beta1, beta2 = 0.9, 0.999
    log, checkpoints, control_steps = [], [], 0
    with multiprocessing.Pool(processes, initializer=init_worker,
                              initargs=(plant, task)) as pool:
        begin = time.perf_counter()
        for generation in range(generations + 1):
            noise = np.array([[normal.sample() for _ in range(dimension)] for _ in range(pairs)])
            candidates = [theta.copy()]
            if generation < generations:
                for epsilon in noise:
                    for sign in (1.0, -1.0):
                        candidates.append(theta + sign * sigma * epsilon)
            start = starts[generation] if task["random_starts"] else (0.0, 0.0)
            outcomes = pool.map(run_candidate, [(inputs, hidden, c, start) for c in candidates])
            control_steps += sum(o["control_steps"] for o in outcomes)
            fitness = [o["fitness"] for o in outcomes]
            entry = {"generation": generation, "mean": outcome_json(outcomes[0]),
                     "population_mean_fitness": sum(fitness) / len(fitness),
                     "population_best_fitness": max(fitness),
                     "wall_s": time.perf_counter() - begin,
                     "simulated_s": control_steps * CONTROL_DT_S}
            if task["random_starts"]:
                entry["start_m"] = list(start)
            print(json.dumps(entry), flush=True)
            every = config["checkpoint_every"]
            if every and (generation % every == 0 or generation == generations):
                checkpoints.append({"generation": generation, "wall_s": entry["wall_s"],
                                    "simulated_s": entry["simulated_s"],
                                    "policy": Mlp(inputs, hidden, theta).to_json()})
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
            "task": task, "processes": processes,
            "training": {"method": rne_report["training"]["method"], "parameters": dimension,
                         "config": config, "log": log, "checkpoints": checkpoints},
            "policy": Mlp(inputs, hidden, theta).to_json()}


# PPO (`--learner ppo` in RNE's report), as ppo.rs: the same networks,
# settings, rewards and random streams.
PPO_NOISE_STREAM = 0x5050_4F4E_4F49_5345
PPO_SHUFFLE_STREAM = 0x5050_4F53_4855_4646
PPO_CRITIC_STREAM = 0x5050_4F43_5249_5449
HALF_LOG_TAU = 0.5 * math.log(2.0 * math.pi)


class Normalizer:
    """Running observation means and variances (Chan's parallel update)."""

    def __init__(self, width):
        self.count, self.mean, self.m2 = 0.0, np.zeros(width), np.zeros(width)

    def scale(self):
        if self.count == 0.0:
            return np.ones_like(self.mean)
        return np.sqrt(self.m2 / self.count + 1e-8)

    def update(self, batch):
        if len(batch) == 0:
            return
        batch = np.asarray(batch)
        n = float(len(batch))
        mean = batch.mean(axis=0)
        m2 = ((batch - mean) ** 2).sum(axis=0)
        total = self.count + n
        delta = mean - self.mean
        self.mean = self.mean + delta * n / total
        self.m2 = self.m2 + m2 + delta * delta * self.count * n / total
        self.count = total

    def fold(self, inputs, hidden, params):
        """Actor parameters that read raw observations."""
        scale = self.scale()
        params = np.array(params, dtype=float)
        w1 = params[:hidden * inputs].reshape(hidden, inputs) / scale
        params[:hidden * inputs] = w1.ravel()
        params[hidden * inputs:hidden * inputs + hidden] -= w1 @ self.mean
        return params


def critic_initial(inputs, hidden, normal):
    count = hidden * inputs + hidden + hidden * hidden + hidden + hidden + 1
    params = np.zeros(count)
    for index in range(hidden * inputs):
        params[index] = normal.sample() / math.sqrt(inputs)
    w2 = hidden * inputs + hidden
    for index in range(hidden * hidden):
        params[w2 + index] = normal.sample() / math.sqrt(hidden)
    return params


def critic_parts(params, inputs, hidden):
    w1 = params[:hidden * inputs].reshape(hidden, inputs)
    b1 = params[hidden * inputs:hidden * inputs + hidden]
    at = hidden * inputs + hidden
    w2 = params[at:at + hidden * hidden].reshape(hidden, hidden)
    b2 = params[at + hidden * hidden:at + hidden * hidden + hidden]
    at += hidden * hidden + hidden
    return w1, b1, w2, b2, params[at:at + hidden], params[at + hidden]


def critic_values(params, inputs, hidden, x):
    w1, b1, w2, b2, w3, b3 = critic_parts(params, inputs, hidden)
    h1 = np.tanh(x @ w1.T + b1)
    h2 = np.tanh(h1 @ w2.T + b2)
    return h2 @ w3 + b3, h1, h2


def actor_parts(params, inputs, hidden):
    w1 = params[:hidden * inputs].reshape(hidden, inputs)
    b1 = params[hidden * inputs:hidden * inputs + hidden]
    at = hidden * inputs + hidden
    return w1, b1, params[at:at + ACTIONS * hidden].reshape(ACTIONS, hidden), \
        params[at + ACTIONS * hidden:]


class Adam:
    """Adam after scaling the gradient to a norm of at most max_norm, as ppo.rs."""

    def __init__(self, dimension):
        self.m, self.v, self.t = np.zeros(dimension), np.zeros(dimension), 0

    def step(self, params, grad, learning_rate, max_norm):
        norm = float(np.sqrt(np.sum(grad * grad)))
        grad = grad * (max_norm / norm if norm > max_norm else 1.0)
        self.t += 1
        self.m = 0.9 * self.m + 0.1 * grad
        self.v = 0.999 * self.v + 0.001 * grad * grad
        m_hat = self.m / (1.0 - 0.9 ** self.t)
        v_hat = self.v / (1.0 - 0.999 ** self.t)
        return params - learning_rate * m_hat / (np.sqrt(v_hat) + 1e-5)


def ppo_rollout(job):
    """One robot's episode with the stochastic policy (a pool job)."""
    inputs, hidden, actor, log_std, mean, scale, start, noise_seed = job
    w1, b1, w2, b2 = actor_parts(actor, inputs, hidden)
    noise, std = Normal(noise_seed), np.exp(log_std)
    record = {"observations": [], "raw": [], "actions": [], "log_probs": [], "rewards": [],
              "fell": False}

    def act(observation):
        x = (observation - mean) / scale
        mu = np.tanh(w2 @ np.tanh(w1 @ x + b1) + b2)
        action = mu + std * np.array([noise.sample() for _ in range(ACTIONS)])
        z = (action - mu) / std
        record["log_probs"].append(float(np.sum(-0.5 * z * z - log_std - HALF_LOG_TAU)))
        record["actions"].append(action)
        record["raw"].append(observation)
        record["observations"].append(x)
        return np.clip(action, -1.0, 1.0)

    def on_step(reward, fell):
        record["rewards"].append(reward)
        record["fell"] = record["fell"] or fell

    outcome, last = ROBOT.episode_with(act, start, on_step)
    record["bootstrap"] = None if record["fell"] else (last - mean) / scale
    record["outcome"] = outcome
    return record


def ppo_mean_episode(job):
    """The deterministic policy's episode (a pool job)."""
    inputs, hidden, params, start = job
    return ROBOT.episode(Mlp(inputs, hidden, params), start)


def ppo_update(actor, critic, log_std, adams, trajectories, iteration, config, inputs):
    hidden, gamma, lam = config["hidden"], config["gamma"], config["lambda"]
    xs, actions, old_log_probs, advantages, targets = [], [], [], [], []
    for t in trajectories:
        x = np.array(t["observations"])
        values = critic_values(critic, inputs, hidden, x)[0]
        last = 0.0 if t["bootstrap"] is None else float(
            critic_values(critic, inputs, hidden, t["bootstrap"][None, :])[0][0])
        adv, running = np.zeros(len(values)), 0.0
        for step in range(len(values) - 1, -1, -1):
            following = values[step + 1] if step + 1 < len(values) else last
            delta = t["rewards"][step] + gamma * following - values[step]
            running = delta + gamma * lam * running
            adv[step] = running
        xs.append(x)
        actions.append(np.array(t["actions"]))
        old_log_probs.append(np.array(t["log_probs"]))
        advantages.append(adv)
        targets.append(adv + values)
    x, a = np.concatenate(xs), np.concatenate(actions)
    old, adv, target = (np.concatenate(old_log_probs), np.concatenate(advantages),
                        np.concatenate(targets))
    adv = (adv - adv.mean()) / (adv.std() + 1e-8)
    shuffle = Normal(config["seed"] ^ PPO_SHUFFLE_STREAM ^ iteration)
    order = list(range(len(adv)))
    actor_adam, std_adam, critic_adam = adams
    for _ in range(config["epochs"]):
        for i in range(len(order) - 1, 0, -1):
            j = min(int(shuffle.uniform() * (i + 1)), i)
            order[i], order[j] = order[j], order[i]
        size = -(-len(order) // config["minibatches"])
        for begin in range(0, len(order), size):
            batch = np.array(order[begin:begin + size])
            n = float(len(batch))
            xb, ab, ob, advb, tb = x[batch], a[batch], old[batch], adv[batch], target[batch]
            w1, b1, w2, b2 = actor_parts(actor, inputs, hidden)
            h = np.tanh(xb @ w1.T + b1)
            mu = np.tanh(h @ w2.T + b2)
            std = np.exp(log_std)
            z = (ab - mu) / std
            new = np.sum(-0.5 * z * z - log_std - HALF_LOG_TAU, axis=1)
            ratio = np.exp(new - ob)
            clipped = ((advb > 0) & (ratio > 1 + config["clip"])) | \
                      ((advb < 0) & (ratio < 1 - config["clip"]))
            d_log_prob = np.where(clipped, 0.0, -ratio * advb / n)
            variance = std * std
            diff = ab - mu
            d_mu = d_log_prob[:, None] * diff / variance
            std_grad = np.sum(d_log_prob[:, None] * (diff * diff / variance - 1.0), axis=0)
            d_z2 = d_mu * (1 - mu * mu)
            d_h = d_z2 @ w2
            d_z1 = d_h * (1 - h * h)
            actor_grad = np.concatenate([(d_z1.T @ xb).ravel(), d_z1.sum(axis=0),
                                         (d_z2.T @ h).ravel(), d_z2.sum(axis=0)])
            values, h1, h2 = critic_values(critic, inputs, hidden, xb)
            cw1, cb1, cw2, cb2, cw3, cb3 = critic_parts(critic, inputs, hidden)
            d_v = (values - tb) / n
            d_h2 = d_v[:, None] * cw3 * (1 - h2 * h2)
            d_h1 = (d_h2 @ cw2) * (1 - h1 * h1)
            critic_grad = np.concatenate([(d_h1.T @ xb).ravel(), d_h1.sum(axis=0),
                                          (d_h2.T @ h1).ravel(), d_h2.sum(axis=0),
                                          d_v @ h2, [d_v.sum()]])
            actor = actor_adam.step(actor, actor_grad, config["learning_rate"],
                                    config["max_grad_norm"])
            log_std = std_adam.step(log_std, std_grad, config["learning_rate"],
                                    config["max_grad_norm"])
            critic = critic_adam.step(critic, critic_grad, config["learning_rate"],
                                      config["max_grad_norm"])
    return actor, critic, log_std


def train_ppo(rne_report, plant, processes):
    config = rne_report["training"]["config"]
    task = {"height_scan": False, "random_starts": False, **rne_report.get("task", {})}
    inputs, hidden, envs = observations(task), config["hidden"], config["envs"]
    iterations, seed = config["iterations"], config["seed"]
    actor = Mlp.initial_params(inputs, hidden, Normal(seed))
    critic = critic_initial(inputs, hidden, Normal(seed ^ PPO_CRITIC_STREAM))
    log_std = np.full(ACTIONS, math.log(config["initial_std"]))
    normalizer = Normalizer(inputs)
    adams = (Adam(len(actor)), Adam(ACTIONS), Adam(len(critic)))
    per_iteration = envs + 1
    starts = training_starts(seed, (iterations + 1) * per_iteration - 1)
    log, checkpoints, control_steps = [], [], 0
    with multiprocessing.Pool(processes, initializer=init_worker,
                              initargs=(plant, task)) as pool:
        begin = time.perf_counter()
        for iteration in range(iterations + 1):
            def start_of(index):
                return (starts[iteration * per_iteration + index] if task["random_starts"]
                        else (0.0, 0.0))
            policy = normalizer.fold(inputs, hidden, actor)
            mean_job = pool.apply_async(ppo_mean_episode,
                                        ((inputs, hidden, policy, start_of(envs)),))
            trajectories = []
            if iteration < iterations:
                scale = normalizer.scale()
                jobs = [(inputs, hidden, actor, log_std, normalizer.mean, scale, start_of(env),
                         seed ^ PPO_NOISE_STREAM ^ (iteration << 20 | env))
                        for env in range(envs)]
                trajectories = pool.map(ppo_rollout, jobs)
            mean_outcome = mean_job.get()
            control_steps += mean_outcome["control_steps"] + sum(
                t["outcome"]["control_steps"] for t in trajectories)
            fitness = [t["outcome"]["fitness"] for t in trajectories]
            entry = {"generation": iteration, "mean": outcome_json(mean_outcome),
                     "population_mean_fitness": sum(fitness) / len(fitness) if fitness else None,
                     "population_best_fitness": max(fitness) if fitness else None,
                     "action_std": float(np.exp(log_std).mean()),
                     "wall_s": time.perf_counter() - begin,
                     "simulated_s": control_steps * CONTROL_DT_S}
            print(json.dumps(entry), flush=True)
            every = config["checkpoint_every"]
            if every and (iteration % every == 0 or iteration == iterations):
                checkpoints.append({"generation": iteration, "wall_s": entry["wall_s"],
                                    "simulated_s": entry["simulated_s"],
                                    "policy": Mlp(inputs, hidden, policy).to_json()})
            log.append(entry)
            if iteration == iterations:
                break
            actor, critic, log_std = ppo_update(actor, critic, log_std, adams, trajectories,
                                                iteration, config, inputs)
            normalizer.update([o for t in trajectories for o in t["raw"]])
    policy = normalizer.fold(inputs, hidden, actor)
    return {"schema_version": 1, "engine": f"mujoco {mujoco.__version__}", "plant": plant,
            "task": task, "processes": processes,
            "training": {"method": rne_report["training"]["method"],
                         "parameters": len(actor) + ACTIONS + len(critic),
                         "config": config, "log": log, "checkpoints": checkpoints},
            "policy": Mlp(inputs, hidden, policy).to_json()}


def load_policy(path, generation):
    """A policy file's policy and the task its inputs fit, as main.rs reads it."""
    report = json.loads(Path(path).read_text())
    if generation is not None:
        value = next(c["policy"] for c in report["training"]["checkpoints"]
                     if c["generation"] == generation)
    else:
        value = report.get("policy", report)
    inputs = value.get("inputs", PROPRIOCEPTION)
    task = {"height_scan": inputs > PROPRIOCEPTION,
            "random_starts": bool(report.get("task", {}).get("random_starts", False))}
    if inputs != observations(task):
        raise SystemExit(f"a policy with {inputs} inputs fits no task")
    return Mlp(inputs, value["hidden"], value["params"]), task


def mean_outcome(outcomes):
    """The mean outcome, a fall in any counting as a fall, as main.rs."""
    mean = {key: sum(o[key] for o in outcomes) / len(outcomes)
            for key in ("fitness", "forward_m", "lateral_m", "heading_rad", "survived_s")}
    mean["fell"] = any(o["fell"] for o in outcomes)
    return mean


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
    # The height scan at RNE's scan probes: the robot placed there, out of
    # the rays' way thanks to the floor-only ray group.
    robot = Go2({"floor_friction": 1.0, "cone": "elliptic", "impratio": 100.0,
                 "terrain": str(path)}, {"height_scan": True})
    worst_scan = 0.0
    scan_probes = terrain.get("height_scan", {}).get("probes", [])
    for probe in scan_probes:
        robot.data.qpos[:3] = probe["base_m"]
        half = 0.5 * probe["heading_rad"]
        robot.data.qpos[3:7] = [math.cos(half), 0.0, 0.0, math.sin(half)]
        mujoco.mj_forward(robot.model, robot.data)
        readings = robot.height_scan()
        worst_scan = max(worst_scan, max(abs(a - b) for a, b in zip(readings, probe["readings"])))
    # MuJoCo splits each 10 cm cell into two triangles, RNE interpolates it
    # bilinearly: they differ by at most a quarter of the cell's twist
    # (h00 + h11 - h01 - h10), at its centre. The rays may miss by that much
    # and a hair more, and the scan's readings by that times its gain.
    grid = terrain["grid"]
    twist = np.abs(grid[:-1, :-1] + grid[1:, 1:] - grid[:-1, 1:] - grid[1:, :-1]).max()
    ray_tolerance = 0.25 * float(twist) + 1e-4
    scan_tolerance = HEIGHT_SCAN_GAIN_PER_M * ray_tolerance
    print(json.dumps({"probes": len(terrain["probes"]), "worst_ray_error_m": worst,
                      "ray_tolerance_m": ray_tolerance,
                      "worst_bilinear_error_m": worst_bilinear,
                      "scan_probes": len(scan_probes), "worst_scan_reading_error": worst_scan,
                      "scan_tolerance": scan_tolerance}))
    if worst > ray_tolerance or worst_bilinear > 1e-9 or worst_scan > scan_tolerance:
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
    sub.choices["evaluate"].add_argument("--starts", type=int, default=None,
                                         help="walk from RNE's N held-out starts")
    sub.choices["stand"].add_argument("seconds", type=float)
    sub.choices["bench"].add_argument("episodes", type=int)
    sub.choices["bench"].add_argument("--seed", type=int, default=2026)
    sub.choices["bench"].add_argument("--hidden", type=int, default=32)
    sub.choices["bench"].add_argument("--height-scan", action="store_true")
    sub.choices["bench"].add_argument("--random-starts", action="store_true")
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
        learner = rne_report["training"]["config"].get("learner", "es")
        trainer = {"es": train, "ppo": train_ppo}[learner]
        report = trainer(rne_report, plant, arguments.processes)
        arguments.output.write_text(json.dumps(report, indent=2) + "\n")
        print(f"wrote {arguments.output}")
    elif arguments.command == "bench":
        # One process, one model: the initial policy's episodes, as --bench.
        plant = {"floor_friction": 1.0 if arguments.friction is None else arguments.friction,
                 "cone": arguments.cone, "impratio": arguments.impratio,
                 "terrain": None if arguments.terrain is None else str(arguments.terrain)}
        task = {"height_scan": arguments.height_scan, "random_starts": arguments.random_starts}
        inputs = observations(task)
        params = Mlp.initial_params(inputs, arguments.hidden, Normal(arguments.seed))
        robot, policy = Go2(plant, task), Mlp(inputs, arguments.hidden, params)
        starts = training_starts(arguments.seed, arguments.episodes)[:arguments.episodes]
        begin = time.perf_counter()
        steps = sum(robot.episode(policy, start if task["random_starts"] else (0.0, 0.0))
                    ["control_steps"] for start in starts)
        wall = time.perf_counter() - begin
        print(json.dumps({"engine": f"mujoco {mujoco.__version__}", "plant": plant,
                          "task": task, "episodes": arguments.episodes,
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
        policy, task = load_policy(arguments.policy, arguments.generation)
        robot = Go2(plant, task)
        points = [(0.0, 0.0)] if arguments.starts is None else evaluation_starts(arguments.starts)
        outcomes = [robot.episode(policy, start) for start in points]
        outcome = outcome_json(mean_outcome(outcomes))
        report = {"engine": f"mujoco {mujoco.__version__}", "plant": plant,
                  "policy": str(arguments.policy), "generation": arguments.generation,
                  "task": task}
        if arguments.starts is not None:
            outcome["falls"] = sum(o["fell"] for o in outcomes)
            report["episodes"] = [{"start_m": list(start), "outcome": outcome_json(o)}
                                  for start, o in zip(points, outcomes)]
        report["outcome"] = outcome
        print(json.dumps(report))


if __name__ == "__main__":
    main()
