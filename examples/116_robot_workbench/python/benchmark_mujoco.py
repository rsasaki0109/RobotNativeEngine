"""Compare fixed-base, configurable-gravity URDF torque responses with MuJoCo 3.9.0.

Runner-loop times include state reads and language/bridge overhead; they are not
an isolated physics-kernel throughput comparison. No contacts or joint stops.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np


def normalize_urdf(text):
    """Preserve frames and inertials; explicitly remove geometry, stops and losses."""
    root = ET.fromstring(text)
    names = []
    for link in root.findall("link"):
        if link.find("inertial") is None:
            raise ValueError("every link needs explicit inertial properties")
        # Canonicalize inertial orientation into the link frame. MuJoCo's URDF
        # full-inertia import must not silently change the physical tensor when
        # an inertial origin also carries a nonidentity rotation.
        inertial = link.find("inertial")
        origin = inertial.find("origin")
        rpy = np.fromstring(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" ")
        if rpy.shape != (3,) or not np.isfinite(rpy).all():
            raise ValueError("invalid inertial rotation")
        if np.any(rpy):
            cr, cp, cy = np.cos(rpy)
            sr, sp, sy = np.sin(rpy)
            rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
            ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            rotation = rz @ ry @ rx
            inertia = inertial.find("inertia")
            xx, xy, xz, yy, yz, zz = (float(inertia.attrib[k]) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"))
            tensor = rotation @ np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]) @ rotation.T
            for key, (i, j) in zip(("ixx", "ixy", "ixz", "iyy", "iyz", "izz"),
                                   ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))):
                inertia.set(key, repr(float(tensor[i, j])))
            origin.set("rpy", "0 0 0")
        for tag in ("visual", "collision"):
            for element in link.findall(tag):
                link.remove(element)
    for joint in root.findall("joint"):
        if joint.find("mimic") is not None:
            raise ValueError("mimic joints are not supported by this benchmark")
        if joint.get("type") == "fixed":
            continue
        if joint.get("type") not in ("continuous", "revolute"):
            raise ValueError("only rotary joints are supported")
        joint.set("type", "continuous")
        names.append(joint.attrib["name"])
        for tag in ("dynamics", "limit", "safety_controller"):
            for element in joint.findall(tag):
                joint.remove(element)
    if not names or len(names) != len(set(names)):
        raise ValueError("expected unique rotary joints")
    return ET.tostring(root, encoding="unicode"), sorted(names)


def run_rne(binary, request):
    with tempfile.TemporaryDirectory(prefix="rne-cross-sim-") as directory:
        path = Path(directory) / "input.json"
        path.write_text(json.dumps(request, allow_nan=False))
        result = subprocess.run([str(binary), "--torque-benchmark", str(path)],
                                check=True, capture_output=True, text=True, timeout=120)
    return json.loads(result.stdout)


def run_mujoco(request):
    import mujoco
    model = mujoco.MjModel.from_xml_string(request["urdf"])
    model.opt.timestep = request["dt_ticks"] * 1e-9
    model.opt.gravity[:] = request.get("gravity_m_s2", [0, 0, 0])
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_EULER
    model.dof_damping[:] = 0
    model.dof_frictionloss[:] = 0
    model.dof_armature[:] = 0
    model.jnt_stiffness[:] = 0
    model.jnt_limited[:] = False
    names = request["joint_names"]
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in names]
    if min(ids) < 0 or model.nv != len(names) or model.nq != len(names):
        raise ValueError("MuJoCo joint catalog differs from the shared model")
    qi, vi = model.jnt_qposadr[ids], model.jnt_dofadr[ids]
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    q, v = [data.qpos[qi].tolist()], [data.qvel[vi].tolist()]
    start = time.perf_counter()
    for torque in request["torques_nm"]:
        data.qfrc_applied[:] = 0
        data.qfrc_applied[vi] = torque
        mujoco.mj_step(model, data)
        q.append(data.qpos[qi].tolist())
        v.append(data.qvel[vi].tolist())
    return {"backend": "mujoco", "version": mujoco.__version__, "joint_names": names,
            "q_rad": q, "dq_rad_s": v, "runner_loop_s": time.perf_counter() - start}


def compare(a, b):
    if a["joint_names"] != b["joint_names"]:
        raise ValueError("joint ordering differs")
    result = {}
    for field, unit in (("q_rad", "rad"), ("dq_rad_s", "rad_s")):
        x, y = np.asarray(a[field]), np.asarray(b[field])
        if x.shape != y.shape or not np.isfinite(x).all() or not np.isfinite(y).all():
            raise ValueError("invalid or mismatched response shapes")
        result["max_abs_difference_" + unit] = np.max(np.abs(x - y), axis=0).tolist()
        result["rmse_" + unit] = np.sqrt(np.mean((x - y) ** 2, axis=0)).tolist()
    return result


def evaluate(binary, urdf, names, gravity_m_s2=(0, 0, 0)):
    results = []
    for dt_ticks in (1_000_000, 500_000):
        steps = 200_000_000 // dt_ticks
        for driven in [None, *names]:
            torques = np.zeros((steps, len(names)))
            if driven is not None:
                j = names.index(driven)
                torques[20_000_000 // dt_ticks:60_000_000 // dt_ticks, j] = 0.001
                torques[60_000_000 // dt_ticks:100_000_000 // dt_ticks, j] = -0.001
            request = {"urdf": urdf, "joint_names": names, "dt_ticks": dt_ticks,
                       "torques_nm": torques.tolist(), "gravity_m_s2": list(gravity_m_s2)}
            rne, mj = run_rne(binary, request), run_mujoco(request)
            record = {"driven_joint": driven, "dt_ticks": dt_ticks,
                      "torques_nm": request["torques_nm"], "rne": rne, "mujoco": mj,
                      "comparison": compare(rne, mj)}
            results.append(record)
            print(json.dumps({"joint": driven, "dt_ticks": dt_ticks,
                              "max_q_difference_rad": max(record["comparison"]["max_abs_difference_rad"])}), flush=True)
    return results


def trajectory_records(results):
    """Replay physics exactly within each backend; exclude nondeterministic timings."""
    return [{**r, "rne": {k: v for k, v in r["rne"].items() if k != "runner_loop_s"},
             "mujoco": {k: v for k, v in r["mujoco"].items() if k != "runner_loop_s"}}
            for r in results]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("source", type=Path, help="URDF, or saved report with --replay")
    parser.add_argument("output", type=Path, nargs="?")
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--gravity", type=float, nargs=3, metavar=("GX", "GY", "GZ"),
                        help="world gravity in m/s²; defaults to zero, replay uses saved gravity")
    args = parser.parse_args()
    if args.gravity is not None and (not np.isfinite(args.gravity).all() or max(map(abs, args.gravity)) > 100):
        parser.error("gravity components must be finite and at most 100 m/s² in magnitude")
    if args.replay:
        if args.gravity is not None:
            parser.error("replay uses saved gravity; omit --gravity")
        raw = args.source.read_bytes()
        report = json.loads(gzip.decompress(raw) if args.source.suffix == ".gz" else raw)
        if report.get("benchmark") not in ("so101_free_torque_v1", "so101_gravity_torque_v1"):
            parser.error("expected a torque benchmark report")
        results = evaluate(args.binary.resolve(), report["normalized_urdf"], report["joint_names"],
                           report["conditions"]["gravity_m_s2"])
        if trajectory_records(results) != trajectory_records(report["results"]):
            raise SystemExit("trajectory replay mismatch")
        print("exact cross-simulator trajectory replay passed (timings excluded)")
        return
    if args.output is None or args.output.exists():
        parser.error("provide a new output path")
    source = args.source.read_bytes()
    urdf, names = normalize_urdf(source.decode())
    gravity = args.gravity if args.gravity is not None else [0, 0, 0]
    report = {"benchmark": "so101_gravity_torque_v1" if any(gravity) else "so101_free_torque_v1", "source_urdf_sha256": hashlib.sha256(source).hexdigest(),
              "normalized_urdf_sha256": hashlib.sha256(urdf.encode()).hexdigest(),
              "normalized_urdf": urdf, "joint_names": names,
              "conditions": {"base": "fixed", "gravity_m_s2": gravity, "contacts": False,
                             "joint_limits": False, "joint_losses": False, "initial_q_dq": "zero",
                             "pulse_nm": 0.001, "positive_interval_s": [0.02, 0.06],
                             "negative_interval_s": [0.06, 0.10], "duration_s": 0.2},
              "timing_scope": "runner loop including actuation, stepping and state reads; not kernel throughput",
              "platform": platform.platform(), "python": platform.python_version(),
              "rne_binary_sha256": hashlib.sha256(args.binary.read_bytes()).hexdigest()}
    report["results"] = evaluate(args.binary.resolve(), urdf, names, gravity)
    raw = (json.dumps(report, indent=2, allow_nan=False) + "\n").encode()
    with args.output.open("xb") as output:
        output.write(gzip.compress(raw, mtime=0) if args.output.suffix == ".gz" else raw)


if __name__ == "__main__":
    main()
