#!/usr/bin/env python3
"""Independently verify kick-fixture provenance, inertia, limits, and kinematics.

The verifier does not import the generator. Forward kinematics uses homogeneous
matrices and Rodrigues rotations for the URDF joint axes. The derived joint
coordinates are compared with the original physical angles, including legal
interior-limit samples for every actuated joint.
"""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
METADATA = Path(__file__).with_name("models.json")
KINEMATIC_TOLERANCE = 1e-12
MARKER_MASS_KG = 1e-9
MARKER_INERTIA_KG_M2 = 1e-12
SOURCE_PINS = {
    "assets/robots/go2_description/go2_description.rne.urdf":
        "521d2f0bac3b22a942745e2fa0cb32ec979146de66f731f7befd806ec1f6ca98",
    "assets/robots/g1_description/g1_23dof.urdf":
        "b462e1367ccb50f148db8c82f93421cb3f47497b10b735e891d2c9d000870cfc",
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite(value, description):
    number = float(value)
    require(math.isfinite(number), f"non-finite {description}")
    return number


def vector(value, description, default="0 0 0"):
    result = tuple(finite(x, description) for x in (value or default).split())
    require(len(result) == 3, f"invalid {description}")
    return result


def canonical(element, omit_tags=()):
    """Compare XML semantics without indentation or attribute-order differences."""
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(canonical(child) for child in element if child.tag not in omit_tags),
    )


def named_elements(robot, tag):
    elements = robot.findall(tag)
    result = {element.get("name"): element for element in elements}
    require(None not in result and len(result) == len(elements), f"duplicate or unnamed {tag}")
    return result


def identity():
    return tuple(tuple(float(i == j) for j in range(4)) for i in range(4))


def multiply(left, right):
    return tuple(
        tuple(math.fsum(left[i][k] * right[k][j] for k in range(4)) for j in range(4))
        for i in range(4)
    )


def axis_rotation(axis, angle):
    length = math.sqrt(math.fsum(x * x for x in axis))
    require(length > 0.0, "an actuated joint has a zero rotation axis")
    x, y, z = (value / length for value in axis)
    cosine, sine = math.cos(angle), math.sin(angle)
    complement = 1.0 - cosine
    return (
        (cosine + x * x * complement, x * y * complement - z * sine,
         x * z * complement + y * sine, 0.0),
        (y * x * complement + z * sine, cosine + y * y * complement,
         y * z * complement - x * sine, 0.0),
        (z * x * complement - y * sine, z * y * complement + x * sine,
         cosine + z * z * complement, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def origin_matrix(element):
    if element is None:
        return identity()
    translation = vector(element.get("xyz"), "origin translation")
    roll, pitch, yaw = vector(element.get("rpy"), "origin rotation")
    rotation = multiply(
        axis_rotation((0.0, 0.0, 1.0), yaw),
        multiply(axis_rotation((0.0, 1.0, 0.0), pitch),
                 axis_rotation((1.0, 0.0, 0.0), roll)),
    )
    return tuple(
        tuple(translation[i] if j == 3 and i < 3 else rotation[i][j] for j in range(4))
        for i in range(4)
    )


def forward_kinematics(robot, positions):
    links = named_elements(robot, "link")
    joints = named_elements(robot, "joint")
    children = {}
    for name, joint in joints.items():
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        require(parent in links and child in links, f"joint {name} refers to an absent link")
        require(child not in children, f"link {child} has multiple parents")
        children[child] = (parent, joint)
    roots = sorted(set(links) - set(children))
    require(len(roots) == 1, "the model must have exactly one root")
    transforms = {roots[0]: identity()}
    pending = dict(children)
    while pending:
        progressed = False
        for child in sorted(pending):
            parent, joint = pending[child]
            if parent not in transforms:
                continue
            kind = joint.get("type")
            angle = positions.get(joint.get("name"), 0.0)
            motion = identity()
            if kind in ("revolute", "continuous"):
                motion = axis_rotation(
                    vector(joint.find("axis").get("xyz"), "joint axis", "1 0 0"), angle
                )
            elif kind == "prismatic":
                axis = vector(joint.find("axis").get("xyz"), "joint axis", "1 0 0")
                length = math.sqrt(math.fsum(x * x for x in axis))
                require(length > 0.0, "a prismatic joint has a zero axis")
                motion = tuple(
                    tuple(axis[i] * angle / length if j == 3 and i < 3 else motion[i][j]
                          for j in range(4))
                    for i in range(4)
                )
            else:
                require(kind == "fixed", f"unsupported joint type {kind}")
            transforms[child] = multiply(
                transforms[parent], multiply(origin_matrix(joint.find("origin")), motion)
            )
            del pending[child]
            progressed = True
        require(progressed, "the joint graph contains a cycle")
    return transforms


def determinant(matrix):
    a, b, c = matrix
    return (
        a[0] * (b[1] * c[2] - b[2] * c[1])
        - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
    )


def inertia_properties(link):
    inertial = link.find("inertial")
    if inertial is None:
        return None
    mass = finite(inertial.find("mass").get("value"), "link mass")
    inertia = inertial.find("inertia")
    entries = {name: finite(inertia.get(name), f"inertia {name}")
               for name in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
    tensor = (
        (entries["ixx"], entries["ixy"], entries["ixz"]),
        (entries["ixy"], entries["iyy"], entries["iyz"]),
        (entries["ixz"], entries["iyz"], entries["izz"]),
    )
    return mass, tensor


def physical_inertia(tensor):
    require(tensor[0][0] > 0.0
            and tensor[0][0] * tensor[1][1] - tensor[0][1] ** 2 > 0.0
            and determinant(tensor) > 0.0, "inertia is not positive definite")
    half_trace = math.fsum(tensor[i][i] for i in range(3)) * 0.5
    covariance = tuple(tuple((half_trace if i == j else 0.0) - tensor[i][j]
                             for j in range(3)) for i in range(3))
    tolerance = max(half_trace, 1e-12) * 1e-12
    require(all(covariance[i][i] >= -tolerance for i in range(3))
            and all(covariance[i][i] * covariance[j][j] - covariance[i][j] ** 2
                    >= -(tolerance ** 2) for i, j in ((0, 1), (0, 2), (1, 2)))
            and determinant(covariance) >= -(tolerance ** 3),
            "inertia violates the principal-moment triangle inequalities")


def joint_limits(joint):
    limit = joint.find("limit")
    require(limit is not None, f"joint {joint.get('name')} has no finite limits")
    lower = finite(limit.get("lower"), "lower joint limit")
    upper = finite(limit.get("upper"), "upper joint limit")
    require(lower < upper, f"joint {joint.get('name')} has unordered joint limits")
    return lower, upper


def verify_structure(source, derived, spec):
    require(source.attrib == derived.attrib, "robot identity changed")
    original_links = named_elements(source, "link")
    derived_links = named_elements(derived, "link")
    require(original_links.keys() == derived_links.keys(), "link names changed")
    original_joints = named_elements(source, "joint")
    derived_joints = named_elements(derived, "joint")
    require(original_joints.keys() == derived_joints.keys(), "joint names changed")
    require(spec["marker_mass_kg"] == MARKER_MASS_KG
            and spec["marker_inertia_kg_m2"] == MARKER_INERTIA_KG_M2,
            "marker regularization contract changed")

    regularized = []
    masses = []
    positive_masses = []
    for name, original in original_links.items():
        candidate = derived_links[name]
        require(canonical(original, ("inertial",)) == canonical(candidate, ("inertial",)),
                f"visual, collision, or link geometry changed for {name}")
        original_properties = inertia_properties(original)
        candidate_properties = inertia_properties(candidate)
        require(candidate_properties is not None, f"link {name} has no derived inertia")
        mass, tensor = candidate_properties
        require(mass > 0.0, f"link {name} has non-positive derived mass")
        physical_inertia(tensor)
        masses.append(mass)
        if original_properties is not None and original_properties[0] > 0.0:
            require(canonical(original.find("inertial")) == canonical(candidate.find("inertial")),
                    f"positive declared inertial changed for {name}")
            positive_masses.append(original_properties[0])
        else:
            regularized.append(name)
            require(mass == MARKER_MASS_KG, f"marker mass changed for {name}")
            require(tensor == ((MARKER_INERTIA_KG_M2, 0.0, 0.0),
                               (0.0, MARKER_INERTIA_KG_M2, 0.0),
                               (0.0, 0.0, MARKER_INERTIA_KG_M2)),
                    f"marker inertia changed for {name}")
            marker_origin = candidate.find("inertial/origin")
            require(marker_origin is not None
                    and vector(marker_origin.get("xyz"), "marker COM") == (0.0, 0.0, 0.0)
                    and vector(marker_origin.get("rpy"), "marker inertia rotation") == (0.0, 0.0, 0.0),
                    f"marker COM changed for {name}")
    require(sorted(regularized) == sorted(spec["regularized_frames"]),
            "the marker list does not match the original model")
    require(len(spec["regularized_frames"]) == len(regularized), "duplicate regularized frames")
    total_mass = math.fsum(masses)
    require(abs(total_mass - finite(spec["declared_total_mass_kg"], "metadata mass")) <= 1e-12,
            "metadata total mass differs from the derived model")
    expected_mass = math.fsum(positive_masses) + len(regularized) * MARKER_MASS_KG
    require(abs(total_mass - expected_mass) <= 1e-12, "derived mass changed beyond frame regularization")

    offsets = {name: finite(value, "physical angle offset")
               for name, value in spec["physical_angle_offset_rad"].items()}
    require(offsets.keys() <= original_joints.keys(), "angle offset names an unknown joint")
    for name, original in original_joints.items():
        candidate = derived_joints[name]
        offset = offsets.get(name, 0.0)
        if original.get("type") == "fixed":
            require(offset == 0.0 and canonical(original) == canonical(candidate),
                    f"fixed joint changed for {name}")
            continue
        require(original.get("type") in ("revolute", "continuous"),
                f"unsupported coordinate recentering for {name}")
        lower, upper = joint_limits(original)
        candidate_lower, candidate_upper = joint_limits(candidate)
        require(lower <= offset <= upper and candidate_lower <= 0.0 <= candidate_upper,
                f"zero simulation angle is not a legal physical stance for {name}")
        require(abs(candidate_lower + offset - lower) <= KINEMATIC_TOLERANCE
                and abs(candidate_upper + offset - upper) <= KINEMATIC_TOLERANCE,
                f"physical joint limits changed for {name}")
        for key in ("effort", "velocity"):
            require(original.find("limit").get(key) == candidate.find("limit").get(key),
                    f"declared {key} changed for {name}")
        left, right = copy.deepcopy(original), copy.deepcopy(candidate)
        for joint in (left, right):
            joint.find("origin").attrib.pop("rpy", None)
            joint.find("limit").attrib.pop("lower", None)
            joint.find("limit").attrib.pop("upper", None)
        require(canonical(left) == canonical(right),
                f"joint topology, axis, origin translation, or dynamics changed for {name}")
        if offset == 0.0:
            require(canonical(original) == canonical(candidate), f"unshifted joint changed for {name}")

    if spec["source"].endswith("go2_description.rne.urdf"):
        for leg in ("FL", "FR", "RL", "RR"):
            name = f"{leg}_calf_joint"
            lower, upper = joint_limits(original_joints[name])
            require(not lower <= 0.0 <= upper, "original Go2 calf zero should be outside its limits")
            require(lower <= offsets.get(name, 0.0) <= upper,
                    f"initial-coordinate fix did not correct Go2 calf {name}")
    return total_mass, len(regularized), offsets


def verify_kinematics(source, derived, offsets):
    joints = named_elements(source, "joint")
    actuated = {name: joint for name, joint in joints.items() if joint.get("type") != "fixed"}
    nominal = {name: offsets.get(name, 0.0) for name in actuated}
    samples = [nominal]
    for fraction in (0.05, 0.2, 0.5, 0.8, 0.95):
        samples.append({name: lower + fraction * (upper - lower)
                        for name, joint in actuated.items()
                        for lower, upper in [joint_limits(joint)]})
    for name, joint in sorted(actuated.items()):
        lower, upper = joint_limits(joint)
        for fraction in (0.173, 0.827):
            sample = dict(nominal)
            sample[name] = lower + fraction * (upper - lower)
            samples.append(sample)
    maximum = 0.0
    for index, physical_angles in enumerate(samples):
        simulated_angles = {name: value - offsets.get(name, 0.0)
                            for name, value in physical_angles.items()}
        original_world = forward_kinematics(source, physical_angles)
        candidate_world = forward_kinematics(derived, simulated_angles)
        for name in sorted(original_world):
            residual = max(abs(original_world[name][i][j] - candidate_world[name][i][j])
                           for i in range(4) for j in range(4))
            maximum = max(maximum, residual)
            require(residual <= KINEMATIC_TOLERANCE,
                    f"FK differs for link {name} at sample {index}: {residual:.17g}")
    return len(samples), maximum


def verify_models(metadata):
    specs = metadata["models"]
    require(len(specs) == len(SOURCE_PINS)
            and {spec["source"] for spec in specs} == SOURCE_PINS.keys(),
            "metadata must describe both pinned Unitree source models exactly once")
    reports = []
    for spec in specs:
        source_path = REPO / spec["source"]
        derived_path = REPO / spec["derived"]
        source_hash = hashlib.sha256(source_path.read_bytes()).hexdigest()
        derived_hash = hashlib.sha256(derived_path.read_bytes()).hexdigest()
        require(source_hash == SOURCE_PINS[spec["source"]] == spec["source_sha256"],
                f"source provenance hash mismatch for {spec['source']}")
        require(derived_hash == spec["derived_sha256"],
                f"derived provenance hash mismatch for {spec['derived']}")
        source, derived = ET.parse(source_path).getroot(), ET.parse(derived_path).getroot()
        mass, marker_count, offsets = verify_structure(source, derived, spec)
        samples, maximum = verify_kinematics(source, derived, offsets)
        reports.append({
            "source": spec["source"], "source_sha256": source_hash,
            "derived": spec["derived"], "derived_sha256": derived_hash,
            "total_mass_kg": mass, "regularized_frame_count": marker_count,
            "fk_pose_samples": samples, "fk_links_per_sample": len(source.findall("link")),
            "max_fk_matrix_residual": maximum, "fk_tolerance": KINEMATIC_TOLERANCE,
            "positive_inertials_preserved": True, "geometry_preserved": True,
            "effort_and_speed_limits_preserved": True, "initial_joint_angles_legal": True,
        })
    return reports


def self_test(metadata):
    """Exercise independent FK and physical-contract rejection with broken fixtures."""
    spec = next(item for item in metadata["models"] if item["source"].endswith("go2_description.rne.urdf"))
    source = ET.parse(REPO / spec["source"]).getroot()
    derived = ET.parse(REPO / spec["derived"]).getroot()
    offsets = spec["physical_angle_offset_rad"]

    def rejects(function, message):
        try:
            function()
        except ValueError as error:
            require(message in str(error), f"unexpected self-test error: {error}")
        else:
            raise ValueError(f"broken fixture was accepted: {message}")

    changed_origin = copy.deepcopy(derived)
    named_elements(changed_origin, "joint")["FL_thigh_joint"].find("origin").set("rpy", "0 0.82 0")
    rejects(lambda: verify_kinematics(source, changed_origin, offsets), "FK differs")
    changed_mass = copy.deepcopy(derived)
    named_elements(changed_mass, "link")["base"].find("inertial/mass").set("value", "7.421")
    rejects(lambda: verify_structure(source, changed_mass, spec), "positive declared inertial changed")
    changed_marker = copy.deepcopy(derived)
    named_elements(changed_marker, "link")["imu"].find("inertial/mass").set("value", "1")
    rejects(lambda: verify_structure(source, changed_marker, spec), "marker mass changed")
    changed_limit = copy.deepcopy(derived)
    named_elements(changed_limit, "joint")["FL_calf_joint"].find("limit").set("lower", "-1.0")
    rejects(lambda: verify_structure(source, changed_limit, spec), "physical joint limits changed")
    changed_geometry = copy.deepcopy(derived)
    named_elements(changed_geometry, "link")["FL_foot"].find("collision/geometry/sphere").set("radius", "0.03")
    rejects(lambda: verify_structure(source, changed_geometry, spec), "geometry changed")
    return 5


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--metadata", type=Path, default=METADATA)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--self-test", action="store_true")
    options = parser.parse_args()
    metadata_bytes = options.metadata.read_bytes()
    metadata = json.loads(metadata_bytes)
    report = {
        "metadata_sha256": hashlib.sha256(metadata_bytes).hexdigest(),
        "verifier_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "generator_sha256": hashlib.sha256(Path(__file__).with_name("prepare_models.py").read_bytes()).hexdigest(),
        "models": verify_models(metadata),
    }
    if options.self_test:
        report["rejection_self_tests_passed"] = self_test(metadata)
    if options.report:
        options.report.parent.mkdir(parents=True, exist_ok=True)
        options.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print("kick model verification ok")


if __name__ == "__main__":
    main()
