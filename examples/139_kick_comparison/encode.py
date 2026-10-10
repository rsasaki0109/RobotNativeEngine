#!/usr/bin/env python3
"""Label engine-rendered frames and encode a fixed-clock comparison GIF."""

import argparse
import hashlib
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(name, size)
    except OSError:
        return ImageFont.load_default(size=size)


def validate_compiled_sources(trace, repo):
    manifest = trace.get("compiled_source_sha256")
    required = {"examples/139_kick_comparison/" + name for name in
                ("main.rs", "physics.rs", "render.rs", "model.rs", "disturbance.rs", "go2_controller.rs",
                 "g1_controller.rs", "observation.rs", "models.json", "go2.rne.scene.toml", "g1.rne.scene.toml",
                 "go2.rne.robot.toml", "g1.rne.robot.toml")}
    models = json.loads((repo / "examples/139_kick_comparison/models.json").read_text())
    required.update(model["derived"] for model in models["models"])
    required.add("crates/rne_physics_rapier/src/backend.rs")
    required.update(("crates/rne_data/src/frame.rs", "crates/rne_data/src/bus.rs",
                     "crates/rne_data/src/stream.rs", "crates/rne_core/src/rng.rs",
                     "crates/rne_core/src/time.rs", "crates/rne_world/src/resources.rs"))
    required.add("assets/fixtures/kick_human/cc0_sport_human.glb")
    if not isinstance(manifest, dict) or not required.issubset(manifest):
        raise ValueError("Trace lacks the complete compiled source manifest")
    for name, value in manifest.items():
        relative = Path(name)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError("Compiled manifest entries must be repository-relative paths")
        if digest(repo / relative) != value:
            raise ValueError(f"Compiled capture source differs from the current file: {name}")
    return manifest


def finite_scalar(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return value


def same_scalar(actual, expected, label, abs_tol=1e-9):
    if not math.isclose(finite_scalar(actual, label), expected, rel_tol=0.0, abs_tol=abs_tol):
        raise ValueError(f"{label} differs from the declared disturbance")


def common_impulse(traces):
    if len(traces) != 2 or [trace["robot"] for trace in traces] != ["Go2", "G1"]:
        raise ValueError("Expected the ordered Go2/G1 recordings")
    impulse_ns = finite_scalar(traces[0]["impulse_n_s"], "Common impulse")
    if impulse_ns <= 0.0:
        raise ValueError("The common disturbance impulse must be positive")
    for trace in traces:
        same_scalar(trace["impulse_n_s"], impulse_ns, "Recording impulse")
    return impulse_ns


def measured_disturbance(trace, impulse_ns, direction, dt_s=0.001, start_s=2.0):
    """Integrate each recorded force and verify the declared sampled pulse."""
    same_scalar(trace["impulse_n_s"], impulse_ns, "Trace impulse")
    same_scalar(trace["dt_s"], dt_s, "Physics time step", abs_tol=1e-12)
    same_scalar(trace["push_duration_s"], 0.08, "Pulse duration", abs_tol=1e-12)
    same_scalar(trace["push_start_s"], start_s, "Pulse onset", abs_tol=1e-12)
    history = trace["force_history"]
    if not history:
        raise ValueError("Force history must be present")
    pulse_steps = round(0.08 / dt_s)
    start_step = round(start_s / dt_s) + 1
    weights = [math.sin(math.pi * (index + 0.5) / pulse_steps)
               for index in range(pulse_steps)]
    normalization = math.fsum(weights) * dt_s
    forces = []
    for step, row in enumerate(history, 1):
        same_scalar(row["dt_s"], dt_s, "Force-history time step", abs_tol=1e-12)
        if isinstance(row["step"], bool) or row["step"] != step:
            raise ValueError("Force history must contain consecutive simulation steps")
        same_scalar(row["time_s"], step * dt_s, "Force-history clock", abs_tol=1e-12)
        force = row["force_world_n"]
        if not isinstance(force, list) or len(force) != 3:
            raise ValueError("World force must contain three finite components")
        offset = step - start_step
        magnitude = (impulse_ns * weights[offset] / normalization
                     if 0 <= offset < pulse_steps else 0.0)
        for actual, axis in zip(force, direction):
            same_scalar(actual, magnitude * axis, "Sampled half-sine force")
        forces.append(force)
    if len(history) < start_step + pulse_steps - 1:
        raise ValueError("Force history must contain the complete pulse")
    integrated = [math.fsum(force[axis] * dt_s for force in forces) for axis in range(3)]
    recorded = trace["integrated_impulse_world_ns"]
    if not isinstance(recorded, list) or len(recorded) != 3:
        raise ValueError("Recorded impulse must contain three components")
    for actual, declaration, axis in zip(integrated, recorded, direction):
        same_scalar(declaration, actual, "Recorded integrated impulse")
        same_scalar(actual, impulse_ns * axis, "Measured impulse direction and magnitude")
    peak_n = max(math.sqrt(math.fsum(v * v for v in force)) for force in forces)
    same_scalar(trace["force_n"], peak_n, "Recorded force peak")
    return {"robot": trace["robot"], "integrated_impulse_world_ns": integrated,
            "impulse_n_s": math.sqrt(math.fsum(v * v for v in integrated)),
            "measured_peak_force_n": peak_n}


def validate_ideal_observation_pipeline(trace):
    pipeline = trace.get("observation_pipeline")
    period = round(finite_scalar(trace["dt_s"], "Physics time step") * 1_000_000_000)
    if (not isinstance(pipeline, dict) or pipeline.get("profile") != "ideal_reference"
            or any(type(pipeline.get(key)) is not int for key in
                   ("sample_period_ticks", "latency_ticks", "phase_offset_ticks"))
            or pipeline.get("sample_period_ticks") != period
            or pipeline.get("latency_ticks") != 0 or pipeline.get("phase_offset_ticks") != 0):
        raise ValueError("Public GIF qualification requires ideal observations at each physics tick")
    noise = pipeline.get("noise")
    keys = ("attitude_world_x_z_bound_rad", "angular_velocity_axis_bound_rad_s",
            "com_and_foot_position_axis_bound_m", "com_velocity_axis_bound_m_s",
            "go2_foot_normal_load_bound_n")
    if not isinstance(noise, dict) or any(finite_scalar(noise.get(key), key) != 0 for key in keys):
        raise ValueError("Public GIF qualification requires zero synthetic estimate error")
    state_hash = trace.get("observation_control_state_hash")
    if (trace.get("observation_control_state_bits_exact_repeat") is not True
            or not isinstance(state_hash, str) or len(state_hash) != 16
            or any(character not in "0123456789abcdef" for character in state_hash)):
        raise ValueError("Observation/controller exact replay evidence is missing or invalid")


def validate_recordings(traces, summary):
    impulse_ns = common_impulse(traces)
    repo = Path(__file__).resolve().parents[2]
    manifests = [validate_compiled_sources(trace, repo) for trace in traces]
    if manifests[0] != manifests[1]:
        raise ValueError("Capture recordings use different compiled sources")
    if summary.get("schema_version") != 2 or len(summary["cases"]) != 2:
        raise ValueError("Expected schema-2 recovery summary")
    measurements = []
    for trace, case in zip(traces, summary["cases"]):
        if trace.get("schema_version") != 2 or trace["robot"] != case["robot"]:
            raise ValueError("Trace and summary identities must match")
        validate_ideal_observation_pipeline(trace)
        for key in ("observation_pipeline", "observation_control_state_hash",
                    "observation_control_state_bits_exact_repeat"):
            if trace[key] != case.get(key):
                raise ValueError("Capture observation/controller evidence differs from its summary")
        measurement = measured_disturbance(trace, impulse_ns, [0.0, 0.0, 1.0])
        same_scalar(case["impulse_n_s"], impulse_ns, "Summary impulse")
        same_scalar(case["dt_s"], trace["dt_s"], "Summary time step")
        same_scalar(case["duration_s"], trace["push_duration_s"], "Summary pulse duration")
        same_scalar(case["force_n"], measurement["measured_peak_force_n"], "Summary force peak")
        if trace["robot"] == "Go2" and trace["controller"].get("contact_load_regulation_enabled") is not True:
            raise ValueError("Go2 nominal recording must use the contact-load regulation controller")
        if trace["controller"] != case["controller"] or trace["plant"] != case["plant"]:
            raise ValueError("Capture controller/model configuration differs from its summary")
        if trace["summary"] != case["summary"]:
            raise ValueError("Trace and summary recovery metrics differ")
        if trace["observed_state_hash"] != case["observed_state_hash"]:
            raise ValueError("Trace and summary observable state hashes differ")
        if (trace["summary"]["recovered"] is not True
                or case["observed_state_bits_exact_repeat"] is not True):
            raise ValueError("Recovery and deterministic replay must pass before encoding")
        if (trace.get("observed_state_bits_exact_repeat") is not True
                or trace.get("canonical_render_frames_exact_repeat") is not True
                or case.get("canonical_render_frames_exact_repeat") is not True):
            raise ValueError("Trace state and render replay evidence must be present")
        measurements.append(measurement)
    count = len(traces[0]["frames"])
    if count != 187 or len(traces[1]["frames"]) != count:
        raise ValueError("Expected two synchronized 187-frame traces")
    for index in range(count):
        times = [finite_scalar(trace["frames"][index]["time_s"], "Frame clock")
                 - finite_scalar(trace["frames"][0]["time_s"], "Initial frame clock")
                 for trace in traces]
        if abs(times[0] - times[1]) > 0.001 + 1e-12:
            raise ValueError("Trace clocks are not synchronized within one simulation step")
        if any(abs(t - index / 30.0) > 0.001 + 1e-12 for t in times):
            raise ValueError("Trace frame clock differs from 30 Hz")
    return measurements


def validation_metadata(path, nominal_traces):
    if path is None:
        raise ValueError("Public GIF encoding requires a complete two-plant validation report")
    impulse_ns = common_impulse(nominal_traces)
    report = json.loads(path.read_text())
    robots = report.get("robots")
    if robots != ["Go2", "G1"]:
        raise ValueError("Public GIF validation requires the full Go2/G1 report; filtered scope is incomplete")
    required = {"nominal", "zero", "reverse_early", "late", "rate_500hz", "balance_off_zero"}
    names = required | {"balance_off", "front_limit", "oblique_limit"}
    expected = {(robot, name) for robot in robots for name in names}
    seen = set()
    cases = []
    trace_hashes = {}
    for case in report["cases"]:
        identity = (case["robot"], case["case"])
        if identity not in expected or identity in seen:
            raise ValueError("Validation contains an unknown or duplicate case")
        seen.add(identity)
        if case["required_recovery"] is not (case["case"] in required):
            raise ValueError("Validation required-case flag differs from the declared envelope")
        if case["observed_state_bits_exact_repeat"] is not True:
            raise ValueError("Validation replay did not pass")
        if not isinstance(case["summary"]["recovered"], bool):
            raise ValueError("Validation recovery result must be a boolean")
        trace_path = path.parent / f"{case['robot'].lower()}-{case['case']}.json"
        trace = json.loads(trace_path.read_text())
        validate_ideal_observation_pipeline(trace)
        for key in ("robot", "case", "dt_s", "impulse_n_s", "summary", "observed_state_hash",
                    "observed_state_bits_exact_repeat", "controller", "plant", "push_start_s",
                    "push_duration_s", "integrated_impulse_world_ns", "compiled_source_sha256",
                    "observation_pipeline", "observation_control_state_hash",
                    "observation_control_state_bits_exact_repeat"):
            if trace[key] != case[key]:
                raise ValueError("Validation report differs from its per-case trace")
        manifest = validate_compiled_sources(trace, Path(__file__).resolve().parents[2])
        if any(manifest != nominal["compiled_source_sha256"] for nominal in nominal_traces):
            raise ValueError("Validation and capture use different compiled sources")
        if trace.get("schema_version") != 2:
            raise ValueError("Validation requires schema-2 per-case traces")
        name = case["case"]
        expected_impulse_ns = 0.0 if name in {"zero", "balance_off_zero"} else impulse_ns
        directions = {"reverse_early": [0.0, 0.0, -1.0],
                      "front_limit": [1.0, 0.0, 0.0],
                      "oblique_limit": [math.sqrt(0.5), 0.0, math.sqrt(0.5)]}
        measured_disturbance(trace, expected_impulse_ns,
                             directions.get(name, [0.0, 0.0, 1.0]),
                             dt_s=0.002 if name == "rate_500hz" else 0.001,
                             start_s={"reverse_early": 1.5, "late": 2.25}.get(name, 2.0))
        if case["case"] == "nominal":
            nominal = next(t for t in nominal_traces if t["robot"] == case["robot"])
            for key in ("controller", "plant", "summary", "observed_state_hash", "observation_pipeline",
                        "observation_control_state_hash", "observation_control_state_bits_exact_repeat"):
                if trace[key] != nominal[key]:
                    raise ValueError("Validation nominal differs from the GIF capture")
        trace_hashes[trace_path.name] = digest(trace_path)
        cases.append({key: case[key] for key in
                      ("robot", "case", "required_recovery", "dt_s", "impulse_n_s",
                       "summary", "observed_state_hash", "observed_state_bits_exact_repeat",
                       "controller", "plant", "push_start_s", "push_duration_s",
                       "integrated_impulse_world_ns", "observation_pipeline", "observation_control_state_hash",
                       "observation_control_state_bits_exact_repeat")})
    if seen != expected:
        raise ValueError("Full validation requires exactly nine cases per robot")
    passed = all(not c["required_recovery"] or c["summary"]["recovered"] for c in cases)
    if report["required_cases_passed"] is not passed:
        raise ValueError("Validation aggregate disagrees with computed required-case results")
    if not passed:
        raise ValueError("A required recovery case failed; public GIF validation is incomplete")
    return {"report_sha256": digest(path), "robots": robots, "case_trace_sha256": trace_hashes,
            "required_cases_passed": passed,
            "trajectory_convergence_claimed": report["trajectory_convergence_claimed"],
            "cases": cases,
            "negative_cases": [case for case in cases if not case["summary"]["recovered"]]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--validation", type=Path, required=True,
                        help="Required complete Go2/G1 18-case recovery-validation.json report")
    arguments = parser.parse_args()
    root = arguments.directory
    traces = [json.loads((root / f"{name}-trace.json").read_text()) for name in ("go2", "g1")]
    summary = json.loads((root / "summary.json").read_text())
    measurements = validate_recordings(traces, summary)
    validation = validation_metadata(arguments.validation, traces)
    count = len(traces[0]["frames"])
    frames = []
    small, bold = font(13), font(19, True)
    kick_seen = False
    for index in range(count):
        elapsed_times = [trace["frames"][index]["time_s"] - trace["frames"][0]["time_s"]
                         for trace in traces]
        if abs(elapsed_times[0] - elapsed_times[1]) > 0.001 + 1e-12:
            raise ValueError("Trace clocks are not synchronized")
        path = root / "frames" / f"frame-{index:03}.png"
        with Image.open(path) as raw:
            if raw.size != (960, 420):
                raise ValueError(f"Unexpected frame size: {path}")
            picture = Image.new("RGB", (960, 540), (17, 24, 35))
            picture.paste(raw.convert("RGB"), (0, 55))
        draw = ImageDraw.Draw(picture)
        for side, (name, label) in enumerate([
            ("Unitree Go2", f"{measurements[0]['impulse_n_s']:g} N.s / 80 ms | peak {measurements[0]['measured_peak_force_n']:.0f} N | recovered"),
            ("Unitree G1", f"{measurements[1]['impulse_n_s']:g} N.s / 80 ms | peak {measurements[1]['measured_peak_force_n']:.0f} N | recovered"),
        ]):
            x = side * 480 + 16
            draw.text((x, 7), name, font=bold, fill=(238, 244, 250))
            draw.text((x, 31), label, font=small, fill=(160, 185, 215))
        elapsed = traces[0]["frames"][index]["time_s"] - traces[0]["frames"][0]["time_s"]
        active = any(trace["frames"][index]["force_n"] > 0 for trace in traces)
        kick_seen = kick_seen or active
        phase = "KICK" if active else ("RECOVERY" if kick_seen else "READY")
        draw.rounded_rectangle((395, 64, 565, 96), 6,
                               fill=(196, 79, 37) if active else (25, 45, 62))
        draw.text((408, 69), f"{phase}  {elapsed:.2f}s", font=small, fill="white")
        draw.line((480, 0, 480, 540), fill=(52, 67, 83), width=2)
        draw.text((16, 487), "RNE / Rapier | Common normalized half-sine impulse | Lateral recovery only",
                  font=small, fill=(159, 180, 202))
        draw.text((16, 510), "Human: animated visual only | Declared URDF inertials | Torque-bounded feedback",
                  font=small, fill=(159, 180, 202))
        if index == 63:
            picture.save(root / "kick-comparison.png")
        frames.append(picture)

    palette_sheet = Image.new("RGB", (960, 540 * 5))
    for row, index in enumerate([0, 48, 63, 90, count - 1]):
        palette_sheet.paste(frames[index], (0, row * 540))
    palette = palette_sheet.quantize(colors=256)
    encoded = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]
    durations_ms = [40 if index % 3 == 2 else 30 for index in range(count)]
    gif = root / "kick-comparison.gif"
    encoded[0].save(gif, save_all=True, append_images=encoded[1:],
                    duration=durations_ms, loop=0, optimize=True)
    repo = Path(__file__).resolve().parents[2]
    sources = [str(path.relative_to(repo)) for path in
               sorted((repo / "examples/139_kick_comparison").iterdir())
               if path.is_file() and path.suffix in (".rs", ".toml")]
    sources += ["examples/139_kick_comparison/encode.py",
                "examples/139_kick_comparison/prepare_models.py",
                "examples/139_kick_comparison/verify_models.py",
                "examples/139_kick_comparison/models.json",
                "crates/rne_physics_rapier/src/backend.rs",
                "crates/rne_render/src/animation.rs"]
    models = json.loads((repo / "examples/139_kick_comparison/models.json").read_text())
    for model in models["models"]:
        for field in ("source", "derived"):
            if digest(repo / model[field]) != model[field + "_sha256"]:
                raise ValueError("Model provenance hash differs from the actual URDF")
            sources.append(model[field])
    human = repo / "assets/fixtures/kick_human"
    sources += [str(path.relative_to(repo)) for path in sorted(human.rglob("*"))
                if path.is_file() and path.suffix in (".py", ".json", ".glb", ".md")]
    metadata = {
        "schema_version": 2,
        "scope": f"Common measured {measurements[0]['impulse_n_s']:g} N.s lateral prescribed body wrench, normalized half-sine over 80 ms; declared URDF inertials and torque-bounded feedback. Human animation is visual only; no human contact, all-direction recovery, or hardware qualification claim.",
        "compiled_source_sha256": traces[0]["compiled_source_sha256"],
        "measured_disturbances": measurements,
        "model_provenance": models,
        "recovery_validation": validation,
        "physics": summary,
        "artifacts": {
            "gif_sha256": digest(gif),
            "poster_sha256": digest(root / "kick-comparison.png"),
            "frame_count": count,
            "width": 960,
            "height": 540,
            "gif_duration_ms": sum(durations_ms),
        },
        "source_hash_scope": "SHA256 of repository files at encoding time; not proof of the capture executable or its build sources. Compiled source manifest entries are independently checked against repository files. Nominal controller/model configuration and observable state are checked against the required full validation traces.",
        "source_sha256": {name: digest(repo / name) for name in sources},
        "trace_sha256": {name: digest(root / name) for name in ("go2-trace.json", "g1-trace.json")},
        "human_provenance": "assets/fixtures/kick_human/source-manifest.json",
        "human_license": "assets/fixtures/kick_human/LICENSE.CC0.md",
        "robot_provenance": ["assets/robots/go2_description", "assets/robots/g1_description"],
    }
    (root / "kick-comparison.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata["artifacts"], indent=2))


if __name__ == "__main__":
    main()
