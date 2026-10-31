#!/usr/bin/env python3
"""Label the physical-kick frames and encode the Go2 GIF with its measurements."""

import argparse
import hashlib
import json
from pathlib import Path

from PIL import Image, ImageDraw

from encode import font

FRAME_COUNT = 187
SIZE = (960, 540)
PANEL = (960, 420)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_sources(trace, repo):
    """The capture must come from the current kicking-leg sources."""
    manifest = trace.get("physical_kick_source_sha256")
    required = {"examples/139_kick_comparison/kicker.rs",
                "examples/139_kick_comparison/physics/physical_kick.rs"}
    if not isinstance(manifest, dict) or set(manifest) != required:
        raise ValueError("Trace lacks the physical-kick source manifest")
    for name, value in manifest.items():
        if digest(repo / name) != value:
            raise ValueError(f"Physical-kick source differs from the capture: {name}")


def measured(trace):
    """Recorded contact outcome; nothing here is a declared input."""
    kick = trace["kick"]["measured"]
    if not kick["contact_occurred"] or kick["shoe_contact_steps"] < 1:
        raise ValueError("The simulated shoe never reached the robot")
    if trace.get("exact_replay") != {"robot_state_words": True, "kick_leg_state_words": True,
                                     "controller_state_words": True, "frames_and_contacts": True}:
        raise ValueError("The capture lacks its exact replay record")
    if len(trace["frames"]) != FRAME_COUNT:
        raise ValueError("Unexpected physical frame count")
    return kick


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--stem", default="kick-physical-go2",
                        help="Output file stem for the GIF, poster and metadata")
    arguments = parser.parse_args()
    root, stem = arguments.directory, arguments.stem
    repo = Path(__file__).resolve().parents[2]
    trace = json.loads((root / "go2-physical-trace.json").read_text())
    validate_sources(trace, repo)
    kick = measured(trace)
    recovered = trace["summary"]["recovered"]
    small, bold = font(13), font(19, True)
    settings = trace["kick"]
    strength = settings["strength_scale"]
    friction = settings["floor_and_foot_friction"]
    title = "Unitree Go2 | simulated kicking leg" + (
        f" | {strength:g}x adult strength" if strength != 1.0 else "")
    label = (f"shoe {kick['shoe_speed_before_first_contact_m_s']:.1f} m/s | "
             f"impulse {kick['normal_impulse_magnitude_ns']:.1f} N.s | "
             f"peak {kick['peak_normal_force_10ms_average_n']:.0f} N (10 ms avg) | "
             f"contact {1000 * kick['contact_time_s']:.0f} ms | "
             + ("recovered" if recovered else "did not recover"))
    first_contact_s = kick["first_contact_time_s"]
    last_contact_s = kick["last_contact_time_s"]
    frames = []
    for index, record in enumerate(trace["frames"]):
        path = root / "frames-physical" / f"frame-{index:03}.png"
        with Image.open(path) as raw:
            if raw.size != PANEL:
                raise ValueError(f"Unexpected frame size: {path}")
            picture = Image.new("RGB", SIZE, (17, 24, 35))
            picture.paste(raw.convert("RGB"), (0, 55))
        draw = ImageDraw.Draw(picture)
        draw.text((16, 7), title, font=bold, fill=(238, 244, 250))
        draw.text((16, 31), label, font=small, fill=(160, 185, 215))
        time_s = record["time_s"]
        touching = any(abs(component) > 0 for component in record["force_world_n"])
        phase = ("CONTACT" if touching else
                 "RECOVERY" if time_s > last_contact_s else
                 "KICK" if time_s > first_contact_s - 0.6 else "READY")
        draw.rounded_rectangle((770, 64, 944, 96), 6,
                               fill=(196, 79, 37) if touching else (25, 45, 62))
        draw.text((783, 69), f"{phase}  {time_s:.2f}s", font=small, fill="white")
        draw.text((16, 487), "RNE / Rapier | The leg's shoe hits the robot through physics contact; "
                  "impulse and force are measured",
                  font=small, fill=(159, 180, 202))
        draw.text((16, 510), "Leg: Dempster/Winter segments, torque-limited hip/knee/ankle | "
                  f"floor and foot friction {friction:g} | kicker torso follows the animation",
                  font=small, fill=(159, 180, 202))
        frames.append(picture)
    poster_index = next(i for i, record in enumerate(trace["frames"])
                        if any(abs(c) > 0 for c in record["force_world_n"]))
    frames[poster_index].save(root / f"{stem}.png")
    palette_sheet = Image.new("RGB", (SIZE[0], SIZE[1] * 5))
    for row, index in enumerate([0, 48, poster_index, 90, len(frames) - 1]):
        palette_sheet.paste(frames[index], (0, row * SIZE[1]))
    palette = palette_sheet.quantize(colors=256)
    encoded = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames]
    durations_ms = [40 if index % 3 == 2 else 30 for index in range(len(frames))]
    gif = root / f"{stem}.gif"
    encoded[0].save(gif, save_all=True, append_images=encoded[1:],
                    duration=durations_ms, loop=0, optimize=True)
    metadata = {
        "schema_version": 1,
        "scope": "Go2 recovery from a simulated kicking leg; contact impulse and force are "
                 "measured outputs. Rigid shoe contact, kinematic kicker torso, ideal robot "
                 "observations; no hardware qualification claim.",
        "kick": trace["kick"],
        "summary": trace["summary"],
        "exact_replay": trace["exact_replay"],
        "physical_kick_source_sha256": trace["physical_kick_source_sha256"],
        "artifacts": {"gif_sha256": digest(gif),
                      "poster_sha256": digest(root / f"{stem}.png"),
                      "frame_count": len(frames), "width": SIZE[0], "height": SIZE[1],
                      "gif_duration_ms": sum(durations_ms)},
        "trace_sha256": digest(root / "go2-physical-trace.json"),
        "human_provenance": "assets/fixtures/kick_human/source-manifest.json",
        "robot_provenance": ["assets/robots/go2_description"],
    }
    (root / f"{stem}.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata["artifacts"], indent=2))


if __name__ == "__main__":
    main()
