#!/usr/bin/env python3
"""Label engine-rendered frames and encode a fixed-clock comparison GIF."""

import argparse
import hashlib
import json
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    arguments = parser.parse_args()
    root = arguments.directory
    traces = [json.loads((root / f"{name}-trace.json").read_text()) for name in ("go2", "g1")]
    summary = json.loads((root / "summary.json").read_text())
    for trace, case, expected_force_n in zip(traces, summary["cases"], [120.0, 50.0]):
        if trace["force_n"] != expected_force_n or case["force_n"] != expected_force_n:
            raise ValueError("Force labels must match the measured traces")
        if not case["summary"]["recovered"] or not case["observed_state_bits_exact_repeat"]:
            raise ValueError("Recovery and deterministic replay must pass before encoding")
    count = len(traces[0]["frames"])
    if count != 187 or len(traces[1]["frames"]) != count:
        raise ValueError("Expected two synchronized 187-frame traces")
    frames = []
    small, bold = font(13), font(19, True)
    kick_seen = False
    for index in range(count):
        elapsed_times = [trace["frames"][index]["time_s"] - trace["frames"][0]["time_s"]
                         for trace in traces]
        if abs(elapsed_times[0] - elapsed_times[1]) > 1e-9:
            raise ValueError("Trace clocks are not synchronized")
        path = root / "frames" / f"frame-{index:03}.png"
        with Image.open(path) as raw:
            if raw.size != (960, 420):
                raise ValueError(f"Unexpected frame size: {path}")
            picture = Image.new("RGB", (960, 540), (17, 24, 35))
            picture.paste(raw.convert("RGB"), (0, 55))
        draw = ImageDraw.Draw(picture)
        for side, (name, label) in enumerate([
            ("Unitree Go2", "120 N x 0.20 s = 24 N.s | recovered"),
            ("Unitree G1", "50 N x 0.20 s = 10 N.s | recovered"),
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
        draw.text((16, 487), "RNE / Rapier | Prescribed disturbances | Different impulses per robot",
                  font=small, fill=(159, 180, 202))
        draw.text((16, 510), "Human: animated mesh | Existing approximate mass/inertia models",
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
    sources = [
        "examples/139_kick_comparison/main.rs",
        "examples/139_kick_comparison/physics.rs",
        "examples/139_kick_comparison/render.rs",
        "examples/139_kick_comparison/encode.py",
        "crates/rne_render/src/animation.rs",
        "assets/fixtures/kick_human/cc0_sport_human.glb",
    ]
    metadata = {
        "schema_version": 1,
        "scope": "Prescribed body wrenches with a visual human actor; different impulses; approximate scene mass/inertia",
        "physics": summary,
        "artifacts": {
            "gif_sha256": digest(gif),
            "poster_sha256": digest(root / "kick-comparison.png"),
            "frame_count": count,
            "width": 960,
            "height": 540,
            "gif_duration_ms": sum(durations_ms),
        },
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
