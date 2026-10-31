#!/usr/bin/env python3
"""Label the rendered G1 sim2sim frames and encode the GIF with its measurements.

Usage: encode.py RENDER_DIR OUTPUT_STEM   (e.g. docs/media/g1-sim2sim)
"""
import argparse
import hashlib
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

SIZE = (960, 540)
PANEL_TOP = 50


def font(size, bold=False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(name, size)
    except OSError:
        return ImageFont.load_default(size=size)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("stem", type=Path)
    arguments = parser.parse_args()
    root, stem = arguments.directory, arguments.stem
    trace = json.loads((root / "render-trace.json").read_text())
    rne, mujoco = trace["rne"], trace["mujoco"]
    if rne["fell"] or mujoco["fell"]:
        raise ValueError("A rendered walk fell")
    if trace["frame_count"] != len(trace["frame_times_s"]):
        raise ValueError("Frame count does not match the recorded frame times")
    manifest = Path(__file__).resolve().parents[2] / "assets/policies/unitree_g1_walk/g1_walk_lstm.policy.json"
    if json.loads(manifest.read_text())["weights"]["sha256"] != trace["policy_weights_sha256"]:
        raise ValueError("The render used different policy weights")
    vx, vy, yaw = trace["command"]
    small, bold = font(13), font(18, True)
    title = "Unitree G1 | unitree_rl_gym walking policy, sim2sim into RNE"
    label = (f"command {vx:g} m/s, {yaw:g} rad/s | forward RNE "
             f"{rne['second_half_forward_speed_m_s']:.3f} vs MuJoCo "
             f"{mujoco['second_half_forward_speed_m_s']:.3f} m/s | yaw RNE "
             f"{rne['second_half_yaw_rate_rad_s']:.3f} vs MuJoCo "
             f"{mujoco['second_half_yaw_rate_rad_s']:.3f} rad/s")
    frames = []
    panel_size = tuple(trace["panel"])
    for index, time_s in enumerate(trace["frame_times_s"]):
        path = root / "frames" / f"frame-{index:03}.png"
        with Image.open(path) as raw:
            if raw.size != panel_size:
                raise ValueError(f"Unexpected frame size: {path}")
            picture = Image.new("RGB", SIZE, (17, 24, 35))
            picture.paste(raw.convert("RGB"), (0, PANEL_TOP))
        draw = ImageDraw.Draw(picture)
        draw.text((16, 6), title, font=bold, fill=(238, 244, 250))
        draw.text((16, 29), label, font=small, fill=(160, 185, 215))
        draw.rounded_rectangle((850, PANEL_TOP + 10, 944, PANEL_TOP + 40), 6, fill=(25, 45, 62))
        draw.text((863, PANEL_TOP + 15), f"{time_s:5.2f} s", font=small, fill="white")
        footer = PANEL_TOP + panel_size[1] + 8
        draw.rectangle((16, footer + 3, 28, footer + 15), fill=(51, 140, 255))
        draw.text((34, footer), "RNE pelvis path", font=small, fill=(159, 180, 202))
        draw.rectangle((170, footer + 3, 182, footer + 15), fill=(255, 140, 38))
        draw.text((188, footer), f"{trace['mujoco_engine']} pelvis path (same weights, same 500 Hz PD loop)",
                  font=small, fill=(159, 180, 202))
        frames.append(picture)
    poster = frames[-1]
    poster.save(stem.with_suffix(".png"))
    sheet = Image.new("RGB", (SIZE[0], SIZE[1] * 4))
    for row, index in enumerate([0, len(frames) // 3, 2 * len(frames) // 3, len(frames) - 1]):
        sheet.paste(frames[index], (0, row * SIZE[1]))
    palette = sheet.quantize(colors=256)
    encoded = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in frames[::2]]
    gif = stem.with_suffix(".gif")
    encoded[0].save(gif, save_all=True, append_images=encoded[1:], duration=80, loop=0, optimize=True)
    metadata = {
        "schema_version": 1,
        "scope": "unitree_rl_gym's pretrained G1 walking policy run on RNE's Rapier backend with the "
                 "upstream MuJoCo deployment loop; paths and speeds are measured outputs.",
        "command": trace["command"], "rne": rne, "mujoco": mujoco,
        "policy_weights_sha256": trace["policy_weights_sha256"],
        "artifacts": {"gif_sha256": digest(gif), "poster_sha256": digest(stem.with_suffix(".png")),
                      "frame_count": len(encoded), "width": SIZE[0], "height": SIZE[1],
                      "gif_duration_ms": 80 * len(encoded)},
        "trace_sha256": digest(root / "render-trace.json"),
        "provenance": ["assets/policies/unitree_g1_walk", "assets/robots/g1_description/UPSTREAM.md"],
    }
    stem.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata["artifacts"], indent=2))


if __name__ == "__main__":
    main()
