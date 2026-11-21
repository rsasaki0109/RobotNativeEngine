#!/usr/bin/env python3
"""Adds a HUD to example 149's RNE-rendered 3D frames and encodes a GIF.

    python3 examples/149_gnss_lidar_navigation/encode.py target/gnss3d docs/media/gnss-lidar-navigation-3d.gif

`target/gnss3d` must hold `trace.json` (from `149_gnss_lidar_navigation
--trace`) and `frames/` (from `149_gnss_lidar_navigation_render`). The HUD
shows the localization source, the GNSS solution, the error, the satellite
counts, and a sky plot at the robot. Needs Pillow and ffmpeg.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

SOURCES = {
    "gnss": ("GNSS", (26, 152, 80)),
    "lidar": ("LiDAR (AMCL)", (44, 127, 184)),
    "dead_reckoning": ("dead reckoning", (244, 109, 67)),
}
SOLUTIONS = {
    "fixed": ("RTK FIX", (26, 152, 80)),
    "float": ("RTK FLOAT", (230, 150, 40)),
    "single": ("SINGLE", (215, 48, 31)),
    "none": ("NO FIX", (115, 115, 115)),
}
PATHS = {
    "los": (40, 200, 90),
    "nlos": (230, 60, 40),
    "diffracted": (150, 120, 230),
    "blocked": (120, 120, 120),
    "below_mask": (120, 120, 120),
}


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    names = ["DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"]
    for name in names:
        for directory in ["/usr/share/fonts/truetype/dejavu", "/usr/share/fonts/TTF", "/Library/Fonts"]:
            path = Path(directory) / name
            if path.exists():
                return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def badge(draw: ImageDraw.ImageDraw, x: int, y: int, text: str, color, typeface) -> int:
    width = int(draw.textlength(text, font=typeface)) + 18
    draw.rounded_rectangle([x, y, x + width, y + 26], radius=5, fill=color)
    draw.text((x + 9, y + 4), text, fill=(255, 255, 255), font=typeface)
    return x + width + 8


def sky_plot(draw: ImageDraw.ImageDraw, cx: int, cy: int, radius: int, satellites, typeface) -> None:
    draw.ellipse([cx - radius - 6, cy - radius - 6, cx + radius + 6, cy + radius + 18], fill=(255, 255, 255, 215))
    draw.ellipse([cx - radius, cy - radius, cx + radius, cy + radius], outline=(90, 90, 90), width=2, fill=(240, 244, 250, 230))
    draw.ellipse([cx - radius // 3, cy - radius // 3, cx + radius // 3, cy + radius // 3], outline=(170, 170, 170))
    draw.text((cx - 4, cy - radius - 2), "N", fill=(40, 40, 40), font=typeface, anchor="ms")
    for sat in satellites:
        r = radius * (1.0 - sat["elevation_rad"] / (math.pi / 2))
        x = cx + r * math.sin(sat["azimuth_rad"])
        y = cy - r * math.cos(sat["azimuth_rad"])
        color = PATHS.get(sat["path"], (120, 120, 120))
        draw.ellipse([x - 5, y - 5, x + 5, y + 5], fill=color, outline=(30, 30, 30))
    draw.text((cx, cy + radius + 14), "sky at the robot", fill=(40, 40, 40), font=typeface, anchor="ms")


def compose(frame_png: Path, frame: dict, out: Path) -> None:
    image = Image.open(frame_png).convert("RGBA")
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    width, height = image.size
    big, small, tiny = font(17, True), font(14), font(12)
    draw.rectangle([0, 0, width, 40], fill=(20, 22, 26, 200))
    draw.text((12, 10), f"t = {frame['t']:5.1f} s", fill=(255, 255, 255), font=big)
    x = 120
    name, color = SOURCES[frame["source"]]
    x = badge(draw, x, 7, f"localizing: {name}", color, big)
    name, color = SOLUTIONS[frame["solution"]]
    x = badge(draw, x, 7, name, color, big)
    counts = {key: sum(1 for s in frame["satellites"] if s["path"] == key) for key in PATHS}
    text = (
        f"error {frame['error_m']:.2f} m   heading {'fixed' if frame['heading_fixed'] else '—'}   "
        f"LOS {counts['los']}  NLOS {counts['nlos']}  diffracted {counts['diffracted']}"
    )
    draw.text((x + 4, 12), text, fill=(235, 235, 235), font=small)
    legend = [("line of sight", PATHS["los"]), ("NLOS reflection", PATHS["nlos"]), ("diffraction", PATHS["diffracted"]),
              ("LiDAR", (30, 200, 255)), ("estimate", SOURCES[frame["source"]][1])]
    lx = 10
    draw.rectangle([0, height - 26, width, height], fill=(20, 22, 26, 170))
    for label, color in legend:
        draw.rectangle([lx, height - 17, lx + 16, height - 11], fill=color)
        draw.text((lx + 22, height - 21), label, fill=(240, 240, 240), font=tiny)
        lx += 34 + int(draw.textlength(label, font=tiny))
    draw.text((width - 10, height - 21), "Robot Native Engine · ray-traced GNSS + libgnss++ RTK", fill=(200, 200, 200), font=tiny, anchor="ra")
    sky_plot(draw, width - 82, 116, 58, frame["satellites"], tiny)
    Image.alpha_composite(image, overlay).convert("RGB").save(out)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("capture", type=Path, help="directory with trace.json and frames/")
    parser.add_argument("output", type=Path, help="GIF to write")
    parser.add_argument("--stride", type=int, default=4, help="use every Nth rendered frame")
    parser.add_argument("--fps", type=float, default=8.0)
    parser.add_argument("--width", type=int, default=600)
    parser.add_argument("--colors", type=int, default=80)
    args = parser.parse_args()

    trace = json.loads((args.capture / "trace.json").read_text())
    index = json.loads((args.capture / "frames" / "frames.json").read_text())
    if shutil.which("ffmpeg") is None:
        raise SystemExit("ffmpeg is required")
    render_stride = index["stride"]
    with tempfile.TemporaryDirectory() as scratch:
        scratch_dir = Path(scratch)
        entries = index["frames"][:: args.stride]
        for n, entry in enumerate(entries):
            frame = trace["frames"][index["frames"].index(entry) * render_stride]
            compose(args.capture / "frames" / entry["file"], frame, scratch_dir / f"hud_{n:04d}.png")
        last = scratch_dir / f"hud_{len(entries) - 1:04d}.png"
        for extra in range(int(args.fps * 2)):
            shutil.copy(last, scratch_dir / f"hud_{len(entries) + extra:04d}.png")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            ["ffmpeg", "-v", "error", "-y", "-framerate", str(args.fps), "-i", str(scratch_dir / "hud_%04d.png"),
             "-vf", f"scale={args.width}:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors={args.colors}:stats_mode=diff[p];"
                    "[b][p]paletteuse=dither=none:diff_mode=rectangle",
             str(args.output)],
            check=True,
        )
    print(f"wrote {args.output} ({args.output.stat().st_size} bytes, {len(entries)} frames)")


if __name__ == "__main__":
    main()
