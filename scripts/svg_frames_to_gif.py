#!/usr/bin/env python3
"""Rasterizes a directory of SVG frames (frame_0000.svg, ...) into a GIF.

Used for example 149:

    cargo run --release -p gnss_lidar_navigation --example 149_gnss_lidar_navigation -- --frames target/gnss_frames
    python3 scripts/svg_frames_to_gif.py target/gnss_frames docs/media/gnss-lidar-navigation.gif

Needs Python Playwright with a Chromium build and ffmpeg. Set
PLAYWRIGHT_CHROMIUM to a Chromium binary to skip Playwright's own download.
"""

import argparse
import os
import shutil
import subprocess
import tempfile
from pathlib import Path

from playwright.sync_api import sync_playwright


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("frames", type=Path, help="directory of frame_NNNN.svg files")
    parser.add_argument("output", type=Path, help="GIF to write")
    parser.add_argument("--fps", type=float, default=8.0, help="frames per second")
    parser.add_argument("--width", type=int, default=800, help="output width in pixels")
    parser.add_argument("--hold", type=int, default=12, help="extra copies of the last frame")
    args = parser.parse_args()

    frames = sorted(args.frames.glob("frame_*.svg"))
    if not frames:
        raise SystemExit(f"no frame_*.svg files in {args.frames}")
    if shutil.which("ffmpeg") is None:
        raise SystemExit("ffmpeg is required")

    with tempfile.TemporaryDirectory() as scratch:
        scratch_dir = Path(scratch)
        launch = {"args": ["--no-sandbox"]}
        if os.environ.get("PLAYWRIGHT_CHROMIUM"):
            launch["executable_path"] = os.environ["PLAYWRIGHT_CHROMIUM"]
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(**launch)
            page = browser.new_page(device_scale_factor=1)
            for index, frame in enumerate(frames):
                svg = frame.read_text(encoding="utf-8")
                page.set_content(f'<html><body style="margin:0">{svg}</body></html>')
                element = page.query_selector("svg")
                element.screenshot(path=str(scratch_dir / f"png_{index:04d}.png"))
            browser.close()
        last = scratch_dir / f"png_{len(frames) - 1:04d}.png"
        for extra in range(args.hold):
            shutil.copy(last, scratch_dir / f"png_{len(frames) + extra:04d}.png")

        args.output.parent.mkdir(parents=True, exist_ok=True)
        scale = f"scale={args.width}:-1:flags=lanczos"
        subprocess.run(
            [
                "ffmpeg", "-v", "error", "-y",
                "-framerate", str(args.fps),
                "-i", str(scratch_dir / "png_%04d.png"),
                "-vf", f"{scale},split[a][b];[a]palettegen=max_colors=96:stats_mode=diff[p];"
                       "[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle",
                str(args.output),
            ],
            check=True,
        )
    print(f"wrote {args.output} ({args.output.stat().st_size} bytes, {len(frames)} frames)")


if __name__ == "__main__":
    main()
