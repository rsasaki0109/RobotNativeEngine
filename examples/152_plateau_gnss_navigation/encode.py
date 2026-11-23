#!/usr/bin/env python3
"""Adds a HUD to example 152's rendered frames and encodes a GIF.

    cargo run --release -p plateau_gnss_navigation --features capture \\
        --example 152_plateau_gnss_navigation -- --quick --render target/152_render
    python3 examples/152_plateau_gnss_navigation/encode.py target/152_render/shibuya \\
        docs/media/plateau-gnss-shibuya.gif

The directory must hold `frames.json` and the PNG frames written by the
example's `--render`. Needs Pillow (see requirements.txt).
"""

import argparse
import json
import pathlib

from PIL import Image, ImageDraw, ImageFont

SOURCE_COLOURS = {
    "Gnss": (60, 130, 255),
    "Lidar": (235, 60, 235),
    "DeadReckoning": (255, 150, 0),
}
SOURCE_NAMES = {"Gnss": "GNSS (RTK)", "Lidar": "LiDAR map", "DeadReckoning": "dead reckoning"}


def font(size):
    for name in ("DejaVuSans-Bold.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def hud(image, area, frame, width):
    scale = width / image.width
    image = image.resize((width, round(image.height * scale)), Image.LANCZOS)
    draw = ImageDraw.Draw(image, "RGBA")
    big, small = font(max(12, width // 36)), font(max(10, width // 50))
    pad = width // 60
    draw.rectangle([0, 0, width, pad * 2 + big.size + small.size * 2 + 6], fill=(10, 12, 18, 170))
    draw.text((pad, pad), f"{area} - PLATEAU LOD2, ray-traced GPS+Galileo+QZSS", font=big, fill=(255, 255, 255))
    source = frame["source"]
    y = pad + big.size + 4
    draw.text(
        (pad, y),
        f"t {frame['t']:5.0f} s   localization: ",
        font=small,
        fill=(220, 220, 220),
    )
    x = pad + draw.textlength(f"t {frame['t']:5.0f} s   localization: ", font=small)
    draw.text((x, y), SOURCE_NAMES.get(source, source), font=small, fill=SOURCE_COLOURS.get(source, (255, 255, 255)))
    if frame.get("weather"):
        x += draw.textlength(SOURCE_NAMES.get(source, source), font=small)
        draw.text((x, y), f"   {frame['weather']}", font=small, fill=(220, 220, 220))
    y += small.size + 2
    draw.text(
        (pad, y),
        f"{frame['solution']}   error {frame['error_m']:.2f} m   "
        f"satellites: {frame['los']} line of sight, {frame['nlos']} NLOS/diffracted",
        font=small,
        fill=(220, 220, 220),
    )
    # Legend.
    legend = [
        ((60, 230, 120), "line of sight"),
        ((255, 60, 50), "NLOS reflection"),
        ((170, 90, 255), "diffracted"),
        ((60, 240, 255), "LiDAR"),
    ]
    lx, ly = pad, image.height - pad - small.size
    for colour, label in legend:
        draw.rectangle([lx, ly + small.size // 3, lx + small.size, ly + small.size // 3 + 4], fill=colour)
        draw.text((lx + small.size + 4, ly), label, font=small, fill=(255, 255, 255))
        lx += small.size + 12 + draw.textlength(label, font=small)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("capture", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--stride", type=int, default=3, help="use every Nth rendered frame")
    parser.add_argument("--fps", type=float, default=7.0)
    parser.add_argument("--width", type=int, default=560)
    parser.add_argument("--colors", type=int, default=64)
    args = parser.parse_args()

    meta = json.loads((args.capture / "frames.json").read_text())
    images = []
    for frame in meta["frames"][:: args.stride]:
        image = Image.open(args.capture / frame["file"]).convert("RGB")
        images.append(hud(image, meta["area"], frame, args.width))
    # One palette for the whole GIF: colours from sample frames plus the
    # exact HUD and legend colours, so those never shift.
    samples = images[:: max(1, len(images) // 12)]
    exact = list(SOURCE_COLOURS.values()) + [(60, 230, 120), (255, 60, 50), (170, 90, 255), (255, 255, 255)]
    sheet = Image.new("RGB", (args.width, sum(im.height for im in samples)))
    y = 0
    for im in samples:
        sheet.paste(im, (0, y))
        y += im.height
    base = sheet.quantize(colors=args.colors - len(exact), method=Image.Quantize.MEDIANCUT)
    entries = base.getpalette()[: 3 * (args.colors - len(exact))]
    for colour in exact:
        entries.extend(colour)
    palette = Image.new("P", (1, 1))
    palette.putpalette(entries + [0] * (768 - len(entries)))
    frames = [im.quantize(palette=palette, dither=Image.Dither.NONE) for im in images]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        args.output,
        save_all=True,
        append_images=frames[1:],
        duration=round(1000 / args.fps),
        loop=0,
        optimize=True,
    )
    print(f"wrote {args.output} ({args.output.stat().st_size} bytes, {len(frames)} frames)")


if __name__ == "__main__":
    main()
