#!/usr/bin/env python3
"""Compose labelled rows from the evaluation's view panels for the docs.

Usage: montage.py EVAL_DIR OUTPUT.png [VIEW ...]
"""
import json
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

TILE = 300


def font(size, bold=False):
    try:
        return ImageFont.truetype("DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf", size)
    except OSError:
        return ImageFont.load_default(size=size)


def main():
    root, output = Path(sys.argv[1]), Path(sys.argv[2])
    views = [int(v) for v in sys.argv[3:]] or [1, 4]
    report = json.loads((root / "report.json").read_text())
    variants = report["variants"]
    labels = ["RNE render", "RNE depth (truth)"] + [
        f"{v['variant']} · {v['bytes'] / 1e6:.0f} MB · AbsRel {v['mean_abs_rel']:.3f}" for v in variants]
    header = 64
    sheet = Image.new("RGB", (TILE * len(labels), header + TILE * len(views)), (17, 24, 35))
    draw = ImageDraw.Draw(sheet)
    draw.text((12, 8), f"Depth Anything V2 Small on RNE renders · {report['runtime']} · "
              f"{report['views']} views at {report['input_px'][0]} px",
              font=font(17, True), fill=(238, 244, 250))
    for column, label in enumerate(labels):
        draw.text((column * TILE + 8, 38), label, font=font(13), fill=(160, 185, 215))
    for row, view in enumerate(views):
        with Image.open(root / f"view-{view}.png") as panel:
            size = panel.height
            for column in range(len(labels)):
                tile = panel.crop((column * size, 0, (column + 1) * size, size)).convert("RGB")
                sheet.paste(tile.resize((TILE, TILE), Image.LANCZOS), (column * TILE, header + row * TILE))
    sheet.save(output, optimize=True)
    print(output, sheet.size)


if __name__ == "__main__":
    main()
