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


VERDICT_COLOURS = [
    ("-> used", (120, 230, 140)),
    ("passes", (240, 210, 90)),
    ("rejected", (255, 110, 90)),
]
EVENT_COLOUR = (255, 220, 80)
LIDAR_COLOUR = (60, 240, 255)


def fitted(draw, text, typeface, width):
    """The text, shortened with an ellipsis to fit `width` pixels."""
    if draw.textlength(text, font=typeface) <= width:
        return text
    while text and draw.textlength(text + "...", font=typeface) > width:
        text = text[:-1]
    return text + "..."


GHOST_COLOUR = (255, 120, 100)
ROBOT_COLOUR = (250, 200, 40)


class Minimap:
    """A north-up map of the roads with the robot's and the ghost's tracks,
    for runs with `--ghost`: the ghost is often too far away for the chase
    camera to show."""

    def __init__(self, data, size):
        x0, y0, x1, y1 = data["bounds"]
        span = max(x1 - x0, y1 - y0)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        self.size = size
        self.px = lambda x, y: (
            (x - cx) / span * (size - 8) + size / 2,
            size / 2 - (y - cy) / span * (size - 8),
        )
        self.base = Image.new("RGBA", (size, size), (20, 22, 28, 200))
        draw = ImageDraw.Draw(self.base)
        for t in data["roads"]:
            draw.polygon([self.px(t[0], t[1]), self.px(t[2], t[3]), self.px(t[4], t[5])], fill=(95, 98, 105, 255))
        draw.line([self.px(x, y) for x, y in data["route"]], fill=(200, 200, 200, 255), width=1)

    def draw(self, image, x, y, frames, upto, small):
        """Pastes the map at `(x, y)` with the tracks of `frames[: upto + 1]`."""
        tile = self.base.copy()
        draw = ImageDraw.Draw(tile)
        seen = frames[: upto + 1]
        truth = [self.px(*f["truth"]) for f in seen if f.get("truth")]
        ghost = [self.px(*f["ghost"]) for f in seen if f.get("ghost")]
        if len(ghost) > 1:
            draw.line(ghost, fill=GHOST_COLOUR, width=2)
        if len(truth) > 1:
            draw.line(truth, fill=ROBOT_COLOUR, width=2)
        for points, colour in ((ghost, GHOST_COLOUR), (truth, ROBOT_COLOUR)):
            if points:
                (px, py), r = points[-1], 4
                draw.ellipse([px - r, py - r, px + r, py + r], fill=colour, outline=(0, 0, 0))
        image.paste(tile, (x, y), tile)
        out = ImageDraw.Draw(image)
        ly = y + self.size + 2
        out.text((x, ly), "robot", font=small, fill=ROBOT_COLOUR)
        out.text((x + out.textlength("robot  ", font=small), ly), "without GNSS", font=small, fill=GHOST_COLOUR)


def hud(image, area, frame, width, timeline, constellation="GPS+Galileo+QZSS", minimap=None):
    scale = width / image.width
    image = image.resize((width, round(image.height * scale)), Image.LANCZOS)
    draw = ImageDraw.Draw(image, "RGBA")
    big, small = font(max(12, width // 36)), font(max(10, width // 50))
    pad = width // 60
    lines = (4 if frame.get("gnss") else 2) + (frame.get("ghost_error_m") is not None)
    hud_bottom = pad * 2 + big.size + (small.size + 2) * lines + 4
    draw.rectangle([0, 0, width, hud_bottom], fill=(10, 12, 18, 170))
    if minimap is not None:
        frames, upto, mm = minimap
        mm.draw(image, width - mm.size - pad, hud_bottom + pad, frames, upto, small)
    model = "synthetic scene" if area.startswith(("Suburb", "Tunnel", "Long tunnel")) else "PLATEAU LOD2"
    title = f"{area} - {model}, ray-traced {constellation}"
    heading = big
    while draw.textlength(title, font=heading) > width - 2 * pad and heading.size > 10:
        heading = font(heading.size - 1)
    draw.text((pad, pad), title, font=heading, fill=(255, 255, 255))
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
    if frame.get("gnss"):
        # Why the localizer trusts what it trusts: the verdict on the latest
        # GNSS solution and the LiDAR's map match.
        y += small.size + 2
        colour = next((c for key, c in VERDICT_COLOURS if key in frame["gnss"]), (200, 200, 200))
        draw.text((pad, y), fitted(draw, frame["gnss"], small, width - 2 * pad), font=small, fill=colour)
        y += small.size + 2
        draw.text((pad, y), fitted(draw, frame["lidar"], small, width - 2 * pad), font=small, fill=LIDAR_COLOUR)
    if frame.get("ghost_error_m") is not None:
        # The same LiDAR and wheels without GNSS: the red robot under a pole.
        y += small.size + 2
        draw.text(
            (pad, y),
            f"without GNSS (red, map at right): {frame['ghost_error_m']:.1f} m off",
            font=small,
            fill=GHOST_COLOUR,
        )
    # Legend.
    legend = [
        ((60, 230, 120), "line of sight"),
        ((255, 60, 50), "NLOS reflection"),
        ((170, 90, 255), "diffracted"),
        (LIDAR_COLOUR, "LiDAR"),
    ]
    lx, ly = pad, image.height - pad - small.size
    for colour, label in legend:
        draw.rectangle([lx, ly + small.size // 3, lx + small.size, ly + small.size // 3 + 4], fill=colour)
        draw.text((lx + small.size + 4, ly), label, font=small, fill=(255, 255, 255))
        lx += small.size + 12 + draw.textlength(label, font=small)
    # The source over the whole run, with the current moment marked.
    if timeline:
        bar_y = ly - 10
        x0, x1 = pad, width - pad
        span = timeline[-1][0] - timeline[0][0] or 1.0
        for (t, src), (t_next, _) in zip(timeline, timeline[1:] + [(timeline[-1][0] + 1.0, None)]):
            a = x0 + (x1 - x0) * (t - timeline[0][0]) / span
            b = x0 + (x1 - x0) * (t_next - timeline[0][0]) / span
            draw.rectangle([a, bar_y, max(a + 1, b), bar_y + 5], fill=SOURCE_COLOURS.get(src, (128, 128, 128)))
        cx = x0 + (x1 - x0) * (frame["t"] - timeline[0][0]) / span
        draw.rectangle([cx - 1, bar_y - 3, cx + 1, bar_y + 8], fill=(255, 255, 255))
        # A change of source, and why, just above it.
        if frame.get("event"):
            text = fitted(draw, frame["event"], small, width - 4 * pad)
            tw = draw.textlength(text, font=small)
            top = bar_y - small.size - 12
            draw.rectangle([pad, top, pad + tw + 2 * pad, bar_y - 5], fill=(20, 20, 20, 210))
            draw.text((2 * pad, top + 3), text, font=small, fill=EVENT_COLOUR)
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("capture", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--stride", type=int, default=3, help="use every Nth rendered frame")
    parser.add_argument("--fps", type=float, default=7.0)
    parser.add_argument("--width", type=int, default=560)
    parser.add_argument("--colors", type=int, default=64)
    parser.add_argument("--label", default="", help="what the run shows, after the area in the title")
    args = parser.parse_args()

    meta = json.loads((args.capture / "frames.json").read_text())
    timeline = [(f["t"], f["source"]) for f in meta["frames"]]
    mm = Minimap(meta["map"], args.width * 2 // 7) if meta.get("map") else None
    images = []
    for k in range(0, len(meta["frames"]), args.stride):
        frame = meta["frames"][k]
        image = Image.open(args.capture / frame["file"]).convert("RGB")
        area = f"{meta['area']}, {args.label}" if args.label else meta["area"]
        images.append(
            hud(
                image,
                area,
                frame,
                args.width,
                timeline,
                meta.get("constellation", "GPS+Galileo+QZSS"),
                (meta["frames"], k, mm) if mm else None,
            )
        )
    # One palette for the whole GIF: colours from sample frames plus the
    # exact HUD and legend colours, so those never shift.
    samples = images[:: max(1, len(images) // 12)]
    exact = list(SOURCE_COLOURS.values()) + [
        (60, 230, 120),
        (255, 60, 50),
        (170, 90, 255),
        (255, 255, 255),
        LIDAR_COLOUR,
        EVENT_COLOUR,
        GHOST_COLOUR,
        ROBOT_COLOUR,
    ] + [c for _, c in VERDICT_COLOURS]
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
