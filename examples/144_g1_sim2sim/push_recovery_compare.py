#!/usr/bin/env python3
"""Compare push-recovery residuals trained in RNE and in MuJoCo by what they buy
per minute of training.

Inputs are the two training reports (example 144's `--train-residual
--checkpoint-every N` and mujoco_residual.py, which repeats it in MuJoCo with
the same seed and settings) and 640-trial push sweeps of their checkpoints in
both engines (`--robustness --residual REPORT --residual-generation G` and
mujoco_robustness.py `--residual REPORT --residual-generation G`); each
generation-0 sweep is the plain policy's. Each training engine's own fitness
is not comparable across engines, since a push is harder to survive in one
than the other, so the figure instead plots falls in each engine's sweep
against the wall-clock minutes the checkpoint took to train. Uses no
third-party packages.

Usage: push_recovery_compare.py SUMMARY.json FIGURE.svg RNE_TRAINING.json
       MUJOCO_TRAINING.json --sweep TRAINED/EVALUATED/GENERATION=SWEEP.json ...
where TRAINED and EVALUATED are rne or mujoco.
"""
import argparse
import json
import statistics
from pathlib import Path

ENGINES = {"rne": "RNE", "mujoco": "MuJoCo"}
COLORS = {"rne": "#2563eb", "mujoco": "#ea580c"}
INK, MUTED, GRID = "#1f2937", "#6b7280", "#e5e7eb"


def timing(report):
    log = report["training"]["log"]
    walls = [entry["wall_s"] for entry in log]
    per_generation = [b - a for a, b in zip(walls, walls[1:-1])] or walls[:1]
    return {"generations": report["training"]["generations"],
            "median_wall_s_per_generation": round(statistics.median(per_generation), 2),
            "simulated_s_per_wall_s": round(log[-1]["simulated_s"] / log[-1]["wall_s"], 1),
            "total_wall_s": round(log[-1]["wall_s"], 1)}


def checkpoint_wall_s(report):
    return {c["generation"]: c["wall_s"] for c in report["training"]["checkpoints"]}


def sweep_summary(path):
    sweep = json.loads(Path(path).read_text())
    speeds = {f["friction"]: f["summary"]["second_half_forward_speed_m_s"] for f in sweep["friction"]}
    return {"falls": sum(trial["fell"] for trial in sweep["push"]), "trials": len(sweep["push"]),
            "walks_at_friction": [f["friction"] for f in sweep["friction"] if not f["summary"]["fell"]],
            "forward_speed_friction_1_m_s": round(speeds.get(1.0, float("nan")), 3)}


def panel(parts, x0, y0, w, h, title, lines, xmax, ymin, ymax):
    parts.append(f'<text x="{x0}" y="{y0 - 12}" fill="{INK}" font-size="13">{title}</text>')
    sx = lambda t: x0 + t / xmax * w
    sy = lambda v: y0 + h - (v - ymin) / (ymax - ymin) * h
    for tick in range(int(ymin), int(ymax) + 1, 20):
        parts.append(f'<line x1="{x0}" x2="{x0 + w}" y1="{sy(tick):.1f}" y2="{sy(tick):.1f}" '
                     f'stroke="{GRID}" stroke-width="1"/>')
        parts.append(f'<text x="{x0 - 6}" y="{sy(tick) + 4:.1f}" text-anchor="end" '
                     f'fill="{MUTED}">{tick}</text>')
    step = 10 if xmax > 30 else 5
    for minute in range(0, int(xmax) + 1, step):
        parts.append(f'<text x="{sx(minute):.1f}" y="{y0 + h + 16}" text-anchor="middle" '
                     f'fill="{MUTED}">{minute}</text>')
    for trained, points in lines:
        color = COLORS[trained]
        coords = [(sx(minutes), sy(falls)) for minutes, falls in points]
        parts.append('<polyline points="' + " ".join(f"{x:.1f},{y:.1f}" for x, y in coords)
                     + f'" fill="none" stroke="{color}" stroke-width="2" stroke-linejoin="round" '
                     f'stroke-linecap="round"/>')
        for x, y in coords:
            parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" fill="{color}" '
                         f'stroke="white" stroke-width="2"/>')
        x, y = coords[-1]
        parts.append(f'<text x="{x + 8:.1f}" y="{y + 4:.1f}" fill="{INK}">{points[-1][1]}</text>')


def svg(robot, curves, falls_range):
    width, height = 960, 340
    xmax = max(m for lines in curves.values() for _, pts in lines for m, _ in pts) * 1.12
    ymin, ymax = falls_range
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
             f'font-family="sans-serif" font-size="12">',
             '<rect width="100%" height="100%" fill="white"/>',
             f'<text x="56" y="22" fill="{INK}" font-size="14">{robot.upper()}: falls in 640 '
             f'pushes after training an MLP push-recovery residual for so many minutes</text>']
    x = 56
    for trained in ("rne", "mujoco"):
        name = f"trained in {ENGINES[trained]}"
        parts.append(f'<line x1="{x}" x2="{x + 22}" y1="42" y2="42" stroke="{COLORS[trained]}" '
                     f'stroke-width="3" stroke-linecap="round"/>')
        parts.append(f'<text x="{x + 28}" y="46" fill="{INK}">{name}</text>')
        x += 50 + 7 * len(name)
    for index, evaluated in enumerate(("rne", "mujoco")):
        panel(parts, 56 + index * 470, 90, 360, 200, f"Evaluated in {ENGINES[evaluated]}",
              curves[evaluated], xmax, ymin, ymax)
    parts.append(f'<text x="{width / 2}" y="{height - 10}" text-anchor="middle" fill="{MUTED}">'
                 f'training wall-clock minutes on the same 4-core machine (fewer falls is better)'
                 f'</text>')
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument("figure", type=Path)
    parser.add_argument("rne_training", type=Path)
    parser.add_argument("mujoco_training", type=Path)
    parser.add_argument("--sweep", action="append", default=[], required=True,
                        help="TRAINED/EVALUATED/GENERATION=SWEEP.json")
    arguments = parser.parse_args()
    trainings = {"rne": json.loads(arguments.rne_training.read_text()),
                 "mujoco": json.loads(arguments.mujoco_training.read_text())}
    robot = trainings["rne"]["robot"]
    sweeps = {}
    for spec in arguments.sweep:
        key, path = spec.split("=", 1)
        trained, evaluated, generation = key.split("/")
        if trained not in ENGINES or evaluated not in ENGINES:
            raise SystemExit(f"bad --sweep {spec}")
        sweeps[(trained, evaluated, int(generation))] = sweep_summary(path)
    walls = {t: checkpoint_wall_s(r) for t, r in trainings.items()}
    curves, rows = {}, []
    for evaluated in ENGINES:
        curves[evaluated] = []
        for trained in ENGINES:
            points = []
            for generation in sorted(g for (t, e, g) in sweeps if t == trained and e == evaluated):
                result = sweeps[(trained, evaluated, generation)]
                minutes = walls[trained].get(generation, 0.0) / 60 if generation else 0.0
                points.append((minutes, result["falls"]))
                rows.append({"trained_in": ENGINES[trained], "evaluated_in": ENGINES[evaluated],
                             "generation": generation, "training_wall_s": round(minutes * 60, 1),
                             **result})
            curves[evaluated].append((trained, points))
            print(f"trained in {ENGINES[trained]}, evaluated in {ENGINES[evaluated]}: "
                  + ", ".join(f"{m:.1f} min {f}" for m, f in points))
    falls = [row["falls"] for row in rows]
    falls_range = (20 * (min(falls) // 20) - 20, 20 * (max(falls) // 20) + 20)
    training = trainings["rne"]["training"]
    summary = {"schema_version": 1, "robot": robot,
               "settings": {k: training[k] for k in (
                   "method", "generations", "pairs", "sigma", "learning_rate",
                   "offset_penalty_per_rad2", "walk_offset_penalty_per_rad2",
                   "speed_penalty_per_m2_s2", "seed", "checkpoint_every") if k in training},
               "residual_model": {k: v for k, v in trainings["rne"]["residual"].items()
                                  if k not in ("w1", "b1", "w2", "weights_rad")},
               "training": {ENGINES[t]: timing(r) for t, r in trainings.items()},
               "sweeps": rows}
    arguments.summary.write_text(json.dumps(summary, indent=2) + "\n")
    arguments.figure.write_text(svg(robot, curves, falls_range))


if __name__ == "__main__":
    main()
