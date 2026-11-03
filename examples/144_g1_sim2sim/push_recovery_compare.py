#!/usr/bin/env python3
"""Compare push-recovery residuals trained in RNE and in MuJoCo by what they buy
per minute of training.

Inputs are training reports (example 144's `--train-residual --checkpoint-every
N --seed S`, and mujoco_residual.py, which repeats a report's run in MuJoCo
with the same seed and settings), the plain policy's 640-trial push sweep in
each engine, and sweeps of the runs' checkpoints in both engines
(`--robustness --residual REPORT --residual-generation G` and
mujoco_robustness.py `--residual REPORT --residual-generation G`). A training
engine's own fitness is not comparable across engines, since the same push is
harder to survive in one than the other, so the figure plots falls in each
engine's sweep against the wall-clock minutes a checkpoint took to train, one
line per run. The summary also averages falls over the runs at every
generation all runs were swept at. Uses no third-party packages.

Usage: push_recovery_compare.py SUMMARY.json FIGURE.svg
       --training TRAINED:SEED=REPORT.json ... --plain EVALUATED=SWEEP.json ...
       --sweep TRAINED:SEED/EVALUATED/GENERATION=SWEEP.json ...
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


def sweep_summary(path):
    sweep = json.loads(Path(path).read_text())
    speeds = {f["friction"]: f["summary"]["second_half_forward_speed_m_s"] for f in sweep["friction"]}
    return {"falls": sum(trial["fell"] for trial in sweep["push"]), "trials": len(sweep["push"]),
            "lowest_friction_walked": min(f["friction"] for f in sweep["friction"]
                                          if not f["summary"]["fell"]),
            "forward_speed_friction_1_m_s": round(speeds[1.0], 3)}


def run_key(text):
    trained, seed = text.split(":")
    if trained not in ENGINES:
        raise SystemExit(f"unknown engine {trained}")
    return trained, int(seed)


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
                     f'stroke-linecap="round" opacity="0.85"/>')
        for x, y in coords[1:]:
            parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" fill="{color}" '
                         f'stroke="white" stroke-width="2"/>')
    x, y = sx(0), sy(lines[0][1][0][1])
    parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" fill="{MUTED}" stroke="white" '
                 f'stroke-width="2"/>')
    parts.append(f'<text x="{x + 8:.1f}" y="{y - 8:.1f}" fill="{MUTED}">plain policy</text>')


def svg(robot, curves, falls_range, runs):
    width, height = 960, 340
    xmax = max(m for lines in curves.values() for _, pts in lines for m, _ in pts) * 1.08
    ymin, ymax = falls_range
    seeds = sorted({seed for _, seed in runs})
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
             f'font-family="sans-serif" font-size="12">',
             '<rect width="100%" height="100%" fill="white"/>',
             f'<text x="56" y="22" fill="{INK}" font-size="14">{robot.upper()}: falls in 640 '
             f'pushes against minutes spent training an MLP push-recovery residual '
             f'({len(seeds)} seeds per engine)</text>']
    x = 56
    for trained in ENGINES:
        name = f"trained in {ENGINES[trained]}"
        parts.append(f'<line x1="{x}" x2="{x + 22}" y1="42" y2="42" stroke="{COLORS[trained]}" '
                     f'stroke-width="3" stroke-linecap="round"/>')
        parts.append(f'<text x="{x + 28}" y="46" fill="{INK}">{name}</text>')
        x += 50 + 7 * len(name)
    for index, evaluated in enumerate(ENGINES):
        panel(parts, 56 + index * 470, 90, 380, 200, f"Evaluated in {ENGINES[evaluated]}",
              curves[evaluated], xmax, ymin, ymax)
    parts.append(f'<text x="{width / 2}" y="{height - 10}" text-anchor="middle" fill="{MUTED}">'
                 f'training wall-clock minutes on the same 4-core machine; one line per run; '
                 f'fewer falls is better</text>')
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument("figure", type=Path)
    parser.add_argument("--training", action="append", required=True,
                        help="TRAINED:SEED=REPORT.json")
    parser.add_argument("--plain", action="append", required=True, help="EVALUATED=SWEEP.json")
    parser.add_argument("--sweep", action="append", required=True,
                        help="TRAINED:SEED/EVALUATED/GENERATION=SWEEP.json")
    arguments = parser.parse_args()
    trainings = {}
    for spec in arguments.training:
        key, path = spec.split("=", 1)
        trainings[run_key(key)] = json.loads(Path(path).read_text())
    plain = {}
    for spec in arguments.plain:
        evaluated, path = spec.split("=", 1)
        plain[evaluated] = sweep_summary(path)
    sweeps = {}
    for spec in arguments.sweep:
        key, path = spec.split("=", 1)
        run, evaluated, generation = key.split("/")
        sweeps[(run_key(run), evaluated, int(generation))] = sweep_summary(path)
    robots = {r["robot"] for r in trainings.values()}
    if len(robots) != 1:
        raise SystemExit("reports are for different robots")
    robot = robots.pop()
    curves, rows = {e: [] for e in ENGINES}, []
    for evaluated in ENGINES:
        for run, report in sorted(trainings.items()):
            walls = {c["generation"]: c["wall_s"] for c in report["training"]["checkpoints"]}
            points = [(0.0, plain[evaluated]["falls"])]
            for generation in sorted(g for (r, e, g) in sweeps if r == run and e == evaluated):
                result = sweeps[(run, evaluated, generation)]
                points.append((walls[generation] / 60, result["falls"]))
                rows.append({"trained_in": ENGINES[run[0]], "seed": run[1],
                             "evaluated_in": ENGINES[evaluated], "generation": generation,
                             "training_wall_s": round(walls[generation], 1), **result})
            curves[evaluated].append((run[0], points))
    common = set.intersection(*({g for (r, _, g) in sweeps if r == run} for run in trainings))
    means = []
    for generation in sorted(common):
        for trained in ENGINES:
            runs = [run for run in trainings if run[0] == trained]
            for evaluated in ENGINES:
                falls = [sweeps[(run, evaluated, generation)]["falls"] for run in runs]
                walls = [next(c["wall_s"] for c in trainings[run]["training"]["checkpoints"]
                              if c["generation"] == generation) for run in runs]
                means.append({"generation": generation, "trained_in": ENGINES[trained],
                              "evaluated_in": ENGINES[evaluated], "runs": len(runs),
                              "mean_falls": round(statistics.mean(falls), 1), "falls": falls,
                              "mean_training_wall_s": round(statistics.mean(walls), 1)})
                print(f"generation {generation}, trained in {ENGINES[trained]}, evaluated in "
                      f"{ENGINES[evaluated]}: mean {statistics.mean(falls):.1f} falls {falls}, "
                      f"{statistics.mean(walls) / 60:.1f} min")
    falls = [p[1] for lines in curves.values() for _, pts in lines for p in pts]
    falls_range = (20 * (min(falls) // 20) - 20, 20 * (max(falls) // 20) + 20)
    first = next(iter(trainings.values()))
    training = first["training"]
    summary = {"schema_version": 1, "robot": robot,
               "settings": {k: training[k] for k in (
                   "method", "pairs", "sigma", "learning_rate", "offset_penalty_per_rad2",
                   "walk_offset_penalty_per_rad2", "speed_penalty_per_m2_s2") if k in training},
               "residual_model": {k: v for k, v in first["residual"].items()
                                  if k not in ("w1", "b1", "w2", "weights_rad")},
               "plain_policy": {ENGINES[e]: result for e, result in plain.items()},
               "training": [{"trained_in": ENGINES[run[0]], "seed": run[1], **timing(report)}
                            for run, report in sorted(trainings.items())],
               "mean_at_common_generations": means,
               "sweeps": rows}
    arguments.summary.write_text(json.dumps(summary, indent=2) + "\n")
    arguments.figure.write_text(svg(robot, curves, falls_range, list(trainings)))


if __name__ == "__main__":
    main()
