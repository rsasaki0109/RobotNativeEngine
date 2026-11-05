#!/usr/bin/env python3
"""Compare example 147's Go2 training runs across engines.

Inputs are training reports (`147_go2_learning --train`, with `--backend
native` for RNE's native backend or `--solver-iterations 1 --inner-passes 4`
for Rapier's cheaper solve, and `mujoco_go2_learning.py train`) and,
optionally, evaluations of each run's final policy in each engine
(`--evaluate`, `mujoco_go2_learning.py evaluate`). For every run the summary
holds:
- when the mean policy's fitness first reached each threshold, in
  generations and in wall-clock seconds;
- the final fitness, forward travel and whether it fell;
- the training speed in simulated seconds per wall second.
For every evaluation it holds the outcome in the evaluating engine. The
figure plots the mean policy's fitness against wall-clock minutes and against
generations, one line per run. Uses no third-party packages.

Usage: learning_compare.py SUMMARY.json FIGURE.svg --training ENGINE:SEED=REPORT.json ...
       [--evaluation EVALUATING/ENGINE:SEED=EVALUATION.json ...]
where ENGINE and EVALUATING are rne, rne2x1, rne1x4, native or mujoco.
"""
import argparse
import json
import statistics
from pathlib import Path

ENGINES = {"rne": "RNE (4 iterations)", "rne2x1": "RNE (2 iterations)",
           "rne1x4": "RNE (1 iteration × 4 passes)",
           "native": "RNE (native)", "mujoco": "MuJoCo"}
COLORS = {"rne": "#2563eb", "rne2x1": "#0891b2", "rne1x4": "#7c3aed", "native": "#16a34a",
          "mujoco": "#ea580c"}
INK, MUTED, GRID = "#1f2937", "#6b7280", "#e5e7eb"
THRESHOLDS = (0.5, 1.0)


def run_summary(report):
    log = report["training"]["log"]
    reached = {}
    for threshold in THRESHOLDS:
        entry = next((e for e in log if e["mean"]["fitness"] >= threshold), None)
        reached[str(threshold)] = None if entry is None else {
            "generation": entry["generation"], "wall_s": round(entry["wall_s"], 1)}
    walls = [entry["wall_s"] for entry in log]
    per_generation = [b - a for a, b in zip(walls, walls[1:-1])] or walls[:1]
    last = log[-1]
    return {"engine": report["engine"], "plant": report["plant"],
            "generations": report["training"]["config"]["generations"],
            "reached": reached,
            "final": {key: last["mean"][key] for key in ("fitness", "forward_m", "fell")},
            "best_fitness": max(e["mean"]["fitness"] for e in log),
            "median_wall_s_per_generation": round(statistics.median(per_generation), 3),
            "simulated_s_per_wall_s": round(last["simulated_s"] / last["wall_s"], 1),
            "total_wall_s": round(last["wall_s"], 1)}


def panel(parts, x0, y0, w, h, title, lines, xmax, xlabel, ymin, ymax):
    parts.append(f'<text x="{x0}" y="{y0 - 10}" font-size="13" fill="{INK}">{title}</text>')
    for k in range(5):
        y = y0 + h - h * k / 4
        value = ymin + (ymax - ymin) * k / 4
        parts.append(f'<line x1="{x0}" y1="{y:.1f}" x2="{x0 + w}" y2="{y:.1f}" stroke="{GRID}"/>')
        parts.append(f'<text x="{x0 - 6}" y="{y + 4:.1f}" font-size="10" fill="{MUTED}" '
                     f'text-anchor="end">{value:.1f}</text>')
    for k in range(5):
        x = x0 + w * k / 4
        parts.append(f'<text x="{x:.1f}" y="{y0 + h + 14}" font-size="10" fill="{MUTED}" '
                     f'text-anchor="middle">{xmax * k / 4:.0f}</text>')
    parts.append(f'<text x="{x0 + w / 2}" y="{y0 + h + 30}" font-size="11" fill="{MUTED}" '
                 f'text-anchor="middle">{xlabel}</text>')
    for engine, points in lines:
        path = " ".join(
            f'{"M" if i == 0 else "L"}{x0 + w * min(px / xmax, 1):.1f},'
            f'{y0 + h - h * (min(max(py, ymin), ymax) - ymin) / (ymax - ymin):.1f}'
            for i, (px, py) in enumerate(points))
        parts.append(f'<path d="{path}" fill="none" stroke="{COLORS[engine]}" '
                     f'stroke-width="1.6" stroke-opacity="0.85"/>')


def svg(runs):
    width, height = 920, 360
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
             f'viewBox="0 0 {width} {height}" font-family="sans-serif">',
             f'<rect width="{width}" height="{height}" fill="white"/>']
    fitness = [e["mean"]["fitness"] for _, _, report in runs for e in report["training"]["log"]]
    ymin, ymax = min(-0.5, min(fitness)), max(1.5, max(fitness))
    minutes = max(r["training"]["log"][-1]["wall_s"] for _, _, r in runs) / 60
    generations = max(r["training"]["config"]["generations"] for _, _, r in runs)
    by_time = [(engine, [(e["wall_s"] / 60, e["mean"]["fitness"])
                         for e in report["training"]["log"]]) for engine, _, report in runs]
    by_generation = [(engine, [(e["generation"], e["mean"]["fitness"])
                               for e in report["training"]["log"]])
                     for engine, _, report in runs]
    panel(parts, 60, 50, 360, 240, "Fitness against wall-clock minutes (4 cores)", by_time,
          minutes, "minutes", ymin, ymax)
    panel(parts, 520, 50, 360, 240, "Fitness against generations", by_generation,
          generations, "generation", ymin, ymax)
    engines = sorted({engine for engine, _, _ in runs}, key=list(ENGINES).index)
    for index, engine in enumerate(engines):
        x = 60 + 200 * index
        parts.append(f'<line x1="{x}" y1="{height - 14}" x2="{x + 18}" y2="{height - 14}" '
                     f'stroke="{COLORS[engine]}" stroke-width="2.5"/>')
        parts.append(f'<text x="{x + 24}" y="{height - 10}" font-size="11" fill="{INK}">'
                     f'{ENGINES[engine]}</text>')
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("summary", type=Path)
    parser.add_argument("figure", type=Path)
    parser.add_argument("--training", action="append", default=[], required=True)
    parser.add_argument("--evaluation", action="append", default=[])
    arguments = parser.parse_args()
    runs = []
    for item in arguments.training:
        key, path = item.split("=", 1)
        engine, seed = key.split(":")
        runs.append((engine, int(seed), json.loads(Path(path).read_text())))
    evaluations = []
    for item in arguments.evaluation:
        key, path = item.split("=", 1)
        evaluating, trained = key.split("/")
        engine, seed = trained.split(":")
        outcome = json.loads(Path(path).read_text())["outcome"]
        evaluations.append({"evaluated_in": evaluating, "trained_in": engine,
                            "seed": int(seed), "outcome": outcome})
    summary = {"schema_version": 1, "thresholds": THRESHOLDS,
               "runs": [{"trained_in": engine, "seed": seed, **run_summary(report)}
                        for engine, seed, report in runs],
               "evaluations": evaluations}
    arguments.summary.write_text(json.dumps(summary, indent=2) + "\n")
    arguments.figure.write_text(svg(runs))
    for run in summary["runs"]:
        print(run["trained_in"], run["seed"], run["reached"], run["final"],
              run["simulated_s_per_wall_s"], run["total_wall_s"])
    for evaluation in evaluations:
        print(evaluation)


if __name__ == "__main__":
    main()
