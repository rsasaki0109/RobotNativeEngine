#!/usr/bin/env python3
"""Compare example 147's Go2 training runs across engines.

Inputs are training reports (`147_go2_learning --train`, with `--backend
native` for RNE's native backend, `--backend locomotion` for
`rne_locomotion`'s terrain step, or `--solver-iterations 1 --inner-passes 4`
for Rapier's cheaper solve, and `mujoco_go2_learning.py train`) and,
optionally, evaluations of each run's final policy in each engine
(`--evaluate`, `mujoco_go2_learning.py evaluate`). For every run the summary
holds:
- when the mean policy's fitness first reached each threshold, in
  generations and in wall-clock seconds;
- the final fitness, forward travel and whether it fell;
- the training speed in simulated seconds per wall second.
For every evaluation it holds the outcome in the evaluating engine. Its
`aggregate` section summarizes the runs of each engine and compares engines:
- per engine, how many runs reached each threshold and the quartiles of the
  generations and seconds they took (a run that never reached it counts as
  slower than every run that did), and the quartiles of the final fitness and
  of the training speed;
- per pair of engines, over the seeds both trained, on how many each reached
  a threshold first (in generations and in seconds) and ended fitter, with
  the two-sided sign test's p-value over the seeds that differ;
- per evaluating and training engine, how many policies held the episode
  without falling, and the quartiles of their fitness there and at home.
The figure plots the mean policy's fitness against wall-clock minutes and
against generations, one line per run, with a bold median line for an engine
with five runs or more. Uses no third-party packages.

Usage: learning_compare.py SUMMARY.json FIGURE.svg --training ENGINE:SEED=REPORT.json ...
       [--evaluation EVALUATING/ENGINE:SEED=EVALUATION.json ...]
where ENGINE and EVALUATING are rne, rne2x1, rne1x4, native, locomotion or mujoco.
"""
import argparse
import json
import math
import statistics
from pathlib import Path

ENGINES = {"rne": "RNE (4 iterations)", "rne2x1": "RNE (2 iterations)",
           "rne1x4": "RNE (1 iteration × 4 passes)",
           "native": "RNE (native)", "locomotion": "RNE (locomotion step)",
           "mujoco": "MuJoCo"}
COLORS = {"rne": "#2563eb", "rne2x1": "#0891b2", "rne1x4": "#7c3aed", "native": "#16a34a",
          "locomotion": "#db2777", "mujoco": "#ea580c"}
INK, MUTED, GRID = "#1f2937", "#6b7280", "#e5e7eb"
THRESHOLDS = (0.5, 1.0)
# An engine with this many runs or more gets a bold median line in the figure.
MEDIAN_FROM_RUNS = 5


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


def quartiles(values):
    """First quartile, median and third quartile by linear interpolation, with
    None for an unreached threshold (infinity) or no values."""
    ordered = sorted(values)
    if not ordered:
        return None
    result = []
    for fraction in (0.25, 0.5, 0.75):
        position = fraction * (len(ordered) - 1)
        low, high = int(position), min(int(position) + 1, len(ordered) - 1)
        weight = position - low
        if weight == 0.0 or ordered[low] == ordered[high]:
            value = ordered[low]
        elif math.isinf(ordered[high]):
            value = math.inf
        else:
            value = ordered[low] + weight * (ordered[high] - ordered[low])
        result.append(None if math.isinf(value) else round(value, 3))
    return result


def sign_test(wins, losses):
    """Two-sided exact sign test p-value for `wins` against `losses`."""
    trials = wins + losses
    if trials == 0:
        return None
    tail = sum(math.comb(trials, k) for k in range(min(wins, losses) + 1)) / 2 ** trials
    # Three significant digits: a lopsided 16-seed tally is far below 1e-4.
    return float(f"{min(1.0, 2.0 * tail):.3g}")


def reach_cost(run, threshold, key):
    reached = run["reached"][str(threshold)]
    return math.inf if reached is None else reached[key]


def aggregate(runs, evaluations):
    engines = sorted({run["trained_in"] for run in runs}, key=list(ENGINES).index)
    by_engine = {engine: [run for run in runs if run["trained_in"] == engine]
                 for engine in engines}
    per_engine = {}
    for engine, own in by_engine.items():
        per_engine[engine] = {
            "runs": len(own),
            "reached": {str(threshold): {
                "runs": sum(run["reached"][str(threshold)] is not None for run in own),
                "generation_quartiles": quartiles(
                    [reach_cost(run, threshold, "generation") for run in own]),
                "wall_s_quartiles": quartiles(
                    [reach_cost(run, threshold, "wall_s") for run in own])}
                for threshold in THRESHOLDS},
            "final_fitness_quartiles": quartiles([run["final"]["fitness"] for run in own]),
            "falls_at_the_end": sum(run["final"]["fell"] for run in own),
            "simulated_s_per_wall_s_quartiles": quartiles(
                [run["simulated_s_per_wall_s"] for run in own]),
            "wall_s_per_generation_quartiles": quartiles(
                [run["median_wall_s_per_generation"] for run in own])}
    pairs = []
    for index, first in enumerate(engines):
        for second in engines[index + 1:]:
            mine = {run["seed"]: run for run in by_engine[first]}
            theirs = {run["seed"]: run for run in by_engine[second]}
            seeds = sorted(set(mine) & set(theirs))
            if not seeds:
                continue
            comparisons = {}

            def tally(name, better):
                outcomes = [better(mine[seed], theirs[seed]) for seed in seeds]
                wins, losses = outcomes.count(1), outcomes.count(-1)
                comparisons[name] = {first: wins, second: losses,
                                     "ties": len(seeds) - wins - losses,
                                     "sign_test_p": sign_test(wins, losses)}

            def sooner(threshold, key):
                def better(a, b):
                    cost_a, cost_b = reach_cost(a, threshold, key), reach_cost(b, threshold, key)
                    return (cost_a < cost_b) - (cost_a > cost_b)
                return better

            for threshold in THRESHOLDS:
                tally(f"reached_{threshold}_in_fewer_generations", sooner(threshold, "generation"))
                tally(f"reached_{threshold}_sooner", sooner(threshold, "wall_s"))
            tally("fitter_after_training",
                  lambda a, b: (a["final"]["fitness"] > b["final"]["fitness"])
                  - (a["final"]["fitness"] < b["final"]["fitness"]))
            pairs.append({"engines": [first, second], "seeds": seeds, **comparisons})
    home = {(run["trained_in"], run["seed"]): run["final"]["fitness"] for run in runs}
    transfer = []
    for evaluating, trained in sorted({(e["evaluated_in"], e["trained_in"])
                                       for e in evaluations if e["evaluated_in"] != e["trained_in"]}):
        chosen = [e for e in evaluations
                  if e["evaluated_in"] == evaluating and e["trained_in"] == trained]
        transfer.append({
            "evaluated_in": evaluating, "trained_in": trained, "policies": len(chosen),
            "held": sum(not e["outcome"]["fell"] for e in chosen),
            "fitness_quartiles": quartiles([e["outcome"]["fitness"] for e in chosen]),
            "home_fitness_quartiles": quartiles(
                [home[(trained, e["seed"])] for e in chosen if (trained, e["seed"]) in home])})
    return {"per_engine": per_engine, "pairs": pairs, "transfer": transfer}


def median_lines(lines, samples=120):
    """Each engine's median fitness at evenly spaced x, holding each run's
    last value before x, up to where its first run ends, for engines with
    MEDIAN_FROM_RUNS runs or more."""
    result = []
    for engine in sorted({engine for engine, _ in lines}, key=list(ENGINES).index):
        runs = [points for e, points in lines if e == engine]
        if len(runs) < MEDIAN_FROM_RUNS:
            continue
        end = min(run[-1][0] for run in runs)
        points = []
        for k in range(samples + 1):
            x = end * k / samples
            values = [next((py for px, py in reversed(run) if px <= x), run[0][1])
                      for run in runs]
            points.append((x, statistics.median(values)))
        result.append((engine, points))
    return result


def panel(parts, x0, y0, w, h, title, lines, xmax, xlabel, ymin, ymax):
    medians = median_lines(lines)
    parts.append(f'<text x="{x0}" y="{y0 - 10}" font-size="13" fill="{INK}">{title}</text>')
    for k in range(5):
        y = y0 + h - h * k / 4
        value = ymin + (ymax - ymin) * k / 4
        parts.append(f'<line x1="{x0}" y1="{y:.1f}" x2="{x0 + w}" y2="{y:.1f}" stroke="{GRID}"/>')
        parts.append(f'<text x="{x0 - 6}" y="{y + 4:.1f}" font-size="10" fill="{MUTED}" '
                     f'text-anchor="end">{value:.1f}</text>')
    # Whole numbers, unless the axis is too short for them to tell ticks apart.
    decimals = 0 if xmax >= 8 else 1
    for k in range(5):
        x = x0 + w * k / 4
        parts.append(f'<text x="{x:.1f}" y="{y0 + h + 14}" font-size="10" fill="{MUTED}" '
                     f'text-anchor="middle">{xmax * k / 4:.{decimals}f}</text>')
    parts.append(f'<text x="{x0 + w / 2}" y="{y0 + h + 30}" font-size="11" fill="{MUTED}" '
                 f'text-anchor="middle">{xlabel}</text>')
    def path(points):
        return " ".join(
            f'{"M" if i == 0 else "L"}{x0 + w * min(px / xmax, 1):.1f},'
            f'{y0 + h - h * (min(max(py, ymin), ymax) - ymin) / (ymax - ymin):.1f}'
            for i, (px, py) in enumerate(points))

    counts = {engine: sum(e == engine for e, _ in lines) for engine, _ in lines}
    for engine, points in lines:
        # Many runs: faint lines under a bold median.
        faint = counts[engine] >= MEDIAN_FROM_RUNS
        parts.append(f'<path d="{path(points)}" fill="none" stroke="{COLORS[engine]}" '
                     f'stroke-width="{1.0 if faint else 1.6}" '
                     f'stroke-opacity="{0.3 if faint else 0.85}"/>')
    for engine, points in medians:
        parts.append(f'<path d="{path(points)}" fill="none" stroke="{COLORS[engine]}" '
                     f'stroke-width="2.6"/>')


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
    summary["aggregate"] = aggregate(summary["runs"], evaluations)
    arguments.summary.write_text(json.dumps(summary, indent=2) + "\n")
    arguments.figure.write_text(svg(runs))
    for run in summary["runs"]:
        print(run["trained_in"], run["seed"], run["reached"], run["final"],
              run["simulated_s_per_wall_s"], run["total_wall_s"])
    for evaluation in evaluations:
        print(evaluation)
    print(json.dumps(summary["aggregate"], indent=1))


if __name__ == "__main__":
    main()
