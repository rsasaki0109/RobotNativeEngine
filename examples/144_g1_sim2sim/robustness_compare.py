#!/usr/bin/env python3
"""Compare robustness reports of RNE and MuJoCo and draw the fall curves.

Each report comes from example 144's `--robustness` (RNE) or from
mujoco_robustness.py (MuJoCo), all with the same protocol. For every report
and push direction the summary gives the share of the eight gait phases that
fell at each force and the 50% fall force (linearly interpolated between the
grid forces where the share first reaches one half). For every pair of an RNE
and a MuJoCo report it gives the trial-by-trial agreement on falling, the
trials where only one engine fell (with the robot-body pairs that touched in
MuJoCo, which its reports list) and the friction sweep side by side. For two
reports of one engine that differ in self-collision, it counts the trials the
switch changes, and likewise for two reports that differ only in the trained
residual (`--residual`). The figure draws RNE blue and MuJoCo orange, solid
when the robot's links collide with each other, dashed when they do not and
dotted with the residual. Uses no third-party packages.

Usage: robustness_compare.py SUMMARY.json FIGURE.svg REPORT.json [REPORT.json ...]
"""
import json
import sys
from collections import defaultdict
from pathlib import Path


def key(trial):
    return trial["direction"], trial["force_n"], trial["phase"]


def is_rne(report):
    return report["engine"].startswith("RNE")


def self_collision(report):
    # RNE reports before --self-collision existed had none; MuJoCo's MJCF has it.
    return report.get("self_collision", not is_rne(report))


def has_residual(report):
    return bool(report.get("residual"))


def label(report):
    engine = "RNE" if is_rne(report) else "MuJoCo"
    text = f"{engine}, {'self-collision' if self_collision(report) else 'no self-collision'}"
    return text + (", residual" if has_residual(report) else "")


def fall_rates(report):
    rates = defaultdict(lambda: defaultdict(list))
    for trial in report["push"]:
        rates[trial["direction"]][trial["force_n"]].append(trial["fell"])
    return {d: {f: sum(v) / len(v) for f, v in sorted(by_force.items())}
            for d, by_force in rates.items()}


def half_force(by_force):
    """Force at which the fall share first reaches 0.5, or None."""
    previous = (0.0, 0.0)
    for force, rate in by_force.items():
        if rate >= 0.5:
            f0, r0 = previous
            return f0 + (0.5 - r0) / (rate - r0) * (force - f0) if rate > r0 else force
        previous = (force, rate)
    return None


def disagreements(first, second):
    """Trials where only one report fell, and the second's self-contacts in
    the trials where only it fell, if it lists them."""
    first_fell = {key(t): t["fell"] for t in first["push"]}
    only_second = [t for t in second["push"] if t["fell"] and not first_fell[key(t)]]
    only_first = [t for t in second["push"] if first_fell[key(t)] and not t["fell"]]
    result = {"only_first_fell": len(only_first), "only_second_fell": len(only_second)}
    if all("self_contacts" in t for t in second["push"]):
        pairs = defaultdict(int)
        for trial in only_second:
            for pair in trial["self_contacts"]:
                pairs[" / ".join(pair)] += 1
        result["only_second_fell_with_self_contact"] = sum(
            bool(t["self_contacts"]) for t in only_second)
        result["self_contact_pairs"] = dict(sorted(pairs.items(), key=lambda kv: -kv[1]))
    return result


def compare(rne, mujoco):
    other = {key(t): t["fell"] for t in mujoco["push"]}
    agreement = sum(other[key(t)] == t["fell"] for t in rne["push"]) / len(rne["push"])
    friction = [{"friction": a["friction"],
                 "rne_fell": a["summary"]["fell"], "mujoco_fell": b["summary"]["fell"],
                 "rne_forward_speed_m_s": a["summary"]["second_half_forward_speed_m_s"],
                 "mujoco_forward_speed_m_s": b["summary"]["second_half_forward_speed_m_s"]}
                for a, b in zip(rne["friction"], mujoco["friction"])]
    return {"rne": label(rne), "mujoco": label(mujoco), "push_agreement": agreement,
            "push_disagreements": disagreements(rne, mujoco), "friction": friction}


COLORS = {"RNE": "#2563eb", "MuJoCo": "#ea580c"}


def svg(reports, directions, forces, robot):
    width, height, margin = 960, 300, 46
    panel = (width - margin) / len(directions)
    fmax = max(forces)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height + 70}" '
             f'font-family="sans-serif" font-size="12">',
             '<rect width="100%" height="100%" fill="white"/>',
             f'<text x="{margin}" y="18" font-size="14">{robot.upper()}: share of 8 gait '
             f'phases that fell after a 0.1 s pelvis push, by force</text>']
    def dash(report):
        if has_residual(report):
            return ' stroke-dasharray="1 4" stroke-linecap="round"'
        return "" if self_collision(report) else ' stroke-dasharray="5 4"'

    curves = [(label(r), COLORS["RNE" if is_rne(r) else "MuJoCo"], dash(r), fall_rates(r))
              for r in reports]
    for index, direction in enumerate(directions):
        x0 = margin + index * panel + 10
        w, top, bottom = panel - 30, 40, height
        parts.append(f'<rect x="{x0}" y="{top}" width="{w}" height="{bottom - top}" '
                     f'fill="none" stroke="#cbd5e1"/>')
        parts.append(f'<text x="{x0 + w / 2}" y="{top - 6}" text-anchor="middle">{direction}</text>')
        for tick in (0.0, 0.5, 1.0):
            y = bottom - tick * (bottom - top)
            parts.append(f'<line x1="{x0}" x2="{x0 + w}" y1="{y}" y2="{y}" stroke="#e2e8f0"/>')
            if index == 0:
                parts.append(f'<text x="{x0 - 6}" y="{y + 4}" text-anchor="end">{tick:.1f}</text>')
        for force in (0, fmax / 2, fmax):
            x = x0 + force / fmax * w
            parts.append(f'<text x="{x}" y="{bottom + 16}" text-anchor="middle">{force:.0f}</text>')
        for _, color, dash_attributes, rates in curves:
            points = " ".join(f"{x0 + f / fmax * w:.1f},{bottom - r * (bottom - top):.1f}"
                              for f, r in [(0.0, 0.0)] + list(rates[direction].items()))
            dash = dash_attributes
            parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" '
                         f'stroke-width="2"{dash}/>')
    parts.append(f'<text x="{width / 2}" y="{height + 36}" text-anchor="middle">push force (N), '
                 f'held 0.1 s on the pelvis origin while walking at 0.5 m/s</text>')
    x = margin + 10
    for name, color, dash, _ in curves:
        parts.append(f'<line x1="{x}" x2="{x + 24}" y1="{height + 56}" y2="{height + 56}" '
                     f'stroke="{color}" stroke-width="3"{dash}/>')
        parts.append(f'<text x="{x + 30}" y="{height + 60}">{name}</text>')
        x += 40 + 7 * len(name)
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def main():
    if len(sys.argv) < 4:
        raise SystemExit(__doc__)
    summary_path, figure_path = Path(sys.argv[1]), Path(sys.argv[2])
    reports = [json.loads(Path(path).read_text()) for path in sys.argv[3:]]
    first = reports[0]
    if any(r["protocol"] != first["protocol"] or r["robot"] != first["robot"] for r in reports):
        raise SystemExit("reports use different protocols")
    if len({label(r) for r in reports}) != len(reports):
        raise SystemExit("two reports have the same engine and self-collision setting")
    directions = [d["name"] for d in first["protocol"]["directions"]]
    rows = []
    for report in reports:
        rates = fall_rates(report)
        rows.append({"label": label(report), "engine": report["engine"],
                     "self_collision": self_collision(report),
                     "half_fall_force_n": {d: half_force(rates[d]) for d in directions},
                     "fall_share": {d: list(rates[d].values()) for d in directions},
                     "friction": report["friction"]})
        print(f"{label(report)}: 50% fall force " + ", ".join(
            f"{d} {rows[-1]['half_fall_force_n'][d]}" for d in directions))
    comparisons = [compare(r, m) for r in reports if is_rne(r) for m in reports if not is_rne(m)]
    for c in comparisons:
        split = c["push_disagreements"]
        print(f"{c['rne']} vs {c['mujoco']}: agreement {c['push_agreement']:.1%}, only RNE fell "
              f"{split['only_first_fell']}, only MuJoCo {split['only_second_fell']}")
    effects = []
    for index, a in enumerate(reports):
        for b in reports[index + 1:]:
            if (is_rne(a) == is_rne(b) and has_residual(a) == has_residual(b)
                    and self_collision(a) != self_collision(b)):
                without, with_ = (a, b) if self_collision(b) else (b, a)
                split = disagreements(without, with_)
                effects.append({"without": label(without), "with": label(with_),
                                "only_without_fell": split.pop("only_first_fell"),
                                "only_with_fell": split.pop("only_second_fell"),
                                **{k.replace("second", "with"): v for k, v in split.items()}})
                print(f"self-collision in {label(with_).split(',')[0]}: "
                      f"{effects[-1]['only_with_fell']} trials fall only with it, "
                      f"{effects[-1]['only_without_fell']} only without")
    residual_effects = []
    for index, a in enumerate(reports):
        for b in reports[index + 1:]:
            if (is_rne(a) == is_rne(b) and self_collision(a) == self_collision(b)
                    and has_residual(a) != has_residual(b)):
                plain, trained = (a, b) if has_residual(b) else (b, a)
                split = disagreements(plain, trained)
                residual_effects.append({
                    "without": label(plain), "with": label(trained),
                    "falls_without": sum(t["fell"] for t in plain["push"]),
                    "falls_with": sum(t["fell"] for t in trained["push"]),
                    "only_without_fell": split["only_first_fell"],
                    "only_with_fell": split["only_second_fell"]})
                print(f"residual in {label(plain)}: falls {residual_effects[-1]['falls_without']} "
                      f"-> {residual_effects[-1]['falls_with']}")
    summary = {"schema_version": 2, "robot": first["robot"], "protocol": first["protocol"],
               "reports": rows, "comparisons": comparisons, "self_collision_effects": effects}
    if residual_effects:
        summary["residual_effects"] = residual_effects
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    figure_path.write_text(svg(reports, directions, first["protocol"]["forces_n"], first["robot"]))


if __name__ == "__main__":
    main()
