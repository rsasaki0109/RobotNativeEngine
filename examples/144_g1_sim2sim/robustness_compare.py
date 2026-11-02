#!/usr/bin/env python3
"""Compare RNE's and MuJoCo's robustness reports and draw the fall curves.

For each push direction it tabulates the share of the eight gait phases that
fell at each force, the 50% fall force (linearly interpolated between the
grid forces where the share first reaches one half), and the agreement of
the two engines trial by trial. Of the trials where only the other engine
fell, it counts those in which robot bodies touched each other, if the other
report lists such contacts. The friction sweep is compared by fall and
forward speed. Writes a JSON summary and an SVG figure; uses no third-party
packages.

Usage: robustness_compare.py RNE.json MUJOCO.json SUMMARY.json FIGURE.svg
       [--variant MUJOCO_VARIANT.json LABEL]

A variant report (for example MuJoCo without self-collision) is compared
with RNE the same way and drawn dashed, and the trials it changes within
MuJoCo are counted.
"""
import json
import sys
from collections import defaultdict
from pathlib import Path


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


def agreement(rne, mujoco):
    key = lambda t: (t["direction"], t["force_n"], t["phase"])
    other = {key(t): t["fell"] for t in mujoco["push"]}
    same = sum(other[key(t)] == t["fell"] for t in rne["push"])
    return same / len(rne["push"])


def disagreements(first, second, names=("rne", "other")):
    """Counts the trials where only one report fell, and the second report's
    self-contacts in the trials where only it fell, if it lists them."""
    key = lambda t: (t["direction"], t["force_n"], t["phase"])
    first_fell = {key(t): t["fell"] for t in first["push"]}
    only_second = [t for t in second["push"] if t["fell"] and not first_fell[key(t)]]
    only_first = [t for t in second["push"] if first_fell[key(t)] and not t["fell"]]
    a, b = names
    result = {f"only_{a}_fell": len(only_first), f"only_{b}_fell": len(only_second)}
    if all("self_contacts" in t for t in second["push"]):
        pairs = defaultdict(int)
        for trial in only_second:
            for pair in trial["self_contacts"]:
                pairs[" / ".join(pair)] += 1
        result[f"only_{b}_fell_with_self_contact"] = sum(
            bool(t["self_contacts"]) for t in only_second)
        result["self_contact_pairs"] = dict(sorted(pairs.items(), key=lambda kv: -kv[1]))
    return result


COLORS = {"RNE": "#2563eb", "MuJoCo": "#ea580c"}


def svg(curves, directions, forces, robot):
    """`curves` is a list of (label, color, dashed, rates by direction)."""
    width, height, margin = 960, 300, 46
    panel = (width - margin) / len(directions)
    fmax = max(forces)
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height + 70}" '
             f'font-family="sans-serif" font-size="12">',
             f'<rect width="100%" height="100%" fill="white"/>',
             f'<text x="{margin}" y="18" font-size="14">{robot.upper()}: share of 8 gait '
             f'phases that fell after a 0.1 s pelvis push, by force</text>']
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
        for _, color, dashed, rates in curves:
            points = " ".join(f"{x0 + f / fmax * w:.1f},{bottom - r * (bottom - top):.1f}"
                              for f, r in [(0.0, 0.0)] + list(rates[direction].items()))
            dash = ' stroke-dasharray="5 4"' if dashed else ""
            parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" '
                         f'stroke-width="2"{dash}/>')
    parts.append(f'<text x="{width / 2}" y="{height + 36}" text-anchor="middle">push force (N), '
                 f'held 0.1 s on the pelvis origin while walking at 0.5 m/s</text>')
    x = margin + 10
    for name, color, dashed, _ in curves:
        dash = ' stroke-dasharray="5 4"' if dashed else ""
        parts.append(f'<line x1="{x}" x2="{x + 24}" y1="{height + 56}" y2="{height + 56}" '
                     f'stroke="{color}" stroke-width="3"{dash}/>')
        parts.append(f'<text x="{x + 30}" y="{height + 60}">{name}</text>')
        x += 40 + 7 * len(name)
    parts.append("</svg>")
    return "\n".join(parts) + "\n"


def compare(rne, other, directions):
    rne_rates, other_rates = fall_rates(rne), fall_rates(other)
    pushes = []
    for direction in directions:
        pushes.append({"direction": direction,
                       "rne_half_fall_force_n": half_force(rne_rates[direction]),
                       "other_half_fall_force_n": half_force(other_rates[direction]),
                       "rne_fall_share": list(rne_rates[direction].values()),
                       "other_fall_share": list(other_rates[direction].values())})
    friction = [{"friction": a["friction"],
                 "rne_fell": a["summary"]["fell"], "other_fell": b["summary"]["fell"],
                 "rne_forward_speed_m_s": a["summary"]["second_half_forward_speed_m_s"],
                 "other_forward_speed_m_s": b["summary"]["second_half_forward_speed_m_s"]}
                for a, b in zip(rne["friction"], other["friction"])]
    return {"other_engine": other["engine"],
            "other_self_collision": other.get("self_collision", True),
            "push": pushes, "push_agreement": agreement(rne, other),
            "push_disagreements": disagreements(rne, other), "friction": friction}


def report(label, result):
    print(f"RNE vs {label}: trial-by-trial agreement on falling "
          f"{result['push_agreement']:.1%}")
    split = result["push_disagreements"]
    line = (f"  only RNE fell in {split['only_rne_fell']} trials, only {label} in "
            f"{split['only_other_fell']}")
    if "only_other_fell_with_self_contact" in split:
        line += f" ({split['only_other_fell_with_self_contact']} with robot self-contact)"
    print(line)
    for row in result["push"]:
        print(f"  {row['direction']:9s} 50% fall force RNE {row['rne_half_fall_force_n']}  "
              f"{label} {row['other_half_fall_force_n']}")
    for row in result["friction"]:
        print(f"  friction {row['friction']:<5} RNE fell={row['rne_fell']!s:5} "
              f"v={row['rne_forward_speed_m_s']:+.3f}  {label} fell={row['other_fell']!s:5} "
              f"v={row['other_forward_speed_m_s']:+.3f}")


def main():
    args = sys.argv[1:]
    variant = None
    if "--variant" in args:
        at = args.index("--variant")
        variant = (Path(args[at + 1]), args[at + 2])
        del args[at:at + 3]
    rne_path, mujoco_path, summary_path, figure_path = map(Path, args[:4])
    rne = json.loads(rne_path.read_text())
    mujoco = json.loads(mujoco_path.read_text())
    others = [("MuJoCo", mujoco)]
    if variant:
        others.append((variant[1], json.loads(variant[0].read_text())))
    for _, other in others:
        if other["protocol"] != rne["protocol"] or other["robot"] != rne["robot"]:
            raise SystemExit("reports use different protocols")
    directions = [d["name"] for d in rne["protocol"]["directions"]]
    comparisons = {}
    for label, other in others:
        comparisons[label] = compare(rne, other, directions)
        report(label, comparisons[label])
    summary = {"schema_version": 1, "robot": rne["robot"], "rne_engine": rne["engine"],
               "protocol": rne["protocol"], "comparisons": comparisons}
    if variant:
        # What the variant changes within MuJoCo, trial by trial.
        effect = disagreements(others[1][1], mujoco, ("variant", "mujoco"))
        summary["variant_effect"] = {"variant": variant[1], **effect}
        print(f"{variant[1]} vs MuJoCo: only MuJoCo fell in {effect['only_mujoco_fell']} "
              f"trials, only the variant in {effect['only_variant_fell']}")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    curves = [("RNE", COLORS["RNE"], False, fall_rates(rne)),
              ("MuJoCo (MJCF as shipped)", COLORS["MuJoCo"], False, fall_rates(mujoco))]
    if variant:
        curves.append((variant[1], COLORS["MuJoCo"], True, fall_rates(others[1][1])))
    figure_path.write_text(svg(curves, directions, rne["protocol"]["forces_n"], rne["robot"]))


if __name__ == "__main__":
    main()
