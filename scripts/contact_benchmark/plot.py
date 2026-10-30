"""Plot the recorded H1 quality/cost samples; does not rerun or fit physics."""
import lzma
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "docs/evidence/hard-contact-drop"
report = json.loads(lzma.decompress((OUT / "h1-v1.json.xz").read_bytes()))
fig, ax = plt.subplots(figsize=(9, 5), layout="constrained")
for backend, color, marker in (("native", "#176b57", "o"), ("mujoco", "#c16a22", "x")):
    rows = [r for r in report["results"] if r["backend"] == backend]
    for i, r in enumerate(rows):
        x = r["median_cost"] * 1000
        y = max(r["metrics"]["max_penetration_m"] * 1e6, 1e-4)
        ax.errorbar(x, y, xerr=[[max(0, x - r["min_cost"] * 1000)],
                               [max(0, r["max_cost"] * 1000 - x)]],
                    fmt=marker, color=color, alpha=.65, markersize=6,
                    label=backend if i == 0 else None)
ax.axhline(10, linestyle="--", color="#555555", label="10 micrometer penetration gate")
ax.set(xscale="log", yscale="log", xlabel="Backend milliseconds per simulated second (median, min-max)",
       ylabel="Maximum geometric penetration (micrometers)",
       title="H1: one frictionless sphere on a floor; 48 settings")
ax.text(.02, .10, "Penetrations below 0.0001 micrometers are plotted at that floor.\n"
        "Other quality gates also apply; failing points are retained.",
        transform=ax.transAxes, fontsize=9)
ax.grid(alpha=.2)
ax.legend(fontsize=9)
fig.savefig(OUT / "quality-cost.png", dpi=160)
