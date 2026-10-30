"""Plot stored H1 v3 validation costs and penetration; no new simulation."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parents[2]
out = root / "docs/evidence/hard-contact-drop/h1-v3"
report = json.loads((out / "report.json").read_text())
fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), layout="constrained")
x = np.arange(4)
labels = [f"{m:g} kg\n{h:g} m drop" for m, h in report["validation_cases"]]
for backend, offset, color, label in (("native", -.18, "#176b57", "RNE Native, 2 ms"),
                                      ("mujoco", .18, "#c16a22", "MuJoCo, 5 us")):
    rows = [r for r in report["records"] if r["phase"] == "validation" and r["backend"] == backend]
    cost = np.array([r["median_cost"] * 1000 for r in rows])
    bounds = np.array([[r["min_cost"] * 1000 for r in rows], [r["max_cost"] * 1000 for r in rows]])
    bars = axes[0].bar(x + offset, cost, .34, color=color, label=label,
                       yerr=[cost - bounds[0], bounds[1] - cost], capsize=3)
    axes[0].bar_label(bars, labels=[f"{v:.3g}" for v in cost], padding=9, fontsize=9)
    penetration = [r["metrics"]["max_penetration_m"] * 1e6 for r in rows]
    bars = axes[1].bar(x + offset, penetration, .34, color=color)
    axes[1].bar_label(bars, labels=["~0" if v < 1e-6 else f"{v:.3g}" for v in penetration], padding=4, fontsize=9)
for ax in axes:
    ax.set_xticks(x, labels)
    ax.grid(axis="y", alpha=.2)
    ax.set_axisbelow(True)
axes[0].set(yscale="log", ylim=(.7, 1600), ylabel="Backend ms / simulated second (log scale)",
            title="Median and observed min-max, 5 repeats")
axes[0].legend(fontsize=9, loc="lower center", bbox_to_anchor=(.5, 1.0), ncol=2)
axes[0].set_title("")
axes[1].axhline(10, color="#555", linestyle="--", label="10 um gate")
axes[1].set(ylim=(0, 11.5), ylabel="Maximum geometric penetration (um)", title="Both backends pass all quality gates")
axes[1].legend(fontsize=9)
fig.savefig(out / "quality-cost.png", dpi=160)
