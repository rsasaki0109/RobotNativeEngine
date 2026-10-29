#!/usr/bin/env python3
"""Regenerate SO-101 comparison figures from the committed measurement report."""
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'examples/116_robot_workbench/python/results/so101_cross_sim.json.gz'
OUT = ROOT / 'docs/media'
report = json.loads(gzip.decompress(SOURCE.read_bytes()))
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'svg.hashsalt': 'so101-cross-sim-v1'})
colors = ['#176b87', '#db7138']
names = report['joint_names']
fig, ax = plt.subplots(figsize=(10.8, 4.2), layout='constrained')
x = np.arange(len(names))
for offset, dt, color in zip([-.19, .19], [1_000_000, 500_000], colors):
    vals = [max(next(r for r in report['results'] if r['dt_ticks'] == dt and r['driven_joint'] == name)['comparison']['max_abs_difference_rad']) * 1000 for name in names]
    bars = ax.bar(x + offset, vals, .36, color=color, label=f'{dt / 1e6:g} ms timestep')
    ax.bar_label(bars, labels=[f'{v:.3g}' for v in vals], padding=4, fontsize=10)
ax.set_yscale('log')
ax.set_ylim(.001, 3)
ax.set_ylabel('Maximum angle difference (mrad, log scale)')
ax.set_xticks(x, [n.replace('_', '\n') for n in names])
ax.set_xlabel('Driven joint; maximum covers every observed joint and sample')
ax.grid(axis='y', alpha=.15)
ax.set_axisbelow(True)
ax.legend(frameon=False, loc='upper left')
fig.savefig(OUT / 'so101-mujoco-response.svg', metadata={'Date': None})
fig.savefig(OUT / 'so101-mujoco-response.png', dpi=160)
plt.close(fig)
fig, axes = plt.subplots(2, 1, figsize=(10.8, 4.8), sharex=True, layout='constrained', gridspec_kw={'height_ratios': [2, 1]})
idx = names.index('wrist_roll')
for dt, style, alpha in [(1_000_000, '-', 1), (500_000, '--', .8)]:
    r = next(r for r in report['results'] if r['dt_ticks'] == dt and r['driven_joint'] == 'shoulder_pan')
    times = np.arange(len(r['rne']['q_rad'])) * dt / 1e6
    for backend, color, label in [('rne', colors[0], 'RNE / Rapier'), ('mujoco', colors[1], 'MuJoCo 3.9')]:
        axes[0].plot(times, np.asarray(r[backend]['q_rad'])[:, idx] * 1000, style, color=color, alpha=alpha,
                     label=f'{label}, {dt / 1e6:g} ms')
    delta = (np.asarray(r['rne']['q_rad'])[:, idx] - np.asarray(r['mujoco']['q_rad'])[:, idx]) * 1000
    axes[1].plot(times, delta, style, color='#77509c', label=f'{dt / 1e6:g} ms')
axes[0].set_ylabel('Wrist-roll angle (mrad)')
axes[0].legend(frameon=False, ncol=2, fontsize=10)
axes[1].set_ylabel('RNE - MuJoCo\n(mrad)')
axes[1].set_xlabel('Simulation time (ms); shoulder-pan input: +0.001 N m at 20-60 ms, -0.001 N m at 60-100 ms')
for ax in axes:
    ax.axvspan(20, 60, color='#176b87', alpha=.05)
    ax.axvspan(60, 100, color='#db7138', alpha=.05)
    ax.grid(alpha=.15)
fig.savefig(OUT / 'so101-mujoco-pan-residual.svg', metadata={'Date': None})
fig.savefig(OUT / 'so101-mujoco-pan-residual.png', dpi=160)
plt.close(fig)
for path in OUT.glob('so101-mujoco-*.svg'):
    path.write_text('\n'.join(line.rstrip() for line in path.read_text().splitlines()) + '\n')
print('Wrote two figures from', SOURCE.relative_to(ROOT))
