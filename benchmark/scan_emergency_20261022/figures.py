"""Plot accepted motion separately from raw surface fits and actual control."""
import csv
import json
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use('Agg')

HERE = Path(__file__).resolve().parent


def read_csv(path):
    """Read preserved numeric diagnostics without filtering unknown intervals."""
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    """Save standalone figures and the contiguous pre-switch comparison."""
    run = HERE / 'runs/round2_D01_t2'
    result = next(r for r in json.loads((HERE / 'analysis.json').read_text())
                  if r['run'] == run.name)
    actor = read_csv(run / 'actor_aligned.csv')
    risk = read_csv(run / 'risk_all.csv')
    gt = read_csv(run / 'ground_truth_aligned.csv')
    epoch = result['epoch']
    elapsed = np.array([float(r['elapsed']) for r in actor])
    vx = np.array([float(r['world_vx']) for r in actor])
    vy = np.array([float(r['world_vy']) for r in actor])
    accepted = np.array([r['motion'] == 'moving' for r in actor])
    early = accepted & (elapsed < 5)
    metric = dict(n=int(sum(early)), elapsed=[min(elapsed[early]), max(elapsed[early])],
                  max_gap=float(max(np.diff(elapsed[early]))),
                  vx_median=float(np.median(vx[early])),
                  vy_median=float(np.median(vy[early])),
                  rmse=float(np.sqrt(np.mean(vx[early]**2 + (vy[early]-.25)**2))))
    (HERE / 'continuous_motion.json').write_text(json.dumps(metric, indent=2) + '\n')
    figure, axes = plt.subplots(3, 1, figsize=(10, 9), constrained_layout=True)
    axes[0].scatter(elapsed[~accepted], vx[~accepted], s=18, c='#aaa', label='raw vx: unknown')
    axes[0].scatter(elapsed[accepted], vx[accepted], s=24, c='#e67e22', label='accepted vx')
    axes[0].scatter(elapsed[accepted], vy[accepted], s=24, c='#1368aa', label='accepted vy')
    axes[0].axhline(.25, c='#1368aa', ls='--', label='truth vy = 0.25 m/s')
    axes[0].axhline(0, c='#777', lw=.7)
    axes[0].set(ylabel='World velocity (m/s)', xlim=(0, 13),
                title='D01 2s: short transverse trend, explicit unknown surface fits')
    axes[0].legend(loc='lower left', fontsize=9, ncol=2)
    t = [float(r['t'])-epoch for r in risk]
    axes[1].step(t, [float(r['cap']) for r in risk], where='post', label='Governor speed cap')
    axes[1].plot([float(r['elapsed']) for r in gt], [float(r['vx']) for r in gt],
                 c='#d1495b', label='measured robot vx')
    axes[1].set(ylabel='Linear speed (m/s)', xlim=(0, 16))
    for at, label in [(3, 'brake'), (6.1, 'stopped'), (11, 'local clear'), (11.7, 'release')]:
        axes[1].axvline(at, ls=':', c='#777', lw=.9)
        axes[1].text(at+.05, .29, label, fontsize=8, rotation=90, va='top')
    axes[1].legend(loc='lower left', fontsize=9)
    labels = ['normal', 'braking', 'dwa_fallback', 'recovering']
    axes[2].step(t, [labels.index(r['state']) for r in risk], where='post', c='#5a4780')
    axes[2].set(yticks=range(len(labels)), yticklabels=labels, xlim=(0, 24),
                xlabel='Time since navigation dispatch (s)', ylabel='Governor state')
    for ax in axes:
        ax.grid(alpha=.18)
    figure.savefig(HERE / 'motion_and_control.png', dpi=170)
    figure.savefig(HERE / 'motion_and_control.pdf')
    plt.close(figure)


if __name__ == '__main__':
    main()
