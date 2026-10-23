"""Standalone scientific figures for the four frozen diagnostic runs."""
import csv
import json
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt

matplotlib.use('Agg')
HERE = Path(__file__).resolve().parent


def read(path):
    """Read every logged risk and truth sample."""
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    """Show velocity, angular motion and states separately from terminal delay."""
    rows = json.loads((HERE/'analysis.json').read_text())[:-1]
    fig, axes = plt.subplots(3, 4, figsize=(16, 9), constrained_layout=True)
    labels = ['normal', 'braking', 'wait', 'dwa_fallback', 'recovering']
    for column, r in enumerate(rows):
        dest = HERE/'analysis'/r['name']
        risk = read(dest/'risk.csv')
        gt = read(dest/'ground_truth.csv')
        t = [float(g['t'])-r['epoch'] for g in gt]
        axes[0, column].set_title(['D01 t2 A', 'D01 t2 B', 'D01 t4', 'D01 t6'][column])
        axes[0, column].plot(t, [float(g['vx']) for g in gt], c='#c44e52', label='actual v')
        axes[0, column].step([float(g['elapsed']) for g in risk],
                             [float(g['cap']) for g in risk], where='post',
                             c='#2676a6', label='Governor cap')
        axes[0, column].set(ylim=(-.02, .30))
        axes[1, column].plot(t, [float(g['wz']) for g in gt], c='#79549e')
        axes[1, column].axhline(0, lw=.5, c='#777')
        axes[2, column].step([float(g['elapsed']) for g in risk],
                             [labels.index(g['state']) for g in risk], where='post',
                             c='#396b54')
        axes[2, column].set(yticks=range(5), yticklabels=['N', 'B', 'W', 'F', 'R'])
        axes[2, column].set_xlabel('Time since dispatch (s)')
        for ax in axes[:, column]:
            ax.set_xlim(0, min(16, r['duration'] + .2))
            ax.grid(alpha=.18)
            if r['physical_contact']:
                ax.axvline(r['first_contact']['stamp']-r['epoch'], c='#d55e00', ls='--')
    axes[0, 0].set_ylabel('Linear speed (m/s)')
    axes[1, 0].set_ylabel('Actual angular speed (rad/s)')
    axes[2, 0].set_ylabel('Risk state')
    axes[0, 0].legend(fontsize=9)
    fig.suptitle('Frozen Scan V2: interaction intervals; dashed line = physical contact', fontsize=15)
    fig.savefig(HERE/'matrix_interaction.png', dpi=160)
    fig.savefig(HERE/'matrix_interaction.pdf')
    plt.close(fig)
    cycles = read(HERE/'analysis'/rows[0]['name']/'control_cycles.csv')
    terminal = [r for r in cycles if r['phase'] == 'rotating']
    fig, axes = plt.subplots(2, 1, figsize=(10, 5), constrained_layout=True)
    t = [float(r['elapsed_estimate']) for r in terminal]
    import ast
    axes[0].plot(t, [abs(float(r['yaw_error'])) for r in terminal], 'o-', ms=3)
    axes[0].axhline(.25, ls='--', c='#c44e52', label='yaw tolerance = 0.25')
    axes[0].set_ylabel('Absolute yaw error (rad)')
    axes[0].legend()
    axes[1].plot(t, [ast.literal_eval(r['input'])[1] for r in terminal], 'o-', ms=3,
                 label='input omega')
    axes[1].plot(t, [ast.literal_eval(r['chosen'])[1] for r in terminal], 'x-',
                 label='chosen omega')
    axes[1].set(ylabel='Angular speed (rad/s)',
                xlabel='Dispatch-aligned estimate from collector wall elapsed (s)')
    axes[1].legend()
    for ax in axes:
        ax.grid(alpha=.18)
    fig.suptitle('t2 A terminal delay: risk released at 11.1s; goal succeeded at 53.7s')
    fig.savefig(HERE/'terminal_t2_a.png', dpi=160)
    fig.savefig(HERE/'terminal_t2_a.pdf')
    plt.close(fig)


if __name__ == '__main__':
    main()
