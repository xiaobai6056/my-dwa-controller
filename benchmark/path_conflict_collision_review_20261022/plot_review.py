"""Plot recorded velocity responses separately from governor-only evidence replay."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

HERE = Path(__file__).resolve().parent
OLD = HERE.parent / 'path_conflict_v01_20261022/runs'
CASES = ['prefix_fix_D01_t2', 'prefix_fix_D01_t4']
COLORS = {'normal': '#4980ad', 'limited': '#ceaa36', 'braking': '#df8e48',
          'dwa_fallback': '#c65b5b'}


def read(path):
    """Load a derived CSV."""
    with path.open() as stream:
        return list(csv.DictReader(stream))


def state_bands(ax, rows, epoch, end, y):
    """Merge repeated observations without inventing unrecorded intermediate states."""
    start, state = None, None
    for row in [*rows, {'stamp': str(epoch + end), 'state': None}]:
        t = float(row['stamp']) - epoch
        if state is not None and row['state'] != state:
            ax.broken_barh([(start, t - start)], (y, 0.35), facecolors=COLORS[state])
        if row['state'] != state:
            start, state = t, row['state']


def main():
    """Keep actual measurements on the left and replayed state labels on the right."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True,
                             gridspec_kw={'width_ratios': [1.7, 1]})
    for index, case in enumerate(CASES):
        summary = json.loads((OLD / case / 'trial/summary.json').read_text())
        epoch = summary['navigation_start_sim_s']
        contacts = [json.loads(x) for x in
                    (OLD / case / 'trial/contacts.jsonl').read_text().splitlines()]
        contact = min(x['stamp'] for x in contacts) - epoch
        trajectory = read(HERE / (case + '_trajectory.csv'))
        odom = {float(r['odom_stamp_s']): float(r['actual_vx']) for r in trajectory
                if r['odom_stamp_s'] and epoch <= float(r['odom_stamp_s']) <= epoch + contact}
        commands = [(float(r['controller_cmd_receipt_sim_s']) - epoch, float(r['controller_vx']))
                    for r in trajectory if r['controller_vx'] and
                    epoch <= float(r['controller_cmd_receipt_sim_s']) <= epoch + contact]
        velocity_ax, state_ax = axes[index]
        velocity_ax.plot([t - epoch for t in sorted(odom)], [odom[t] for t in sorted(odom)],
                         color='#222222', label='Odometry (source stamp)')
        velocity_ax.plot([t for t, _ in commands], [v for _, v in commands],
                         color='#2b64ae', label='cmd_vel_nav (receipt clock)')
        velocity_ax.axvline(contact, color='red', label=f'Contact {contact:.3f}s')
        velocity_ax.set(title=case.replace('prefix_fix_', '') + ' | recorded collision',
                        ylabel='Linear speed (m/s)', xlabel='Seconds after navigation dispatch',
                        ylim=(-0.015, 0.30))
        velocity_ax.grid(alpha=0.2)
        velocity_ax.legend(fontsize=8, loc='lower left')
        before = read(HERE / (case + '_before_mid.csv'))
        after = read(HERE / (case + '_after_mid.csv'))
        state_bands(state_ax, before, epoch, contact, 0.65)
        state_bands(state_ax, after, epoch, contact, 0.10)
        state_ax.axvline(contact, color='red')
        state_ax.set(title='State labels: recorded-evidence replay',
                     yticks=[0.275, 0.825], yticklabels=['Fixed replay', 'Original'],
                     xlabel='Seconds after navigation dispatch', xlim=(0, contact + 0.1),
                     ylim=(-0.05, 1.2))
        state_ax.grid(axis='x', alpha=0.2)
    fig.legend(handles=[Patch(facecolor=color, label=state) for state, color in COLORS.items()],
               loc='upper center', ncol=4, fontsize=8, bbox_to_anchor=(0.75, 1.035))
    fig.savefig(HERE / 'state_velocity_review.png', dpi=160, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    main()
