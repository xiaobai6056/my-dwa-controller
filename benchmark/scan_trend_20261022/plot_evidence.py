"""Render source-stamped velocity evidence and actual control traces."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import analyze

HERE = Path(__file__).resolve().parent


def csv_rows(path):
    """Read the analysis table without changing source records."""
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    """Show good and poor motion estimates alongside risk and measured speed."""
    plt.switch_backend('Agg')
    results = json.loads((HERE / 'analysis.json').read_text())
    names = ['round1_verified_D01_t2', 'round1_verified_D01_t4', 'round3_D01_t2']
    fig, axes = plt.subplots(2, 3, figsize=(15, 7), constrained_layout=True)
    details = {}
    for column, name in enumerate(names):
        result = next(r for r in results if r['run'] == name)
        directory = HERE / 'runs' / name
        epoch = result['epoch']
        actor = csv_rows(directory / 'actor_aligned.csv')
        t = np.array([float(r['elapsed']) for r in actor])
        top = axes[0, column]
        top.plot(t, [float(r['world_vy']) for r in actor], '.-', label='estimated world vy')
        top.plot(t, [float(r['world_vx']) for r in actor], '.-', label='estimated world vx')
        start = result['motion_start']
        top.plot([0, start, start, start + 8.8, start + 8.8, 14],
                 [0, 0, .25, .25, 0, 0], color='black', ls='--', label='scenario vy truth')
        top.axhline(0, color='gray', lw=.7)
        top.axvline(result['motion_start'], color='gray', ls=':')
        top.axvline(result['motion_start'] + .8, color='gray', ls=':')
        top.set(xlim=(0, min(14, result['outcome']['navigation_end_sim_s'] - epoch)),
                ylim=(-.45, .7), ylabel='Velocity (m/s)', title=name)
        top.grid(alpha=.2)
        if column == 0:
            top.legend(fontsize=8)
        bottom = axes[1, column]
        risk = csv_rows(directory / 'risk_all.csv')
        bottom.step([float(r['t']) - epoch for r in risk],
                    [float(r['cap']) for r in risk], where='post', label='Governor cap')
        _, _, _, odom = analyze.export_bag(directory)
        bottom.plot([r['stamp'] - epoch for r in odom], [r['v'] for r in odom],
                    label='odom speed', alpha=.8)
        commands = analyze.rows(directory / 'trial/commands.jsonl')
        bottom.plot([r['t'] - epoch for r in commands], [r['vx'] for r in commands],
                    label='/cmd_vel (receipt clock)', alpha=.6)
        for transition in result['transitions'][1:]:
            if transition['t'] > 14:
                continue
            bottom.axvline(transition['t'], ls=':', color='gray')
        bottom.set(xlim=top.get_xlim(), ylim=(-.03, .3),
                   xlabel='Time after task dispatch (s)', ylabel='Linear speed (m/s)')
        bottom.grid(alpha=.2)
        if column == 0:
            bottom.legend(fontsize=8)
        gt = analyze.rows(directory / 'trial/ground_truth.jsonl')
        trigger = result['first_non_normal']['t']
        before = [r['v'] for r in odom if trigger - .3 <= r['stamp'] - epoch < trigger]
        after = [r['v'] for r in odom if trigger + .2 <= r['stamp'] - epoch < trigger + .6]
        details[name] = dict(
            min_gt_polygon_gap=min(g['gap_approx_m'] for g in gt),
            odom_before_trigger=float(np.median(before)) if before else None,
            odom_after_trigger=float(np.median(after)) if after else None,
            first_reduced_command=next((r['t'] - epoch for r in commands
                                        if r['t'] - epoch >= trigger and r['vx'] < .25), None),
            source_stamp_note='Odometry source stamps; cmd_vel and unstamped GT use receipt clock')
    fig.savefig(HERE / 'velocity_and_control.png', dpi=170)
    plt.close(fig)

    name = names[-1]
    directory = HERE / 'runs' / name
    result = next(r for r in results if r['run'] == name)
    epoch = result['epoch']
    gt = analyze.rows(directory / 'trial/ground_truth.jsonl')
    risk = csv_rows(directory / 'risk_all.csv')
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), constrained_layout=True)
    axes[0].plot([g['t'] - epoch for g in gt], [g['robot'][0] for g in gt], label='robot x')
    axes[0].plot([g['t'] - epoch for g in gt], [g['robot'][1] for g in gt], label='robot y')
    axes[0].plot([g['t'] - epoch for g in gt], [g['obstacle'][1] for g in gt],
                 label='obstacle y')
    axes[0].set(ylabel='World position (m)', title='Final D01 2s: no contact, navigation timeout')
    axes[0].legend(ncol=3)
    states = dict(normal=0, limited=1, braking=2, wait=3, dwa_fallback=4, recovering=5)
    axes[1].step([float(r['t']) - epoch for r in risk],
                 [states[r['state']] for r in risk], where='post')
    axes[1].set(yticks=list(states.values()), yticklabels=list(states), ylabel='Governor')
    axes[2].step([float(r['t']) - epoch for r in risk], [float(r['unresolved']) for r in risk],
                 where='post', label='unresolved scan evidence')
    axes[2].step([float(r['t']) - epoch for r in risk], [float(r['known']) for r in risk],
                 where='post', label='full corridor known')
    axes[2].set(xlabel='Time after task dispatch (s)', ylabel='Clear gates')
    axes[2].legend(ncol=2)
    for ax in axes:
        ax.grid(alpha=.2)
        ax.set_xlim(0, 90)
    fig.savefig(HERE / 'final_timeout.png', dpi=170)
    (HERE / 'control_effects.json').write_text(json.dumps(details, indent=2) + '\n')


if __name__ == '__main__':
    main()
