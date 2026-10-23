"""Summarize every retained comparison run without treating it as a paired replay."""

import csv
import json
from pathlib import Path
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent


def csv_rows(path):
    """Read the existing collector's table."""
    with path.open() as stream:
        return list(csv.DictReader(stream))


def save_csv(path, rows):
    """Write only derived tables next to this script."""
    if not rows:
        return
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def candidate(line, name):
    """Extract compact chosen/unconstrained-forward diagnostic observations."""
    block = re.search(name + r'=\{([^}]+)}', line).group(1)
    return {key: float(re.search(key + r':([^,}]+)', block).group(1))
            for key in ['present', 'v', 'w', 'total']}


def analyze_run(directory):
    """Separate map/state evidence, sparse selector evidence and actual outcomes."""
    manifest = json.loads((directory / 'manifest.json').read_text())
    summary = json.loads((directory / 'trial/summary.json').read_text())
    epoch = summary['navigation_start_sim_s']
    risks, selections, score_examples, timings = [], [], [], []
    for index, line in enumerate((directory / 'launch.log').read_text().splitlines(), 1):
        if '[PATH_CONFLICT]' in line:
            fields = dict(re.findall(r'(\w+)=([^\s]+)', line.split('[PATH_CONFLICT]')[1]))
            risks.append(dict(line=index, relative_s=float(fields['t']) - epoch, **fields))
        if '[FALLBACK_SELECTION]' in line:
            fields = dict(re.findall(r'(\w+)=([^\s]+)',
                                     line.split('[FALLBACK_SELECTION]')[1]))
            v, w = map(float, fields['selected'].strip('()').split(','))
            row = dict(line=index, relative_s=float(fields['t']) - epoch,
                       input_v=float(fields['input']), cap=float(fields['preferred_cap']),
                       preferred_valid=int(fields['preferred_valid']), valid=int(fields['valid']),
                       sampled=int(fields['sampled']), selected_v=v, selected_w=w,
                       accelerating_escape=int(fields['accelerating_escape']))
            assert ((v <= row['cap'] + 1e-6 and not row['accelerating_escape']) or
                    (row['preferred_valid'] == 0 and row['accelerating_escape']))
            selections.append(row)
        if '[DWA_DEBUG]' in line and 'fallback_preference[' in line:
            if 'stage=output' not in line:
                continue
            chosen, forward = candidate(line, 'best'), candidate(line, 'best_forward')
            speed = float(re.search(r'input_vel=\(([^,]+)', line).group(1))
            if forward['v'] > speed + 1e-6 and forward['total'] < chosen['total']:
                score_examples.append(dict(line=index, input_v=speed, chosen=chosen,
                                           lower_score_forward=forward))
            match = re.search(r'ms\[total,[^]]+]=\[([^,]+)', line)
            if match:
                timings.append(float(match.group(1)))
    contacts_path = directory / 'trial/contacts.jsonl'
    contacts = ([json.loads(x) for x in contacts_path.read_text().splitlines()]
                if contacts_path.exists() else None)
    contact = (min(x['stamp'] for x in contacts) - epoch if contacts else None)
    transitions = []
    for row in risks:
        if not transitions or row['state'] != transitions[-1]['state']:
            transitions.append(dict(relative_s=row['relative_s'], state=row['state'],
                                    line=row['line']))
    save_csv(HERE / (directory.name + '_risk.csv'), risks)
    save_csv(HERE / (directory.name + '_selections.csv'), selections)
    return dict(run=directory.name, case=manifest['case'], epoch=epoch,
                valid=summary.get('valid'), action_result=summary.get('action_result'),
                stop_reason=summary.get('stop_reason'), contact_relative_s=contact,
                contact_topic_saved=contacts is not None,
                runtime_mapping_verified=manifest.get('runtime_mapping_verified'),
                cleanup_returncode=manifest.get('cleanup_returncode'),
                plugin_sha256=manifest['plugin_sha256'], transitions=transitions,
                selections=selections, selector_checks=len(selections),
                accelerating_escapes=sum(x['accelerating_escape'] for x in selections),
                score_preference_examples=score_examples,
                sparse_fallback_output_ms=timings)


def plot_velocity(ax, directory, result):
    """Use odometry source stamps and command receipt clocks without equating them."""
    rows = csv_rows(directory / 'trial/trajectory.csv')
    epoch = result['epoch']
    odom = {float(r['odom_stamp_s']): float(r['actual_vx']) for r in rows
            if r['odom_stamp_s'] and float(r['odom_stamp_s']) >= epoch}
    commands = [(float(r['controller_cmd_receipt_sim_s']) - epoch, float(r['controller_vx']))
                for r in rows if r['controller_vx'] and
                float(r['controller_cmd_receipt_sim_s']) >= epoch]
    ax.plot([t - epoch for t in sorted(odom)], [odom[t] for t in sorted(odom)],
            color='#333333', label='Odometry: source stamp')
    ax.plot([t for t, _ in commands], [v for _, v in commands],
            color='#2c6ab1', label='cmd_vel_nav: receipt clock')
    if result['contact_relative_s'] is not None:
        ax.axvline(result['contact_relative_s'], color='#bd3b3b', label='Contact stamp')
    for transition in result['transitions']:
        if transition['state'] == 'dwa_fallback':
            ax.axvline(transition['relative_s'], color='#ba8e30', linestyle='--',
                       label='First fallback')
            break
    if result['selections']:
        ax.scatter([x['relative_s'] for x in result['selections']],
                   [x['selected_v'] for x in result['selections']], color='#27965b', s=16,
                   label='Sparse fallback selection')
    ax.set(title=directory.name, ylabel='Linear velocity (m/s)',
           xlabel='Seconds after navigation dispatch', ylim=(-0.015, 0.30), xlim=(3.5, 7.5))
    ax.grid(alpha=0.2)
    ax.legend(loc='upper right', fontsize=7)


def main():
    """Include all runs, including a non-triggering repeat and every collision."""
    results = [analyze_run(directory) for directory in sorted((HERE / 'runs').iterdir())
               if (directory / 'trial/summary.json').exists()]
    (HERE / 'results.json').write_text(json.dumps(results, indent=2) + '\n')
    save_csv(HERE / 'results.csv', [{k: r[k] for k in [
        'run', 'case', 'valid', 'action_result', 'stop_reason', 'contact_relative_s',
        'selector_checks', 'accelerating_escapes', 'runtime_mapping_verified',
        'cleanup_returncode', 'plugin_sha256']} for r in results])
    index = {r['run']: r for r in results}
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), constrained_layout=True)
    for row, case in enumerate(['D01_t2', 'D01_t4']):
        for column, tag in enumerate(['control', 'preference']):
            name = tag + '_' + case
            if name in index:
                plot_velocity(axes[row, column], HERE / 'runs' / name, index[name])
    fig.suptitle('Separate trials: compare responses, not a synchronized counterfactual',
                 fontsize=12)
    fig.savefig(HERE / 'velocity_comparison.png', dpi=160)
    plt.close(fig)
    selection_rows = [dict(run=r['run'], **row) for r in results for row in r['selections']]
    if selection_rows:
        fig, ax = plt.subplots(figsize=(8, 4), constrained_layout=True)
        for run in sorted(set(x['run'] for x in selection_rows)):
            rows = [x for x in selection_rows if x['run'] == run]
            ax.scatter([x['cap'] for x in rows], [x['selected_v'] for x in rows], label=run)
        ax.plot([0, 0.27], [0, 0.27], color='#777777', linestyle='--', label='No acceleration')
        ax.set(xlabel='Preferred cap = min(current velocity, hard cap) (m/s)',
               ylabel='Selected linear velocity (m/s)', title='Recorded fallback selections only')
        ax.legend(fontsize=8)
        ax.grid(alpha=0.2)
        fig.savefig(HERE / 'selection_evidence.png', dpi=160)
        plt.close(fig)
    print(json.dumps([{k: r[k] for k in [
        'run', 'action_result', 'stop_reason', 'contact_relative_s', 'selector_checks',
        'accelerating_escapes']} for r in results], indent=2))


if __name__ == '__main__':
    main()
