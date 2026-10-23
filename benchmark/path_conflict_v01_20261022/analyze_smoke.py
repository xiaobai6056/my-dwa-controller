"""Summarize all retained smoke outcomes without selecting successful repeats."""

import csv
import json
from pathlib import Path
import re
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = Path(__file__).resolve().parent
STATES = ['normal', 'limited', 'braking', 'wait', 'recovering', 'dwa_fallback']


def number(value):
    """Keep non-finite region sentinels out of JSON outputs."""
    if value in ('inf', '-inf', 'nan'):
        return None
    return float(value)


def read_events(directory, epoch, end):
    """Read observation/state-change logs; these are not every controller cycle."""
    events = []
    for line in (directory / 'launch.log').read_text(errors='replace').splitlines():
        if '[PATH_CONFLICT]' not in line:
            continue
        fields = dict(re.findall(r'(\w+)=([^\s]+)', line.split('[PATH_CONFLICT]', 1)[1]))
        if not epoch <= float(fields['t']) <= end:
            continue
        record = dict(run=directory.name, relative_s=float(fields['t']) - epoch,
                      observation_time=float(fields['t']), state=fields['state'])
        for key in ('map', 'kind', 'known', 'aligned', 'history', 'new', 'inner', 'outer',
                    'wait', 'cannot_stop'):
            record[key] = int(fields[key])
        for key in ('cap', 'applied', 'stop', 'available', 'ms'):
            record[key] = number(fields[key])
        record['known_inner_distance'] = number(fields['prefix']) if 'prefix' in fields else None
        record['inner_core'], record['outer_core'] = map(int, fields['core'].split('/'))
        record['region_start'], record['region_end'] = map(number, fields['region'].split(':'))
        events.append(record)
    return events


def write_csv(path, records):
    """Write a structured derived table alongside immutable raw run data."""
    if not records:
        return
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)


def summarize(directory):
    """Retain status, code identity, first risk times and recorded computation cost."""
    manifest = json.loads((directory / 'manifest.json').read_text())
    summary = json.loads((directory / 'trial/summary.json').read_text())
    epoch, end = summary['navigation_start_sim_s'], summary['navigation_end_sim_s']
    events = read_events(directory, epoch, end)
    contact_file = directory / 'trial/contacts.jsonl'
    contacts = [json.loads(line) for line in contact_file.read_text().splitlines()] \
        if contact_file.exists() else []
    contact = min((row['stamp'] for row in contacts), default=None)
    observed_ms = sorted(row['ms'] for row in events)
    result = dict(
        run=directory.name, valid=summary['valid'], action_result=summary['action_result'],
        termination=summary.get('stop_reason') or summary['status'],
        duration_s=end - epoch, contact_relative_s=None if contact is None else contact - epoch,
        contact_recording_present=contact_file.exists(),
        plugin_sha256=manifest['plugin_sha256'],
        runtime_mapping_verified=manifest.get('runtime_mapping_verified', False),
        cleanup_returncode=manifest.get('cleanup_returncode'),
        observation_log_records=len(events),
        module_ms_median=statistics.median(observed_ms) if observed_ms else None,
        module_ms_p95=observed_ms[int(0.95 * (len(observed_ms) - 1))] if observed_ms else None,
        module_ms_max=max(observed_ms, default=None))
    for state in STATES:
        result['first_' + state + '_s'] = next(
            (row['relative_s'] for row in events if row['state'] == state), None)
    write_csv(directory / 'derived_risk_events.csv', events)
    return result, events, summary


def plot_current(results):
    """Show final-code failures and actual speed response, without claiming avoidance."""
    figure, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
    colors = {'limited': '#d9aa18', 'braking': '#e18336', 'wait': '#75a772',
              'recovering': '#58acbc', 'dwa_fallback': '#d74d4d'}
    for ax, name in zip(axes, ['prefix_fix_D01_t2', 'prefix_fix_D01_t4']):
        result, events, summary = results[name]
        directory = HERE / 'runs' / name
        epoch, end = summary['navigation_start_sim_s'], summary['navigation_end_sim_s']
        with (directory / 'trial/trajectory.csv').open() as stream:
            rows = [r for r in csv.DictReader(stream) if epoch <= float(r['sim_time_s']) <= end]
        for key, label, color in [('controller_vx', 'Controller command', '#2758a1'),
                                  ('actual_vx', 'Odometry vx', '#222222')]:
            samples = [(float(r['sim_time_s']) - epoch, float(r[key])) for r in rows if r[key]]
            ax.plot([t for t, _ in samples], [v for _, v in samples], label=label, color=color)
        for index, event in enumerate(events):
            stop = events[index + 1]['relative_s'] if index + 1 < len(events) else end - epoch
            if event['state'] in colors:
                ax.axvspan(event['relative_s'], stop, color=colors[event['state']], alpha=0.13)
        for state in ('limited', 'braking', 'dwa_fallback'):
            first = result['first_' + state + '_s']
            if first is not None:
                ax.axvline(first, color=colors[state], linestyle='--', linewidth=1,
                           label=f'{state}: {first:.3f}s')
        if result['contact_relative_s'] is not None:
            ax.axvline(result['contact_relative_s'], color='red', linewidth=1.5,
                       label=f"Contact: {result['contact_relative_s']:.3f}s")
        ax.set(title=name + ' | collision retained', ylabel='Linear speed (m/s)',
               xlabel='Seconds after navigation dispatch', ylim=(-0.03, 0.30))
        ax.grid(alpha=0.2)
        ax.legend(loc='lower left', fontsize=8, ncol=2)
    figure.savefig(HERE / 'risk_response.png', dpi=160)
    plt.close(figure)


def main():
    """Analyze every run in this task directory, including earlier implementation failures."""
    results = {p.name: summarize(p) for p in sorted((HERE / 'runs').iterdir())}
    table = [value[0] for value in results.values()]
    (HERE / 'results.json').write_text(json.dumps(table, indent=2, allow_nan=False) + '\n')
    write_csv(HERE / 'results.csv', table)
    plot_current(results)
    print(json.dumps(table, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
