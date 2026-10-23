#!/usr/bin/env python3
"""Read saved V1/diagnostic trials; never start ROS or rewrite their raw artifacts."""
import argparse
import bisect
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import statistics

HERE = Path(__file__).resolve().parent
BENCH = HERE.parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_jsonl(path):
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def first(items, predicate):
    return next((item for item in items if predicate(item)), None)


def debug_records(path):
    records = []
    for number, line in enumerate(path.read_text().splitlines(), 1):
        if '[DWA_DEBUG]' not in line:
            continue
        record = {'line': number, 'wall_stamp_s': float(re.search(r'\[(\d+\.\d+)\]', line)[1])}
        for key in ('stage', 'goal_phase'):
            match = re.search(r'\b' + key + r'=([^ ]+)', line)
            record[key] = match[1] if match else None
        for key in ('true_goal_distance', 'terminal_radius', 'yaw_error', 'yaw_tolerance',
                    'sampled', 'valid', 'valid_forward', 'terminal_rejected'):
            match = re.search(r'\b' + key + r'=([^ ]+)', line)
            value = float(match[1]) if match else None
            record[key] = value if value is not None and math.isfinite(value) else None
        for key, pattern in (
                ('odom_pose', r'\bpose=\(([^)]+)\)'),
                ('true_goal_local', r'true_goal_local=\(([^)]+)\)'),
                ('input_velocity', r'input_vel=\(([^)]+)\)'),
                ('rejections', r'reject\[[^\]]+\]=\[([^\]]+)\]'),
                ('timing_ms', r'ms\[[^\]]+\]=\[([^\]]+)\]')):
            match = re.search(pattern, line)
            record[key] = [float(v) if math.isfinite(float(v)) else None
                           for v in match[1].split(',')] if match else None
        match = re.search(r'\bbest=\{present:(\d+),v:([^,]+),w:([^,]+)', line)
        record['best'] = [int(match[1]), float(match[2]), float(match[3])] if match else None
        records.append(record)
    return records


def stats(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    return {'n': len(values), 'mean': statistics.mean(values) if values else None,
            'sample_sd': statistics.stdev(values) if len(values) > 1 else None}


def numeric(row, name):
    return float(row[name]) if row.get(name) else None


def s04_details(path, log, output, label):
    summary = json.loads((path / 'summary.json').read_text())
    metrics = json.loads((path / 'metrics.json').read_text())
    records = debug_records(log) if summary['arguments']['controller'] == 'mydwa' else []
    bt = read_jsonl(path / 'bt_events.jsonl')
    plans = read_jsonl(path / 'plans.jsonl')
    loc = read_jsonl(path / 'localization.jsonl')
    events = read_jsonl(path / 'nav_events.jsonl')
    odom = sorted((p for p in loc if p['source'] == 'odom'), key=lambda p: p['source_stamp_s'])
    stamps = [p['source_stamp_s'] for p in odom]
    rows = list(csv.DictReader((path / 'trajectory.csv').open()))
    goal = summary['arguments']['goal']
    transform_rows = []
    for row in rows:
        stamp = numeric(row, 'tf_stamp_s')
        if stamp is None or numeric(row, 'x') is None or not stamps:
            continue
        index = bisect.bisect_left(stamps, stamp)
        neighbors = odom[max(0, index-1):min(len(odom), index+1)]
        p = min(neighbors, key=lambda p: abs(p['source_stamp_s']-stamp))
        mismatch = p['source_stamp_s']-stamp
        if abs(mismatch) > .025:
            continue
        x, y, yaw = (float(row[k]) for k in ('x', 'y', 'yaw'))
        angle = math.atan2(math.sin(yaw-p['yaw']), math.cos(yaw-p['yaw']))
        cosine, sine = math.cos(angle), math.sin(angle)
        transform_rows.append({
            'tf_stamp_s': stamp, 'row_clock_s': float(row['sim_time_s']),
            'map_x': x, 'map_y': y, 'odom_x': p['x'], 'odom_y': p['y'],
            'goal_distance_m': math.hypot(x-goal[0], y-goal[1]),
            'map_from_odom_x': x-cosine*p['x']+sine*p['y'],
            'map_from_odom_y': y-sine*p['x']-cosine*p['y'],
            'map_from_odom_yaw': angle, 'pair_time_difference_s': mismatch,
            'actual_vx': numeric(row, 'actual_vx'),
            'controller_vx': numeric(row, 'controller_vx')})
    if transform_rows:
        with (output / (label + '_map_odom.csv')).open('w') as stream:
            writer = csv.DictWriter(stream, transform_rows[0].keys())
            writer.writeheader()
            writer.writerows(transform_rows)
    first_terminal = first(records, lambda r: r['goal_phase'] in ('slowing', 'rotating'))
    outside = first(records, lambda r: r['goal_phase'] == 'rotating' and
                    r['true_goal_distance'] is not None and
                    r['true_goal_distance'] > r['terminal_radius'])
    deadlock = first(records, lambda r: r['stage'] == 'output' and
                     r['goal_phase'] == 'rotating' and
                     r['best'] is not None and r['best'][0] == 1 and
                     r['true_goal_distance'] > r['terminal_radius'] and
                     r['best'][1] == 0. and abs(r['best'][2]) < 1e-9)
    # Nearby BT events provide receipt-time labels only, not exact debug sim stamps.
    for record in records:
        if bt:
            nearby = min(bt, key=lambda b: abs(b['bt_internal_stamp_s']-record['wall_stamp_s']))
            record['nearest_bt_receipt_sim_s'] = nearby['receipt_sim_s']
            record['bt_wall_difference_s'] = nearby['bt_internal_stamp_s']-record['wall_stamp_s']
    lines = log.read_text().splitlines()
    failures = [{'line': i, 'text': line} for i, line in enumerate(lines, 1)
                if '[controller_server]' in line and (
                    '[ERROR]' in line or 'No collision-free feasible trajectory' in line or
                    'No candidate velocities' in line)]
    normal = [r for r in records if r['goal_phase'] == 'normal' and r['sampled']]
    endpoint_geometries = {
        tuple(round(v, 6) for v in p['points'][-1]) for p in plans if p['points']}
    result = {'trial': str(path.relative_to(BENCH)), 'log': str(log.relative_to(BENCH)),
              'action_result': summary.get('action_result'), 'status': summary['status'],
              'stop_reason': summary.get('stop_reason'),
              'completion_sim_s': metrics['timing']['first_planner_to_observed_stop_sim_s']
              if metrics['success'] else None,
              'observed_span_sim_s': metrics['timing']['first_planner_to_observed_stop_sim_s'],
              'motion': metrics['motion'], 'final_error': metrics['final_error'],
              'wz_std': metrics['velocity']['actual']['angular']['std'],
              'wz_switches': metrics['velocity']['actual']['angular_direction_switches'],
              'max_feedback_recoveries': summary.get('max_feedback_recoveries'),
              'behavior_goals': metrics['navigation_system']['behavior_goals_executed'],
              'bt_failures': metrics['navigation_system']['bt_node_failure_counts'],
              'first_terminal_debug': first_terminal, 'first_outside_rotating_debug': outside,
              'first_zero_outside_debug': deadlock, 'controller_errors': failures,
              'normal_sampled_cycle_time_ms': stats([r['timing_ms'][0] for r in normal]),
              'debug_record_count': len(records),
              'zero_valid_normal_records': sum(r['valid'] == 0 for r in normal),
              'control_rate_warnings': sum('Control loop missed' in line for line in lines),
              'plan_messages': len(plans), 'unique_plan_endpoints': sorted(endpoint_geometries),
              'amcl_messages': sum(p['source'] == 'amcl' for p in loc),
              'map_odom_pairing': 'Nearest native stamps <=0.025 s; diagnostic approximation, '
              'not a new ground-truth pose or a safety metric',
              'action_transitions': dict(Counter(
                  e['action'] + ':' + str(e['status']) for e in events)),
              'input_sha256': {str(p.relative_to(BENCH)): sha(p) for p in
                               [path / 'summary.json', path / 'metrics.json',
                                path / 'trajectory.csv', path / 'plans.jsonl',
                                path / 'bt_events.jsonl', path / 'nav_events.jsonl',
                                path / 'localization.jsonl', log] if p.exists()}}
    (output / (label + '_debug.json')).write_text(
        json.dumps(records, indent=2, allow_nan=False)+'\n')
    return result


def analyze(suite, output):
    output.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((suite / 'manifest.json').read_text())
    groups, details, input_hashes = {}, [], {}
    for trial in manifest['trials']:
        path = suite / trial['directory']
        if not (path / 'summary.json').exists():
            raise RuntimeError('Incomplete trial: ' + str(path))
        summary = json.loads((path / 'summary.json').read_text())
        metrics = json.loads((path / 'metrics.json').read_text())
        for name, expected in metrics['input_sha256'].items():
            assert sha(path / name) == expected, path / name
        for config in summary.get('config_files', []):
            assert sha(path / config['saved']) == config['sha256']
        input_hashes[str((path / 'summary.json').relative_to(BENCH))] = sha(path / 'summary.json')
        key = trial['controller'] + '/' + trial['scene']
        groups.setdefault(key, []).append(metrics)
        if trial['scene'] == 'S04':
            label = f'{trial["controller"]}_{trial["scene"]}_{trial["repeat"]}'
            log = suite / 'logs' / (label + '_launch.log')
            details.append(s04_details(path, log, output, label))
    aggregated = {}
    for key, trials in groups.items():
        good = [m for m in trials if m['success']]
        aggregated[key] = {'scheduled': len(trials), 'successes': len(good),
                           'completion_sim_s_successes_only': stats([
                               m['timing']['first_planner_to_observed_stop_sim_s'] for m in good]),
                           'failure_statuses': dict(Counter(m['status'] for m in trials
                                                            if not m['success'])),
                           'velocity_peaks': {
                               source + '.' + component: max((
                                   m['velocity'][source][component]['max_abs'] for m in trials
                                   if m['velocity'][source][component]['max_abs'] is not None),
                                   default=None)
                               for source in ('actual', 'controller_command', 'robot_command')
                               for component in ('linear', 'angular')}}
    result = {'suite': str(suite.relative_to(BENCH)), 'groups': aggregated,
              's04': details, 'summary_input_sha256': input_hashes,
              'analysis_source_sha256': sha(Path(__file__))}
    (output / 'analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'groups': aggregated, 's04': [
        {k: d[k] for k in ('trial', 'status', 'completion_sim_s', 'first_outside_rotating_debug',
                           'first_zero_outside_debug', 'normal_sampled_cycle_time_ms')}
        for d in details]}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    analyze(arguments.suite.resolve(), arguments.output.resolve())
