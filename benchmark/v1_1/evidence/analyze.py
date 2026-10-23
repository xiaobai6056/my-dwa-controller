"""Offline evidence audit for the finite V1.1 regression; raw trials are read-only."""
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import statistics
import sys
import xml.etree.ElementTree as ET

import yaml

HERE = Path(__file__).resolve().parent
BENCH = HERE.parents[1]
PACKAGE = BENCH.parent
sys.path.insert(0, str(BENCH / 'tuning_r1'))
from analyze import debug_records  # noqa: E402
sys.path.insert(0, str(BENCH))
from run_suite import fairness_signature  # noqa: E402


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    values = [v for v in values if v is not None]
    return dict(n=len(values), mean=statistics.mean(values) if values else None,
                sample_sd=statistics.stdev(values) if len(values) > 1 else None)


def row_for(suite, trial, label):
    path = suite / trial['directory']
    summary = json.loads((path / 'summary.json').read_text())
    metrics = json.loads((path / 'metrics.json').read_text())
    for name, expected in metrics['input_sha256'].items():
        assert sha(path / name) == expected, path / name
    for config in summary['config_files']:
        assert sha(path / config['saved']) == config['sha256']
    expected_file = 'nav2_params.yaml' if trial['controller'] == 'mydwa' else 'nav2_params_dwb.yaml'
    expected = yaml.safe_load((PACKAGE / 'config' / expected_file).read_text())[
        'controller_server']['ros__parameters']['FollowPath']
    runtime = summary['runtime_parameters']['controller_server']
    for key, value in expected.items():
        if trial['controller'] == 'dwb' and 'FollowPath.' + key not in runtime:
            continue  # V1 recorded DWB's undeclared stateful YAML key explicitly.
        assert runtime['FollowPath.' + key] == value, (path, key)
    assert trial['cleanup_returncode'] == 0
    rows = list(csv.DictReader((path / 'trajectory.csv').open()))
    stamps = [float(r['sim_time_s']) for r in rows if r.get('sim_time_s')]
    assert len(stamps) > 1 and all(b >= a for a, b in zip(stamps, stamps[1:]))
    source_counts = {k: sum(bool(r.get(k)) for r in rows)
                     for k in ('x', 'actual_vx', 'controller_vx', 'robot_cmd_vx')}
    prefix = f"{trial['controller']}_{trial['scene']}_{trial['repeat']}"
    log = suite / 'logs' / (prefix + '_launch.log')
    lines = log.read_text().splitlines()
    # Exclude launch and teardown errors from navigation control diagnostics.
    begin = next((i for i, line in enumerate(lines, 1)
                  if 'Received a goal, begin computing control effort' in line), 0)
    end = next((i for i, line in enumerate(lines, 1) if 'Goal succeeded' in line), len(lines))
    records = [r for r in debug_records(log) if begin <= r['line'] <= end]
    sequence = []
    for r in records:
        if not sequence or sequence[-1] != r['goal_phase']:
            sequence.append(r['goal_phase'])
    outside = [r for r in records if r['goal_phase'] == 'rotating'
               and r['true_goal_distance'] > r['terminal_radius']]
    zero_outside = [r for r in outside if r['best'] and r['best'][0]
                    and abs(r['best'][1]) < 1e-9 and abs(r['best'][2]) < 1e-9]
    releases = []
    for r in outside:
        after = next((n for n in records if n['line'] > r['line']
                      and n['goal_phase'] == 'normal' and n['best']
                      and n['best'][1] > 0), None)
        if after:
            releases.append(dict(outside=r, resumed=after))
    navigation_lines = lines[max(0, begin-1):end]
    result = dict(version=label, scene=trial['scene'], controller=trial['controller'],
                  trial=str(path.relative_to(BENCH)), status=summary['status'],
                  action_result=summary.get('action_result'), valid=summary['valid'],
                  start_valid=summary['start_valid'], success=metrics['success'],
                  completion_sim_s=metrics['timing']['first_planner_to_observed_stop_sim_s']
                  if metrics['success'] else None,
                  observed_sim_span_s=metrics['timing']['first_planner_to_observed_stop_sim_s'],
                  length_m=metrics['motion']['trajectory_length_m'],
                  lateral_rmse_m=metrics['motion']['lateral_rmse_m'],
                  lateral_max_m=metrics['motion']['lateral_max_m'],
                  actual_wz_std=metrics['velocity']['actual']['angular']['std'],
                  actual_wz_switches=metrics['velocity']['actual']['angular_direction_switches'],
                  final_position_m=metrics['final_error']['position_m'],
                  final_yaw_rad=metrics['final_error']['yaw_rad'],
                  recoveries=summary.get('max_feedback_recoveries'),
                  progress_failures=sum('Failed to make progress' in s for s in navigation_lines),
                  behavior_goals=metrics['navigation_system']['behavior_goals_executed'],
                  spin_goals=metrics['navigation_system']['spin_goals_executed'],
                  controller_phase_sequence=sequence, outside_rotating_records=len(outside),
                  zero_outside_rotating_records=len(zero_outside), releases=releases,
                  plan_messages=metrics['reference']['plan_messages'],
                  distinct_plans=metrics['reference']['distinct_plans'],
                  rate_warnings=sum('Control loop missed' in s for s in navigation_lines),
                  no_feasible_records=sum(r['valid'] == 0 for r in records),
                  samples=len(rows), source_counts=source_counts,
                  anomalies=summary.get('anomalies'),
                  fairness_signature=fairness_signature(summary),
                  summary_sha256=sha(path / 'summary.json'))
    if label == 'V1.1':
        assert all(source_counts.values()), (path, source_counts)
        (HERE / (suite.name + '_' + prefix + '_debug.json')).write_text(
            json.dumps(records, indent=2, allow_nan=False) + '\n')
    return result


rows = []
for suite in sorted((BENCH / 'results/v1').glob('v1_1_*')):
    manifest = json.loads((suite / 'manifest.json').read_text())
    for trial in manifest['trials']:
        if trial['state'] != 'pending':
            rows.append(row_for(suite, trial, 'V1.1'))
for name, label in [('static_v1_20261019', 'V1'),
                    ('tuning_r1_diagnostic_baseline_20261019', 'R1-original')]:
    suite = BENCH / 'results/v1' / name
    for trial in json.loads((suite / 'manifest.json').read_text())['trials']:
        rows.append(row_for(suite, trial, label))
groups = {}
for row in rows:
    key = '/'.join(row[k] for k in ('version', 'controller', 'scene'))
    groups.setdefault(key, []).append(row)
aggregates = {}
for key, group in groups.items():
    good = [r for r in group if r['success'] and r['valid']]
    aggregates[key] = dict(attempts=len(group), successes=len(good),
                           failures=dict(Counter(r['status'] for r in group if not r['success'])),
                           recoveries=[r['recoveries'] for r in group],
                           progress_failures=sum(r['progress_failures'] for r in group),
                           successful_metrics={k: stats([r[k] for r in good]) for k in (
                               'completion_sim_s', 'length_m', 'lateral_rmse_m', 'actual_wz_std',
                               'actual_wz_switches', 'final_position_m', 'final_yaw_rad')})
tests = []
for path in (HERE / 'test_results/my_dwa_controller').glob('*.xml'):
    root = ET.parse(path).getroot()
    suites = [root] if root.tag == 'testsuite' else list(root)
    tests.append(dict(file=path.name, tests=sum(int(s.get('tests', 0)) for s in suites),
                      failures=sum(int(s.get('failures', 0)) for s in suites),
                      errors=sum(int(s.get('errors', 0)) for s in suites)))
result = dict(trials=rows, groups=aggregates, tests=tests,
              new_real_trials=sum(r['version'] == 'V1.1' for r in rows))
(HERE / 'analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
fields = [k for k in rows[0] if not isinstance(rows[0][k], (dict, list))]
with (HERE / 'comparison.csv').open('w') as output:
    writer = csv.DictWriter(output, fieldnames=fields, extrasaction='ignore')
    writer.writeheader()
    writer.writerows(rows)
print(json.dumps({k: v for k, v in aggregates.items()
                  if k.startswith('V1.1') or k.endswith('S04')}, indent=2))
