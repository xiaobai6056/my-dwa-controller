"""Read-only final checks of all scheduled trials, including unsuccessful ones."""
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

directory = Path(sys.argv[1]).resolve()
manifest = json.loads((directory / 'manifest.json').read_text())
aggregate = json.loads((directory / 'aggregate.json').read_text())
counts = Counter((t['controller'], t['scene']) for t in manifest['trials'])
assert len(counts) == 12 and set(counts.values()) == {3}, counts
assert all(t['state'] not in ('pending', 'running') for t in manifest['trials'])
assert all(f['common_runtime_matches'] for f in aggregate['fairness'].values())
for source in aggregate['analysis_sources'].values():
    assert hashlib.sha256((directory / source['saved']).read_bytes()).hexdigest() == source['sha256']
outcomes, warnings, terminals = Counter(), [], []
row_count, disagreements, coverages, plan_lengths = 0, [], [], []
for trial in manifest['trials']:
    path = directory / trial['directory']
    summary = json.loads((path / 'summary.json').read_text())
    metrics = json.loads((path / 'metrics.json').read_text())
    assert metrics['metrics_source_sha256'] == aggregate['analysis_sources']['metrics.py']['sha256']
    outcomes[(trial['controller'], trial['scene'], summary['status'])] += 1
    assert trial.get('cleanup_returncode') == 0, trial
    for config in summary.get('config_files', []):
        assert hashlib.sha256((path / config['saved']).read_bytes()).hexdigest() == config['sha256']
    for name, expected in metrics['input_sha256'].items():
        assert hashlib.sha256((path / name).read_bytes()).hexdigest() == expected
    rows = list(csv.DictReader((path / 'trajectory.csv').open()))
    row_count += len(rows)
    if summary.get('action_accepted'):
        assert rows and summary.get('action_result') is not None
        simulated = [float(r['sim_time_s']) for r in rows]
        walls = [float(r['wall_elapsed_s']) for r in rows]
        assert all(b >= a for a, b in zip(simulated, simulated[1:]))
        assert all(b > a for a, b in zip(walls, walls[1:]))
        assert summary['event_monitor_ready']
        assert summary['actual_controller_plugin'] == {
            'mydwa': 'my_dwa_controller::MyDWAController', 'dwb': 'dwb_core::DWBLocalPlanner'
        }[trial['controller']]
        assert metrics['timing']['first_planner_to_observed_stop_sim_s'] > 0.
        plan = json.loads((path / 'plans.jsonl').read_text().splitlines()[0])
        points = plan['points']
        length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
        plan_lengths.append({'trial': trial['directory'], 'length_m': length})
        scene = trial['scene']
        if scene in ('S01', 'S06'):
            assert length > 3.5 and max(p[1] for p in points)-min(p[1] for p in points) < .3
        elif scene == 'S02':
            assert length > 7 and any(y > 2 and x > 4.2 for x, y, _ in points)
        elif scene == 'S03':
            assert length > 10 and any(4 < x < 5 and y > 2.2 for x, y, _ in points)
        elif scene == 'S04':
            assert length > 4.1 and max(abs(p[1]) for p in points) > .3
        elif scene == 'S05':
            assert length > 7 and all(abs(y) < .4 for x, y, _ in points if 0 < x < 6)
        disagreements.append(summary['end_map_world_disagreement_m'])
        coverages.append({'trial': trial['directory'],
                          'pose_s': metrics['motion']['integrated_time_s'],
                          'odom_s': metrics['velocity']['actual']['linear']['integrated_time_s'],
                          'controller_cmd_s': metrics['velocity']['controller_command'][
                              'linear']['integrated_time_s']})
    if summary.get('action_result') == 'SUCCEEDED':
        assert metrics['reference']['point_count'] > 1
        assert metrics['motion']['lateral_rmse_m'] is not None
        assert metrics['motion']['integrated_time_s'] > 1.
        assert any(abs(float(row.get('actual_vx') or 0)) > .01 for row in rows)
    assert metrics['safety']['minimum_clearance_m'] is None
    assert metrics['safety']['collision_count'] is None and metrics['safety']['missing_reason']
    if metrics['localization']['warning_over_0_5m']:
        warnings.append(trial['directory'])
    if trial['scene'] == 'S06':
        terminals.append({'trial': trial['directory'], 'action_result': summary.get('action_result'),
                          **(metrics.get('terminal_rotation') or {})})
result = {'scheduled_trials': len(manifest['trials']), 'outcomes': [
    {'controller': c, 'scene': s, 'status': state, 'count': count}
    for (c, s, state), count in sorted(outcomes.items())],
    'fairness': aggregate['fairness'], 'localization_warnings': warnings,
    'terminal_rotation_trials': terminals, 'total_csv_rows': row_count,
    'end_map_world_disagreement_max_m': max(disagreements),
    'source_coverage': coverages, 'first_plan_lengths': plan_lengths,
    'artifact_validation': 'passed'}
print(json.dumps(result, indent=2))
