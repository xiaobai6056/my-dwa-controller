"""Verify observed first plans exercise the approved scene geometry, not just endpoints."""
import json
import math
from pathlib import Path

bench = Path(__file__).resolve().parents[1]
paths = {'S01': 'results/v1_pilots/S01_mydwa',
         'S02': 'results/v1_pilots/S02_surface_mydwa',
         'S03': 'results/v1/stage_a_remaining/mydwa/S03/trial_001',
         'S04': 'results/v1/stage_a_remaining/mydwa/S04/trial_001',
         'S05': 'results/v1/stage_a_remaining/mydwa/S05/trial_001',
         'S06': 'results/v1_pilots/S06_mydwa'}
report = []
for name, relative in paths.items():
    directory = bench / relative
    summary = json.loads((directory / 'summary.json').read_text())
    assert summary['action_result'] == 'SUCCEEDED' and summary['valid']
    plan = json.loads((directory / 'plans.jsonl').read_text().splitlines()[0])
    points = plan['points']
    assert plan['frame_id'] == 'map' and len(points) > 2
    length = sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(points, points[1:]))
    if name == 'S01':
        assert length > 3.5 and max(p[1] for p in points)-min(p[1] for p in points) < .3
    if name == 'S02':
        assert any(2 < x < 3 and abs(y) < .8 for x, y, _ in points)
        assert any(y > 2 and x > 4.2 for x, y, _ in points) and length > 7
    if name == 'S03':
        assert any(4 < x < 5 and y > 2.2 for x, y, _ in points)
        assert points[0][1] < .5 and points[-1][1] < .5 and length > 10
    if name == 'S04':
        assert max(abs(p[1]) for p in points) > .3 and length > 4.1
    if name == 'S05':
        assert all(abs(y) < .4 for x, y, _ in points if 0 < x < 6) and length > 7
    if name == 'S06':
        metrics = json.loads((directory / 'metrics.json').read_text())
        assert metrics['terminal_rotation']['attribution'] == 'controller'
        assert metrics['navigation_system']['spin_goals_executed'] == 0 and length > 3.5
        assert 'goal_phase=rotating' in (bench / 'evidence/pilot_S06_launch.log').read_text()
    report.append({'scene': name, 'trial': relative, 'planned_length_m': length,
                   'navigation_sim_s': summary['navigation_sim_s'],
                   'map_world_end_disagreement_m': summary.get('end_map_world_disagreement_m'),
                   'geometry_purpose_check': 'passed'})
print(json.dumps(report, indent=2))
