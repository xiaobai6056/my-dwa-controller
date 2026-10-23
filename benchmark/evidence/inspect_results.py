"""Read saved acceptance artifacts with stdlib; no ROS or metric framework."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

for argument in sys.argv[1:]:
    directory = Path(argument)
    summary = json.loads((directory / 'summary.json').read_text())
    rows = list(csv.DictReader((directory / 'trajectory.csv').open()))
    times = [float(row['sim_time_s']) for row in rows]
    walls = [float(row['wall_elapsed_s']) for row in rows]
    assert len(rows) == summary['samples'] and len(rows) > 2
    assert all(b >= a for a, b in zip(times, times[1:]))
    assert all(b > a for a, b in zip(walls, walls[1:]))
    assert len(set(times)) > 2
    for column in ('x', 'y', 'yaw', 'actual_vx', 'actual_wz', 'controller_vx', 'robot_cmd_vx'):
        values = [float(row[column]) for row in rows if row[column]]
        assert values and all(math.isfinite(value) for value in values), column
    assert any(abs(float(row['actual_vx'] or 0)) > .01 for row in rows)
    assert summary['action_result'] == 'SUCCEEDED' and summary['action_status_code'] == 4
    assert summary['valid'] and summary['final_error'] is not None
    for config in summary['config_files']:
        content = (directory / config['saved']).read_bytes()
        assert hashlib.sha256(content).hexdigest() == config['sha256']
    print(json.dumps({key: summary[key] for key in (
        'trial_id', 'actual_controller_plugin', 'action_result', 'samples', 'valid_samples',
        'navigation_wall_s', 'navigation_sim_s', 'final_error', 'anomalies')}, indent=2))
