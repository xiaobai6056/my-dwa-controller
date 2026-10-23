"""Read-only acceptance of the diagnostic experiment, archive and existing regression."""
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

here = Path(__file__).resolve().parents[1]
bench = here.parent
freeze = json.loads((here / 'freeze_manifest.json').read_text())
for name, expected in freeze['current_source_sha256'].items():
    assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == expected, name
for record in freeze['frozen_configurations'].values():
    assert hashlib.sha256((here / record['saved']).read_bytes()).hexdigest() == record['sha256']
assert not freeze['accepted_tuned_configuration'] and not freeze['reliability_acceptance_passed']
assert freeze['new_real_gazebo_trials'] == 3 and freeze['candidate_count'] == 0
suite = bench / 'results/v1/tuning_r1_diagnostic_baseline_20261019'
manifest = json.loads((suite / 'manifest.json').read_text())
assert len(manifest['trials']) == 3
observed = []
for trial in manifest['trials']:
    path = suite / trial['directory']
    summary = json.loads((path / 'summary.json').read_text())
    metrics = json.loads((path / 'metrics.json').read_text())
    assert trial['state'] == 'succeeded' and trial['cleanup_returncode'] == 0
    assert summary['action_accepted'] and summary['action_result'] == 'SUCCEEDED'
    assert summary['valid'] and summary['start_valid'] and summary['event_monitor_ready']
    assert summary['actual_controller_plugin'] == 'my_dwa_controller::MyDWAController'
    for name, expected in metrics['input_sha256'].items():
        assert hashlib.sha256((path / name).read_bytes()).hexdigest() == expected
    rows = list(csv.DictReader((path / 'trajectory.csv').open()))
    sim = [float(r['sim_time_s']) for r in rows]
    wall = [float(r['wall_elapsed_s']) for r in rows]
    assert all(b >= a for a, b in zip(sim, sim[1:]))
    assert all(b > a for a, b in zip(wall, wall[1:]))
    assert len(rows) > 100 and any(float(r.get('actual_vx') or 0) > .01 for r in rows)
    for name, limit in [('controller_vx', .26), ('controller_wz', 1.),
                        ('robot_cmd_vx', .26), ('robot_cmd_wz', 1.)]:
        assert all(abs(float(r[name])) <= limit + 1e-12 for r in rows if r[name])
    observed.append({'trial': trial['directory'], 'rows': len(rows),
                     'completion_sim_s': metrics['timing']['first_planner_to_observed_stop_sim_s'],
                     'max_feedback_recoveries': summary['max_feedback_recoveries']})
probe = ET.parse(here / 'evidence/terminal_latch_probe.xml').getroot()
assert probe.attrib['tests'] == '1' and probe.attrib['failures'] == '0'
test_xml = max(Path('/home/user/ros2_ws/build/my_dwa_controller/Testing').glob('*/Test.xml'),
               key=lambda path: path.stat().st_mtime)
tests = ET.parse(test_xml).findall('.//Testing/Test')
states = {t.findtext('Name'): t.get('Status') for t in tests}
assert len(states) == 15 and states['uncrustify'] == 'failed'
assert all(v == 'passed' for k, v in states.items() if k != 'uncrustify')
helper = Path('/home/user/ros2_ws/scripts/_baseline_processes.py')
spec = importlib.util.spec_from_file_location('baseline_processes', helper)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
active = {pid: p['args'] for pid, p in module.snapshot().items() if p['domain'] == '77' and (
    module.is_baseline_launch(p['args']) or (p['args'] and Path(p['args'][0]).name in
    ('gzserver', 'gzclient', 'rviz2', 'component_container_isolated')))}
assert not active, active
result = {'validation': 'passed', 'new_real_trials': observed, 'candidate_count': 0,
          'data_validity_is_not_reliability_acceptance': True, 'regression': states,
          'active_simulation_processes_in_domain_77': active}
print(json.dumps(result, indent=2))
