"""Read-only integrity and consistency audit, repeatable without simulation."""
import hashlib
import json
from pathlib import Path

HERE=Path(__file__).resolve().parents[1]
ROOT=HERE.parents[2]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
initial=json.loads((HERE/'evidence/initial_files.json').read_text())
# This new test was created before the initial freeze, then moved into diagnostics.
moved=str(ROOT/'benchmark/test_dynamic_diagnostics.py')
changed=[name for name,digest in initial.items() if name!=moved and (not Path(name).is_file() or sha(Path(name))!=digest)]
assert not changed, changed
rows=json.loads((HERE/'outputs/v0_1_analysis/all_events.json').read_text())
for row in rows:
    source=Path(row['source'])
    for name,digest in row['source_sha256'].items():assert sha(source/name)==digest
    contacts=[json.loads(l) for l in (source/'contacts.jsonl').read_text().splitlines()]
    assert row['event_native_or_receipt_absolute']['T7']==min(x['stamp'] for x in contacts)
    assert row['first_scan']['valid'] and row['first_scan']['hit_rays']>=3
    assert row['first_costmap']['valid'] and row['first_costmap']['lethal_cells']>=3
    assert row['events']['T7']>row['events']['T4']
checks=[]
for run in sorted((HERE/'runs').iterdir()):
    if not (run/'manifest.json').exists():continue
    manifest=json.loads((run/'manifest.json').read_text())
    summary=json.loads((run/'trial/summary.json').read_text())
    metrics=json.loads((run/'metrics.json').read_text())
    baseline=json.loads((ROOT/f'benchmark/results/dynamic/formal_v0_1_20261020/D01/{metrics["controller"]}/trial_001/summary.json').read_text())
    # Original parameter files and all captured runtime parameters must match.
    assert summary['runtime_parameters']==baseline['runtime_parameters'],run
    assert summary['expected_controller_plugin'] in (run/'launch.log').read_text(),run
    assert summary['valid'] and summary['action_accepted'],run
    checks.append(dict(run=run.name,plugin=summary['expected_controller_plugin'],
        action=summary['action_result'],termination=metrics['termination'],
        cleanup_returncode=manifest['cleanup_returncode'],runtime_parameters_equal_v01=True))
result=dict(initial_files=len(initial),unchanged_existing_files=len(initial)-1,
    excluded_new_test_move=moved,changed_existing_files=changed,
    offline_source_hashes_and_earliest_contact_checks=6,runs=checks,
    plugin_sha256=sha(Path('/home/user/ros2_ws/install/my_dwa_controller/lib/libmy_dwa_controller.so')))
(HERE/'evidence/integrity_audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))