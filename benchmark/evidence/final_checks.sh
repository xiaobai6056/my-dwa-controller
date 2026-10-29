#!/usr/bin/env bash
set -eo pipefail

# Baseline helper scripts/install prefixes live outside this archive.
export MYDWA_WS=${MYDWA_WS:-"$HOME/ros2_ws"}
REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source "${MYDWA_WS}/install/setup.bash"
cd "${REPO_ROOT}"
python3 -m unittest discover -s benchmark -p test_run_trial.py -v
ament_flake8
ament_pep257 --exclude benchmark/results benchmark/evidence
python3 benchmark/evidence/inspect_results.py benchmark/results/mydwa_006 benchmark/results/dwb_001
sha256sum -c benchmark/evidence/unchanged.sha256
bash -n ${MYDWA_WS}/scripts/start_headless_baseline.sh
bash -n ${MYDWA_WS}/scripts/start_ui_baseline.sh
python3 - <<'PY'
import csv
import os
import json
from pathlib import Path
import xml.etree.ElementTree as ET

bench = Path('benchmark')
for name, reason in [('timeout_002', 'wall_timeout'), ('clock_stall_002', 'clock_stalled')]:
    summary = json.loads((bench / 'results' / name / 'summary.json').read_text())
    assert summary['status'] == 'timeout' and summary['stop_reason'] == reason
    assert summary['action_result'] == 'CANCELED' and summary['cancellation_confirmed']
    if reason == 'clock_stalled':
        assert not summary['valid'] and summary['final_pose'] is None
        rows = list(csv.DictReader((bench / 'results' / name / 'trajectory.csv').open()))
        assert any(not row['x'] and 'tf_unavailable' in row['flags'] for row in rows)
for name in ('reject_wrong_start', 'reject_wrong_controller'):
    summary = json.loads((bench / 'results' / name / 'summary.json').read_text())
    assert not summary['valid'] and not summary.get('action_accepted', False)
test_xml = max((Path(os.environ['MYDWA_WS']) / 'build/my_dwa_controller/Testing').glob('*/Test.xml'),
               key=lambda path: path.stat().st_mtime)
tests = ET.parse(test_xml).findall('.//Testing/Test')
states = {test.findtext('Name'): test.get('Status') for test in tests}
assert states['uncrustify'] == 'failed'
assert all(state == 'passed' for name, state in states.items() if name != 'uncrustify')
print('Latest full CTest status:', states)
print('Acceptance, clock-stall, refusal and unchanged-source checks passed.')
PY
