"""Conservative post-trial gate for this finite regression, not a new runner."""
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
trial = root / 'mydwa/S04/trial_001'
summary = json.loads((trial / 'summary.json').read_text())
log = (root / 'logs/mydwa_S04_1_launch.log').read_text()
progress_failures = log.count('Failed to make progress')
passed = (summary['status'] == 'succeeded' and summary.get('valid') is True
          and summary.get('max_feedback_recoveries') == 0 and progress_failures == 0)
result = dict(passed=passed, status=summary['status'], valid=summary.get('valid'),
              recoveries=summary.get('max_feedback_recoveries'),
              progress_failures=progress_failures)
(root / 's04_gate.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result), flush=True)
sys.exit(0 if passed else 1)
