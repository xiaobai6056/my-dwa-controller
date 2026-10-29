"""Verify the frozen report, evidence, raw trials and current tested plugin."""
import hashlib
import json
from pathlib import Path
import os


# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

RELEASE = Path(__file__).resolve().parent.parent
BENCH = RELEASE.parent
manifest = json.loads((RELEASE / 'freeze_manifest.json').read_text())


def check(path, expected):
    assert hashlib.sha256(path.read_bytes()).hexdigest() == expected, path


for name, expected in manifest['raw_artifact_sha256'].items():
    check(BENCH / name, expected)
for name, expected in manifest['evidence_sha256'].items():
    check(RELEASE / name, expected)
for name, expected in manifest['report_sha256'].items():
    check(RELEASE / name, expected)
for name, expected in manifest['unchanged_source_and_config_sha256'].items():
    check(Path(name), expected)
for artifact in manifest['configuration_and_source_snapshots']:
    check(RELEASE / artifact['saved'], artifact['sha256'])
    check(Path(artifact['source']), artifact['sha256'])
check((ARCHIVE_WS / 'install/my_dwa_controller/lib/libmy_dwa_controller.so'),
      manifest['tested_plugin_sha256'])
assert manifest['accepted'] and manifest['new_real_trials'] == manifest['successes'] == 14
assert manifest['recoveries'] == 0
assert json.loads((RELEASE / 'evidence/final_process_audit.json').read_text()) == []
print(json.dumps(dict(verified=True, real_trials=14, successes=14, recoveries=0,
                      cpp_tests=manifest['cpp_tests_passed'],
                      python_tests=manifest['benchmark_python_tests_passed'],
                      raw_artifacts=len(manifest['raw_artifact_sha256']),
                      evidence_files=len(manifest['evidence_sha256']),
                      decision=manifest['decision']), indent=2))
