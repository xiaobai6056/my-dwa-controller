"""Verify and snapshot the tested finite V1.1 baseline, preserving V1/R1 records."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
RELEASE = HERE.parent
BENCH = RELEASE.parent
PACKAGE = BENCH.parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def archive(source, target):
    content = source.read_bytes()
    if target.exists():
        assert target.read_bytes() == content, target
    else:
        target.parent.mkdir(exist_ok=True, parents=True)
        target.write_bytes(content)
    return dict(source=str(source), saved=str(target.relative_to(RELEASE)), sha256=sha(target))


analysis = json.loads((HERE / 'analysis.json').read_text())
trials = [r for r in analysis['trials'] if r['version'] == 'V1.1']
assert len(trials) == 14
assert {s: sum(r['scene'] == s for r in trials) for s in ('S01', 'S03', 'S04', 'S05', 'S06')} == {
    'S01': 2, 'S03': 2, 'S04': 5, 'S05': 2, 'S06': 3}
previous = json.loads((BENCH / 'tuning_r1/freeze_manifest.json').read_text())
assert {r['fairness_signature'] for r in trials if r['scene'] == 'S04'} == {
    previous['s04_common_runtime_and_resource_signature']}
cpp = [t for t in analysis['tests'] if t['file'].endswith('.gtest.xml')]
assert sum(t['tests'] for t in cpp) == 206
assert all(t['failures'] == 0 and t['errors'] == 0 for t in cpp)
assert 'Ran 34 tests' in (HERE / 'tests.log').read_text()
unchanged = {}
for name, expected in previous['current_source_sha256'].items():
    if name in (str(PACKAGE / 'src/controller.cpp'), str(PACKAGE / 'test/test_controller.cpp')):
        continue
    assert sha(Path(name)) == expected, name
    unchanged[name] = expected
for line in (BENCH / 'evidence/unchanged.sha256').read_text().splitlines():
    if not line.strip():
        continue
    expected, name = line.split(maxsplit=1)
    if name == str(PACKAGE / 'src/controller.cpp'):
        continue
    assert sha(Path(name)) == expected, name
    unchanged[name] = expected
for filename in ('v1', 'diagnostic'):
    history = json.loads((BENCH / 'tuning_r1/evidence' / filename / 'analysis.json').read_text())
    for name, expected in history['summary_input_sha256'].items():
        assert sha(BENCH / name) == expected, name
for line in (HERE / 'tested_artifacts.sha256').read_text().splitlines():
    expected, name = line.split(maxsplit=1)
    assert sha(BENCH / name) == expected, name
runtime = json.loads((HERE / 'runtime_processes.json').read_text())
binary = Path('/home/user/ros2_ws/install/my_dwa_controller/lib/libmy_dwa_controller.so')
assert any(lib['sha256'] == sha(binary) for p in runtime for lib in p['libraries'])
assert any(Path(p['command'].split()[0]).name == 'gzserver' for p in runtime)
assert not any(Path(p['command'].split()[0]).name in ('gzclient', 'rviz2') for p in runtime)
versions = subprocess.check_output([
    'dpkg-query', '-W', '-f=${Package}\t${Version}\n', 'gazebo', 'libgazebo11',
    'ros-humble-nav2-bringup', 'ros-humble-nav2-controller', 'ros-humble-nav2-dwb-controller',
    'ros-humble-nav2-simple-commander', 'ros-humble-turtlebot3-gazebo'], text=True)
assert versions == (BENCH / 'tuning_r1/evidence/package_versions.txt').read_text()
(HERE / 'package_versions.txt').write_text(versions)
snapshots = [archive(PACKAGE / name, RELEASE / 'configs' / saved) for name, saved in (
    ('config/nav2_params.yaml', 'mydwa_static_v1_1.yaml'),
    ('config/nav2_params_dwb.yaml', 'dwb_v1_reference.yaml'))]
snapshots += [archive(PACKAGE / name, HERE / (Path(name).name + '.tested.snapshot'))
              for name in ('src/controller.cpp', 'test/test_controller.cpp')]
raw = {}
for row in trials:
    path = BENCH / row['trial']
    for p in path.iterdir():
        if p.is_file():
            raw[str(p.relative_to(BENCH))] = sha(p)
    run = path.parents[2]
    raw[str((run / 'manifest.json').relative_to(BENCH))] = sha(run / 'manifest.json')
    for p in (run / 'logs').glob('*'):
        if p.is_file():
            raw[str(p.relative_to(BENCH))] = sha(p)
accepted = all(r['success'] and r['valid'] and r['start_valid'] and r['recoveries'] == 0
               and r['progress_failures'] == 0 for r in trials)
accepted = accepted and all(r['spin_goals'] == 0 and 'rotating' in r['controller_phase_sequence']
                            for r in trials if r['scene'] == 'S06')
result = dict(created_utc=datetime.now(timezone.utc).isoformat(),
              decision='accept_v1_1_static_baseline' if accepted else 'do_not_freeze',
              accepted=accepted, parameters_changed=False, algorithm_scope='setPlan lifecycle only',
              new_real_trials=len(trials), successes=sum(r['success'] for r in trials),
              recoveries=sum(r['recoveries'] for r in trials),
              regression_scenes=['S01', 'S03', 'S04', 'S05', 'S06'],
              untested_scene_this_round='S02 (V1 historical 3/3; no V1.1 result claimed)',
              readiness='Controlled dynamic-obstacle development may begin; this is not a safety '
              'certification or a claim of statistically proven elimination of rare failures.',
              configuration_and_source_snapshots=snapshots,
              tested_plugin_sha256=sha(binary),
              pre_fix_plugin_sha256=sha(HERE / 'original/libmy_dwa_controller.so.snapshot'),
              unchanged_source_and_config_sha256=unchanged,
              historical_summary_integrity_checks=39,
              cpp_tests_passed=206, benchmark_python_tests_passed=34,
              s04_common_runtime_and_resource_signature=trials[0]['fairness_signature'],
              trial_directories=[r['trial'] for r in trials],
              raw_artifact_sha256=raw,
              evidence_sha256={str(p.relative_to(RELEASE)): sha(p) for p in HERE.rglob('*')
                               if p.is_file() and '__pycache__' not in p.parts},
              report_sha256={p.name: sha(p) for p in RELEASE.glob('*.md')},
              limitations=['No reliable collision/contact or obstacle-clearance measurement',
                           'Debug phases sampled at roughly 1 Hz; brief transitions may be missed',
                           'Known pre-existing obstacle_evaluator.cpp uncrustify failure',
                           'No allocation-failure injection or concurrent stress test'])
(RELEASE / 'freeze_manifest.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
print(json.dumps({k: result[k] for k in ('decision', 'new_real_trials', 'successes', 'recoveries')},
                 indent=2))
