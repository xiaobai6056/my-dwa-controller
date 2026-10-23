#!/usr/bin/env python3
"""Archive the unchanged V1 configuration after checking actual loaded parameters."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import yaml

HERE = Path(__file__).resolve().parent
BENCH = HERE.parent
PACKAGE = BENCH.parent
sys.path.insert(0, str(BENCH))
from run_suite import fairness_signature  # noqa: E402


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def archive(source, target):
    content = source.read_bytes()
    if target.exists():
        if target.read_bytes() != content:
            raise RuntimeError('Refusing to overwrite a different frozen artifact: ' + str(target))
    else:
        target.write_bytes(content)
    return {'source': str(source), 'saved': str(target.relative_to(HERE)), 'sha256': sha(target)}


def main():
    output = HERE / 'configs'
    output.mkdir(exist_ok=True)
    configurations = {}
    for controller, name in [('mydwa', 'nav2_params.yaml'), ('dwb', 'nav2_params_dwb.yaml')]:
        path = PACKAGE / 'config' / name
        configurations[controller] = yaml.safe_load(path.read_text())
    unchanged = {}
    for line in (BENCH / 'evidence/unchanged.sha256').read_text().splitlines():
        expected, name = line.split(maxsplit=1)
        assert sha(Path(name)) == expected, name
        unchanged[name] = expected
    runs = ['static_v1_20261019', 'tuning_r1_diagnostic_baseline_20261019']
    observations, signatures, undeclared_dwb_keys = [], set(), set()
    checked = 0
    for run in runs:
        directory = BENCH / 'results/v1' / run
        manifest = json.loads((directory / 'manifest.json').read_text())
        for trial in manifest['trials']:
            path = directory / trial['directory']
            summary = json.loads((path / 'summary.json').read_text())
            assert trial['cleanup_returncode'] == 0 and summary['start_valid'] and summary['valid']
            controller = trial['controller']
            expected = configurations[controller]['controller_server'][
                'ros__parameters']['FollowPath']
            actual = summary['runtime_parameters']['controller_server']
            for name, value in expected.items():
                qualified = 'FollowPath.' + name
                if qualified not in actual and controller == 'dwb':
                    undeclared_dwb_keys.add(qualified)
                    continue
                assert actual[qualified] == value, (run, trial, name)
            param_record = next(c for c in summary['config_files']
                                if Path(c['saved']).name.startswith('params_'))
            filename = 'nav2_params.yaml' if controller == 'mydwa' else 'nav2_params_dwb.yaml'
            current = PACKAGE / 'config' / filename
            assert param_record['sha256'] == sha(current)
            for config in summary['config_files']:
                assert sha(path / config['saved']) == config['sha256']
            checked += 1
            if trial['scene'] == 'S04':
                signatures.add(fairness_signature(summary))
                observations.append({'suite': run, 'trial': trial['directory'],
                                     'action_result': summary['action_result'],
                                     'max_feedback_recoveries': summary['max_feedback_recoveries'],
                                     'summary_sha256': sha(path / 'summary.json')})
    assert checked == 39 and len(signatures) == 1
    frozen = {
        'original_mydwa': archive(
            PACKAGE / 'config/nav2_params.yaml', output / 'mydwa_static_v1_original.yaml'),
        'official_dwb_reference': archive(
            PACKAGE / 'config/nav2_params_dwb.yaml', output / 'dwb_v1_reference.yaml')}
    sources = list((PACKAGE / 'src').glob('*.cpp')) + list((PACKAGE / 'include').rglob('*.hpp'))
    sources += [PACKAGE / 'test/test_controller.cpp', BENCH / 'run_trial.py',
                BENCH / 'run_suite.py', BENCH / 'metrics.py', BENCH / 'start_scene.sh',
                BENCH / 'scenarios.yaml', Path(__file__), HERE / 'analyze.py']
    binary = Path('/home/user/ros2_ws/install/my_dwa_controller/lib/libmy_dwa_controller.so')
    versions = subprocess.check_output([
        'dpkg-query', '-W', '-f=${Package}\t${Version}\n', 'gazebo', 'libgazebo11',
        'ros-humble-nav2-bringup', 'ros-humble-nav2-controller', 'ros-humble-nav2-dwb-controller',
        'ros-humble-nav2-simple-commander', 'ros-humble-turtlebot3-gazebo'], text=True)
    previous_versions = (BENCH / 'evidence/v1_package_versions.txt').read_text()

    def version_map(text):
        return {name.split(':')[0]: version for name, version in
                (line.split('\t') for line in text.splitlines())}

    assert version_map(versions) == version_map(previous_versions), 'Package versions changed'
    (HERE / 'evidence/package_versions.txt').write_text(versions)
    result = {
        'date': '2026-10-19', 'decision': 'retain_original_parameters',
        'candidate_count': 0, 'accepted_tuned_configuration': False,
        'archive_freeze_complete': True, 'reliability_acceptance_passed': False,
        'ready_for_dynamic_obstacle_development': False,
        'reason': 'Terminal latch persists outside XY tolerance while Nav2 replans reset the '
        'GoalChecker; ordinary scoring weights cannot release the state. No justified '
        'parameter-only repair identified; this is not an impossibility proof for all parameters.',
        'new_real_gazebo_trials': 3, 'new_deterministic_plugin_probes': 1,
        'new_s03_s05_or_final_six_scene_trials': 0,
        'conditional_regression_skip_reason': 'No candidate improved S04; no tuned configuration '
        'selected. Do not label the original as having passed a new full regression.',
        'loaded_parameter_and_config_checks': checked,
        'undeclared_dwb_yaml_parameters': sorted(undeclared_dwb_keys),
        'parameter_check_scope': 'All explicit MyDWA YAML parameters verified against runtime; '
        'DWB explicitly lists unclaimed YAML keys instead of pretending they were effective.',
        's04_common_runtime_and_resource_signature': next(iter(signatures)),
        's04_observations': observations, 'frozen_configurations': frozen,
        'core_original_sha256': unchanged,
        'current_source_sha256': {str(p): sha(p) for p in sources},
        'current_plugin_binary': {'path': str(binary), 'sha256': sha(binary),
                                  'limitation': 'No V1 binary hash was captured; unchanged '
                                  'V1 source hashes and fresh baseline are the comparison basis.'}}
    (HERE / 'freeze_manifest.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: result[k] for k in
                      ('decision', 'candidate_count', 'new_real_gazebo_trials',
                       'loaded_parameter_and_config_checks', 'frozen_configurations')}, indent=2))


if __name__ == '__main__':
    main()
