#!/usr/bin/env python3
"""Thin restart-per-trial scheduler and offline aggregation for static benchmark V1."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path

import signal
import statistics as stats
import subprocess
import sys
import time

from metrics import calculate
from scenes import load_scene, resources


# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

HERE = Path(__file__).resolve().parent


def statistics(values):
    values = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    return {'n': len(values), 'mean': stats.mean(values) if values else None,
            'sample_sd': stats.stdev(values) if len(values) > 1 else None,
            'min': min(values) if values else None, 'max': max(values) if values else None}


def fairness_signature(summary):
    runtime = json.loads(json.dumps(summary.get('runtime_parameters', {})))
    controller = runtime.get('controller_server', {})
    constraints = {name: controller.get('FollowPath.' + name) for name in
                   ('max_vel_x', 'max_vel_theta', 'acc_lim_x', 'acc_lim_theta')}
    runtime['controller_server'] = {k: v for k, v in controller.items()
                                    if not k.startswith('FollowPath.')}
    files = {}
    for label in ('world', 'robot', 'map_image', 'map', 'bt'):
        files[label] = next((c['sha256'] for c in summary.get('config_files', [])
                             if Path(c['saved']).name.startswith(label + '_') and
                             (label != 'map' or Path(c['saved']).suffix == '.yaml')), None)
    common = {'runtime': runtime, 'constraints': constraints, 'files': files,
              'start': summary['arguments']['start'], 'goal': summary['arguments']['goal']}
    return hashlib.sha256(json.dumps(common, sort_keys=True).encode()).hexdigest()


def fairness_status(signatures, observed, expected):
    covered = len(expected) >= 2 and observed >= expected
    return {'signature_count': len(signatures), 'observed_controllers': sorted(observed),
            'expected_controllers': sorted(expected), 'both_controllers_observed': covered,
            'common_runtime_matches': len(signatures) == 1 and covered,
            'signatures': sorted(signatures)}


def audit_configurations():
    import yaml
    configurations, constraints, identifiers = [], [], {}
    for controller, filename in [('mydwa', 'nav2_params.yaml'), ('dwb', 'nav2_params_dwb.yaml')]:
        path = HERE.parent / 'config' / filename
        identifiers[controller] = hashlib.sha256(path.read_bytes()).hexdigest()
        configuration = yaml.safe_load(path.read_text())
        follow = configuration['controller_server']['ros__parameters'].pop('FollowPath')
        constraints.append({key: follow[key] for key in
                            ('max_vel_x', 'max_vel_theta', 'acc_lim_x', 'acc_lim_theta')})
        configurations.append(configuration)
    if configurations[0] != configurations[1] or constraints[0] != constraints[1]:
        raise RuntimeError('Common Nav2 YAML or velocity/acceleration constraints differ')
    return {'common_yaml_matches': True, 'constraints': constraints[0], 'yaml_sha256': identifiers}


def nested(data, name):
    for part in name.split('.'):
        if not isinstance(data, dict):
            return None
        data = data.get(part)
    return data


METRICS = ('timing.first_planner_to_observed_stop_sim_s', 'navigation_sim_s',
           'motion.trajectory_length_m', 'motion.lateral_rmse_m',
           'motion.lateral_max_m', 'velocity.actual.linear.std', 'velocity.actual.angular.std',
           'velocity.actual.linear.total_variation', 'velocity.actual.angular.total_variation',
           'velocity.actual.linear.mean_abs_acceleration',
           'velocity.actual.angular.mean_abs_acceleration',
           'velocity.actual.angular_direction_switches', 'final_error.position_m',
           'final_error.yaw_rad', 'navigation_system.behavior_goals_executed')


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def aggregate(directory, recompute=False):
    sources = {}
    source_directory = directory / 'analysis_sources'
    source_directory.mkdir(exist_ok=True)
    for name in ('metrics.py', 'run_suite.py'):
        source = Path(__file__).with_name(name)
        sha = hashlib.sha256(source.read_bytes()).hexdigest()
        saved = source_directory / (sha + '_' + name)
        if not saved.exists():
            saved.write_bytes(source.read_bytes())
        sources[name] = {'sha256': sha, 'saved': str(saved.relative_to(directory))}
    manifest = json.loads((directory / 'manifest.json').read_text())
    groups, fair = defaultdict(list), defaultdict(set)
    observed, expected = defaultdict(set), defaultdict(set)
    for trial in manifest['trials']:
        expected[trial['scene']].add(trial['controller'])
        path = directory / trial['directory']
        summary_file = path / 'summary.json'
        summary = json.loads(summary_file.read_text()) if summary_file.exists() else {}
        if summary.get('start_valid') and summary.get('runtime_parameters'):
            fair[trial['scene']].add(fairness_signature(summary))
            observed[trial['scene']].add(trial['controller'])
        metrics_file = path / 'metrics.json'
        if summary_file.exists() and (recompute or not metrics_file.exists()):
            try:
                write_json(metrics_file, calculate(path))
            except Exception as exc:
                trial['metric_error'] = repr(exc)
        metrics = json.loads(metrics_file.read_text()) if metrics_file.exists() else {}
        groups[(trial['controller'], trial['scene'])].append((trial, summary, metrics))
    output = {'suite': directory.name, 'comparison': 'end-to-end NavigateToPose',
              'fairness': {scene: fairness_status(fair[scene], observed[scene], expected[scene])
                           for scene in expected},
              'analysis_sources': sources,
              'groups': [], 'interpretation': 'Three repeats are descriptive, not evidence '
              'of a statistically significant controller advantage. Failed partial runs '
              'are included in all_observed; metric n shows missing values.'}
    report = ['# Static navigation benchmark V1', '', output['interpretation'], '',
              'Time = first planner goal native stamp to last observed /clock at result/stop '
              'detection, not exact NavigateToPose action duration. Source data before first '
              'planning is excluded independently per stream. Raw recording span and final '
              'Nav2 feedback duration remain in metrics.json. '
              'Failed partial runs are included.', '',
              '|Controller|Scene|Action success / scheduled|Invalid data|Sim s mean ± SD (n)|'
              'Length m mean ± SD|RMSE m mean ± SD|Final m mean ± SD|Failures|',
              '|---|---|---:|---:|---|---|---|---|---|']
    for (controller, scene), trials in sorted(groups.items()):
        reasons = Counter()
        for trial, summary, metrics in trials:
            if summary.get('action_result') != 'SUCCEEDED':
                reasons[summary.get('stop_reason') or summary.get('error') or
                        summary.get('action_result') or trial.get('state', 'missing')] += 1
        group = {'controller': controller, 'scene': scene, 'scheduled': len(trials),
                 'action_successes': sum(s.get('action_result') == 'SUCCEEDED'
                                         for _, s, _ in trials),
                 'valid_trials': sum(bool(s.get('valid')) for _, s, _ in trials),
                 'localization_warning_trials': sum(bool(m.get('localization', {}).get(
                     'warning_over_0_5m')) for _, _, m in trials),
                 'failures': dict(reasons),
                 'all_observed': {key: statistics([nested(m, key) for _, _, m in trials])
                                  for key in METRICS},
                 'successful_valid': {key: statistics([nested(m, key) for _, s, m in trials
                                                       if m.get('success')]) for key in METRICS}}
        output['groups'].append(group)

        def display(key):
            value = group['all_observed'][key]
            if value['mean'] is None:
                return 'missing (0)'
            sd = f'{value["sample_sd"]:.3f}' if value['sample_sd'] is not None else 'NA'
            return f'{value["mean"]:.3f} ± {sd} ({value["n"]})'

        report.append(f'|{controller}|{scene}|{group["action_successes"]}/{len(trials)}|'
                      f'{len(trials)-group["valid_trials"]}|'
                      f'{display("timing.first_planner_to_observed_stop_sim_s")}|'
                      f'{display("motion.trajectory_length_m")}|'
                      f'{display("motion.lateral_rmse_m")}|'
                      f'{display("final_error.position_m")}|{json.dumps(dict(reasons))}|')
    write_json(directory / 'aggregate.json', output)
    (directory / 'report.md').write_text('\n'.join(report) + '\n')
    return output


def stop_process(process, timeout=15.):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.)


def run_one(directory, trial, env):
    scene = load_scene(trial['scene'])
    path = directory / trial['directory']
    path.parent.mkdir(parents=True, exist_ok=True)
    logs = directory / 'logs'
    logs.mkdir(exist_ok=True)
    stem = '_'.join((trial['controller'], trial['scene'], str(trial['repeat'])))
    launch_command = ['bash', str(HERE / 'start_scene.sh'), trial['controller'], trial['scene']]
    command = [sys.executable, str(HERE / 'run_trial.py'), '--controller', trial['controller'],
               '--start', *map(str, scene['start']), '--goal', *map(str, scene['goal']),
               '--timeout', str(scene['timeout']), '--ready-timeout', '90',
               '--results-dir', str(path.parent), '--trial-id', path.name,
               '--record-navigation', '--world-file', str(resources(scene)[0]),
               '--startup-script', str(HERE / 'start_scene.sh')]
    trial.update(command=command, launch_command=launch_command, state='running')
    launch = runner = None
    with (logs / (stem + '_launch.log')).open('w') as launch_log, (
            logs / (stem + '_trial.log')).open('w') as trial_log:
        try:
            launch = subprocess.Popen(launch_command, env=env, stdout=launch_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            runner = subprocess.Popen(command, env=env, stdout=trial_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            limit = time.monotonic() + 90 + scene['timeout'] + 20
            while runner.poll() is None:
                if time.monotonic() > limit or launch.poll() is not None:
                    trial['supervisor_reason'] = ('overall_wall_timeout' if launch.poll() is None
                                                  else 'launch_exited')
                    stop_process(runner, 8.)
                    break
                time.sleep(.2)
            trial['runner_returncode'] = runner.returncode
        finally:
            if runner is not None:
                stop_process(runner, 8.)
            if launch is not None:
                stop_process(launch)
            cleanup = subprocess.run(
                [sys.executable, str(ARCHIVE_WS / 'scripts/_baseline_processes.py')],
                env=env, stdout=launch_log, stderr=subprocess.STDOUT, timeout=15)
            trial['cleanup_returncode'] = cleanup.returncode
    summary_file = path / 'summary.json'
    if not summary_file.exists():
        path.mkdir(exist_ok=True)
        (path / 'trajectory.csv').write_text('sim_time_s,wall_elapsed_s,x,y,yaw\n')
        failure = {'trial_id': path.name, 'status': 'launch_or_runner_failed',
                   'valid': False, 'error': trial.get('supervisor_reason'),
                   'arguments': {'start': scene['start'], 'goal': scene['goal']}}
        write_json(summary_file, failure)
    summary = json.loads(summary_file.read_text())
    for name in ('metrics.py', 'scenes.py', 'run_suite.py'):
        destination = path / 'configs' / (name + '.snapshot')
        destination.parent.mkdir(exist_ok=True)
        destination.write_bytes((HERE / name).read_bytes())
        summary.setdefault('config_files', []).append({
            'source': str(HERE / name), 'saved': str(destination.relative_to(path)),
            'sha256': hashlib.sha256(destination.read_bytes()).hexdigest()})
    write_json(summary_file, summary)
    write_json(path / 'metrics.json', calculate(path))
    trial['state'] = json.loads(summary_file.read_text())['status']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controllers', nargs='+', choices=['mydwa', 'dwb'],
                        default=['mydwa', 'dwb'])
    parser.add_argument('--scenes', nargs='+', choices=[f'S{i:02}' for i in range(1, 7)],
                        default=['S01', 'S02', 'S03', 'S04', 'S05', 'S06'])
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--run-id',
                        default=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    parser.add_argument('--domain', type=int, default=77)
    parser.add_argument('--aggregate', type=Path,
                        help='recompute metrics/aggregation without simulation')
    args = parser.parse_args()
    if args.aggregate:
        aggregate(args.aggregate.resolve(), recompute=True)
        return
    if args.repeats < 1 or not 0 <= args.domain <= 200:
        parser.error('repeats must be positive; domain must be in [0,200]')
    if Path(args.run_id).name != args.run_id or args.run_id in ('.', '..'):
        parser.error('run-id must be one directory name')
    if os.environ.get('ROS_DISTRO') != 'humble':
        parser.error('source ROS Humble and the workspace overlay before starting a suite')
    audit = audit_configurations()
    directory = HERE / 'results' / 'v1' / args.run_id
    directory.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{11345+args.domain}')
    manifest = {'created_utc': datetime.now(timezone.utc).isoformat(),
                'domain': args.domain, 'configuration_audit': audit,
                'gazebo_master_uri': env['GAZEBO_MASTER_URI'], 'trials': []}
    # Alternate controllers within each repetition; every attempt gets a fresh process tree.
    for scene in args.scenes:
        resources(load_scene(scene))
        for repeat in range(1, args.repeats + 1):
            for controller in (args.controllers if repeat % 2 else args.controllers[::-1]):
                trial = {'controller': controller, 'scene': scene, 'repeat': repeat,
                         'directory': f'{controller}/{scene}/trial_{repeat:03}',
                         'state': 'pending'}
                manifest['trials'].append(trial)
    write_json(directory / 'manifest.json', manifest)
    for index, trial in enumerate(manifest['trials'], 1):
        print(f'[{index}/{len(manifest["trials"])}] {trial["directory"]}', flush=True)
        try:
            run_one(directory, trial, env)
        except Exception as exc:
            trial.update(state='supervisor_error', error=repr(exc))
            path = directory / trial['directory']
            path.mkdir(parents=True, exist_ok=True)
            if not (path / 'summary.json').exists():
                scene = load_scene(trial['scene'])
                write_json(path / 'summary.json', {
                    'trial_id': path.name, 'status': 'supervisor_error', 'valid': False,
                    'error': repr(exc),
                    'arguments': {'start': scene['start'], 'goal': scene['goal']}})
            if not (path / 'trajectory.csv').exists():
                (path / 'trajectory.csv').write_text('sim_time_s,wall_elapsed_s,x,y,yaw\n')
        write_json(directory / 'manifest.json', manifest)
        aggregate(directory)
        print(' -> ' + trial['state'], flush=True)
    print(directory / 'report.md', flush=True)


if __name__ == '__main__':
    main()
