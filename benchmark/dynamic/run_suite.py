#!/usr/bin/env python3
"""Restart-isolated dynamic suite; append-only attempts and offline JSON/CSV report."""
import argparse
from datetime import datetime, timezone
import csv
import hashlib
import json
import os
from pathlib import Path

import statistics
import subprocess
import sys
import time


# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from run_suite import (audit_configurations, stop_process, write_json,  # noqa: E402
                       fairness_signature)
sys.path.insert(0, str(HERE))
from scenario import load  # noqa: E402
from evaluation import calculate  # noqa: E402


def protected_files():
    root = HERE.parent.parent
    files = subprocess.check_output(
        ['git', '-C', str(root), 'ls-files', '-z']).decode().split('\0')
    return {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in files if name
            and not name.startswith('benchmark/dynamic/') and name != 'benchmark/test_dynamic.py'}


def aggregate(directory):
    manifest = json.loads((directory / 'manifest.json').read_text())
    rows = []
    signatures = {}
    for trial in manifest['trials']:
        path = directory / trial['directory']
        if not (path / 'summary.json').exists():
            continue
        result = calculate(path)
        write_json(path / 'metrics.json', result)
        row = dict(run=trial['directory'], repeat=trial['repeat'], **result)
        rows.append(row)
        summary = json.loads((path / 'summary.json').read_text())
        if summary.get('runtime_parameters') and summary.get('start_valid'):
            signatures.setdefault(
                trial['scene'],
                {}).setdefault(
                trial['controller'],
                []).append(
                fairness_signature(summary))
    write_json(directory / 'results.json', rows)
    with (directory / 'results.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, list(rows[0]) if rows else ['run'])
        writer.writeheader()
        writer.writerows(rows)
    groups = []
    for scene in manifest['scenes']:
        for controller in manifest['controllers']:
            group = [r for r in rows if r['scene'] ==
                     scene and r['controller'] == controller]
            success = [r for r in group if r['success']]
            times = [r['navigation_sim_s'] for r in success]
            gaps = [r['min_gap_approx_m']
                    for r in group if r['min_gap_approx_m'] is not None]
            groups.append(
                dict(
                    scene=scene, controller=controller,
                    recorded=len(group),
                    scheduled=manifest['repeats'],
                    accepted=sum(
                        r['action_accepted'] for r in group),
                    successes=len(success),
                    success_mean_sim_s=statistics.mean(times)
                    if times else None,
                    collisions=sum(r['collision'] for r in group),
                    min_gap_approx_m=min(gaps) if gaps else None,
                    failures=[
                        dict(
                            run=r['run'],
                            termination=r['termination'],
                            error=r['error'],
                            stop_reason=r['stop_reason'])
                        for r in group if not r['success']]))
    fair = {scene: dict(signatures=values,
                        common_runtime_matches=len(
                            {v for vals in values.values() for v in vals}) == 1,
                        both_controllers_observed=set(values) == {'mydwa', 'dwb'})
            for scene, values in signatures.items()}
    write_json(
        directory /
        'aggregate.json',
        dict(
            groups=groups,
            fairness=fair))
    report = [
        '# Dynamic Benchmark V0.1', '',
        'Three repeats are descriptive only. Times are simulation seconds '
        'from observed task dispatch '
        'to terminal result; only successful runs enter success-time means. '
        'Ground truth gap is an '
        'approximate conservative rectangle enclosure; collision is physical Gazebo contact. '
        'All attempts remain recorded. See README.md for definitions and limitations.',
        '',
        '|Scene|Controller|Action accepted|Success / scheduled|Success mean s|'
        'Contacts runs|Min approximate gap m|',
        '|---|---|---:|---:|---:|---:|---:|']

    def show(value):
        return 'NA' if value is None else f'{value:.3f}'
    for g in groups:
        report.append(
            f'|{g["scene"]}|{g["controller"]}|{g["accepted"]}|'
            f'{g["successes"]}/{g["scheduled"]}|{show(g["success_mean_sim_s"])}|'
            f'{g["collisions"]}|{show(g["min_gap_approx_m"])}|')
    report += ['',
               '|Run|Action|Termination|Sim s|Physical contact|Approx gap m|Angular TV|'
               'Switches|Stopped s|Resume s|Recoveries|Scan hits|Costmap hits|',
               '|---|---|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in rows:
        report.append(
            f'|{r["run"]}|{r["action_result"]}|{r["termination"]}|'
            f'{show(r["navigation_sim_s"])}|{r["collision"]}|{show(r["min_gap_approx_m"])}|'
            f'{show(r["angular_total_variation_rad_s"])}|{r["angular_direction_switches"]}|'
            f'{show(r["stopped_sim_s"])}|{show(r["resume_after_release_s"])}|{r["recoveries"]}|'
            f'{r["scan_hit_samples"]}/{r["scan_samples"]}|'
            f'{r["costmap_hit_samples"]}/{r["costmap_samples"]}|')
    report += ['', 'Common runtime fairness: `' +
               json.dumps(fair) + '`', '', 'Failure details:', '']
    report += ['- ' + json.dumps(r, ensure_ascii=False)
               for g in groups for r in g['failures']]
    (directory / 'report.md').write_text('\n'.join(report) + '\n')
    return rows


def cleanup_processes(runner, launch, env, launch_log, trial):
    """Confirm cleanup or explicitly prevent another trial after any exception."""
    trial['cleanup_returncode'] = 1
    errors = []
    for process, grace in ((runner, 8.), (launch, 15.)):
        if process is not None:
            try:
                stop_process(process, grace)
            except Exception as exc:
                errors.append(repr(exc))
    try:
        cleanup = subprocess.run(
            [sys.executable, str(ARCHIVE_WS / 'scripts/_baseline_processes.py')],
            env=env, stdout=launch_log, stderr=subprocess.STDOUT, timeout=15)
        if cleanup.returncode:
            errors.append('baseline cleanup returned ' +
                          str(cleanup.returncode))
    except Exception as exc:
        errors.append(repr(exc))
    if errors:
        trial['cleanup_error'] = '; '.join(errors)
    else:
        trial['cleanup_returncode'] = 0


def run_one(directory, trial, env):
    scene = load(trial['scene'])
    output = directory / trial['directory']
    output.parent.mkdir(parents=True, exist_ok=True)
    logs = directory / 'logs'
    logs.mkdir(exist_ok=True)
    stem = trial['directory'].replace('/', '_')
    launch = runner = None
    launch_command = [
        'bash', str(HERE / 'start_scene.sh'),
        trial['controller'],
        trial['scene']]
    command = [
        sys.executable, str(HERE / 'run_trial.py'),
        '--scene', trial['scene'],
        '--controller', trial['controller'],
        '--output', str(output)]
    trial.update(
        command=command,
        launch_command=launch_command,
        state='running')
    with (logs / (stem + '_launch.log')).open('w') as launch_log, (
            logs / (stem + '_trial.log')).open('w') as trial_log:
        try:
            launch = subprocess.Popen(
                launch_command,
                env=env,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True)
            runner = subprocess.Popen(
                command,
                env=env,
                stdout=trial_log,
                stderr=subprocess.STDOUT,
                start_new_session=True)
            deadline = time.monotonic() + 150 + scene['timeout'] * 3
            while runner.poll() is None:
                if time.monotonic() > deadline or launch.poll() is not None:
                    trial['supervisor_reason'] = 'overall_timeout' if launch.poll(
                    ) is None else 'launch_exited'
                    stop_process(runner, 8.)
                    break
                time.sleep(.2)
            trial['runner_returncode'] = runner.returncode
        finally:
            cleanup_processes(runner, launch, env, launch_log, trial)
    output.mkdir(exist_ok=True)
    if not (output / 'summary.json').exists():
        write_json(
            output / 'summary.json',
            dict(
                scene=trial['scene'],
                controller=trial['controller'],
                valid=False,
                error=trial.get(
                    'supervisor_reason',
                    'runner_exited_without_summary')))
    summary = json.loads((output / 'summary.json').read_text())
    if trial['cleanup_returncode']:
        summary.update(valid=False, cleanup_error='process cleanup failed')
        write_json(output / 'summary.json', summary)
    trial['state'] = calculate(output)['termination']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--scenes',
        nargs='+',
        choices=[
            'D01',
            'D02'],
        default=[
            'D01',
            'D02'])
    parser.add_argument(
        '--controllers',
        nargs='+',
        choices=[
            'mydwa',
            'dwb'],
        default=[
            'mydwa',
            'dwb'])
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument(
        '--run-id',
        default=datetime.now(
            timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    parser.add_argument('--domain', type=int, default=86)
    parser.add_argument('--aggregate', type=Path)
    args = parser.parse_args()
    if args.aggregate:
        aggregate(args.aggregate.resolve())
        return
    if args.repeats < 1 or not 0 <= args.domain <= 200:
        parser.error('positive repeats and domain 0..200 required')
    if Path(args.run_id).name != args.run_id or args.run_id in ('.', '..'):
        parser.error('run-id must be one directory name')
    if os.environ.get('ROS_DISTRO') != 'humble':
        parser.error('source ROS Humble and workspace first')
    directory = HERE.parent / 'results' / 'dynamic' / args.run_id
    directory.mkdir(parents=True, exist_ok=False)
    audit = audit_configurations()
    frozen = protected_files()
    write_json(directory / 'protected_before.json', frozen)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{11345+args.domain}')
    manifest = dict(
        created_utc=datetime.now(
            timezone.utc).isoformat(),
        scenes=args.scenes,
        controllers=args.controllers,
        repeats=args.repeats,
        configuration_audit=audit,
        domain=args.domain,
        gazebo_master_uri=env['GAZEBO_MASTER_URI'],
        trials=[])
    for scene in args.scenes:
        for repeat in range(1, args.repeats + 1):
            for controller in (args.controllers if repeat %
                               2 else args.controllers[::-1]):
                manifest['trials'].append(
                    dict(
                        scene=scene,
                        controller=controller,
                        repeat=repeat,
                        directory=f'{scene}/{controller}/trial_{repeat:03}',
                        state='pending'))
    write_json(directory / 'manifest.json', manifest)
    for index, trial in enumerate(manifest['trials'], 1):
        print(
            f'[{index}/{len(manifest["trials"])}] {trial["directory"]}',
            flush=True)
        try:
            if protected_files() != frozen:
                raise RuntimeError('frozen baseline files changed')
            run_one(directory, trial, env)
        except Exception as exc:
            trial.update(state='system_error', error=repr(exc))
            path = directory / trial['directory']
            path.mkdir(parents=True, exist_ok=True)
            if not (path / 'summary.json').exists():
                write_json(
                    path / 'summary.json',
                    dict(
                        scene=trial['scene'],
                        controller=trial['controller'],
                        valid=False,
                        error=repr(exc)))
        write_json(directory / 'manifest.json', manifest)
        aggregate(directory)
        print(' -> ' + trial['state'], flush=True)
        if trial.get('cleanup_returncode', 1):
            raise RuntimeError(
                'cleanup failed; refusing contaminated next trial')
    after = protected_files()
    write_json(directory / 'protected_after.json', after)
    if after != frozen:
        raise RuntimeError('frozen files changed during suite')
    print(directory / 'report.md', flush=True)


if __name__ == '__main__':
    main()
