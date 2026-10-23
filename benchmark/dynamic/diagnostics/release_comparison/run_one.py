#!/usr/bin/env python3
"""One actual dynamic navigation, optional GUI, separate V0.2 diagnostic records."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
DYNAMIC = HERE.parents[1]
sys.path.insert(0, str(DYNAMIC.parent))
from dynamic.run_suite import cleanup_processes  # noqa: E402
from dynamic.evaluation import calculate  # noqa: E402
from settings import selection, mapping_matches, completed_trial


def launch_command(scene, controller, gui):
    return ['bash', str(HERE / 'launch_scene.sh'), controller, scene,
            'gui' if gui else 'headless']


def exit_status(manifest, recorded):
    """Navigation failure can be valid data; execution failure cannot be success."""
    if manifest.get('interrupted'):
        return 130
    return int(bool(manifest.get('cleanup_returncode', 1) or
                    manifest.get('supervisor_error') or
                    manifest.get('runner_returncode') not in (0,2) or not recorded))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['unoptimized','release'], required=True)
    parser.add_argument('--scenario', choices=['D01', 'D02'], default='D01')
    parser.add_argument('--controller', choices=['mydwa', 'dwb'], default='mydwa')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--gui', action='store_true')
    mode.add_argument('--headless', action='store_true')
    parser.add_argument('--run-id', default=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    parser.add_argument('--domain', type=int, default=91)
    parser.add_argument('--hold-seconds', type=float)
    parser.add_argument('--preload', type=Path, help='optional observation-only performance shim')
    args = parser.parse_args()
    prefix, plugin = selection(args.variant)
    if not plugin.is_file(): parser.error('selected plugin not built')
    if args.controller != 'mydwa' or args.scenario != 'D01' or args.gui: parser.error('comparison requires D01/mydwa/headless')
    if Path(args.run_id).name != args.run_id or args.run_id in ('.', '..'):
        parser.error('run-id must be a new single directory name')
    if not 0 <= args.domain <= 200:
        parser.error('domain must be in 0..200')
    if args.gui and not os.environ.get('DISPLAY'):
        parser.error('GUI requires DISPLAY / WSLg')
    output = HERE / 'runs' / args.run_id
    output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{11345+args.domain}')
    env['MYDWA_DIAG_PREFIX'] = str(prefix)
    env['NAVPERF_MYDWA_LIB'] = str(plugin)
    control_env = dict(env)
    if args.preload:
        control_env['LD_PRELOAD'] = str(args.preload.resolve())
        control_env['NAVPERF_DIR'] = str(output)
    command = launch_command(args.scenario, args.controller, args.gui)
    runner_command = [sys.executable, str(DYNAMIC / 'run_trial.py'), '--scene', args.scenario,
                      '--controller', args.controller, '--output', str(output / 'trial')]
    manifest = dict(arguments=vars(args) | {'preload': str(args.preload) if args.preload else None},
        classification='release_build_comparison_not_formal', selected_plugin_path=str(plugin), selected_prefix=str(prefix), launch_command=command,
        runner_command=runner_command, domain=args.domain, gazebo_master_uri=env['GAZEBO_MASTER_URI'],
        current_plugin_sha256=hashlib.sha256(plugin.read_bytes()).hexdigest())
    if args.preload:
        manifest['preload_sha256'] = hashlib.sha256(args.preload.resolve().read_bytes()).hexdigest()
        manifest['probe_source_sha256'] = hashlib.sha256((DYNAMIC/'diagnostics/perf_probe.cpp').read_bytes()).hexdigest()
    manifest['dwb_plugin_sha256'] = hashlib.sha256(Path('/opt/ros/humble/lib/libdwb_core.so').read_bytes()).hexdigest()
    launch = runner = probe = None
    recorded = False
    interrupted = False
    def stop(*_):
        nonlocal interrupted
        interrupted = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    with (output / 'launch.log').open('w') as launch_log, (output / 'trial.log').open('w') as trial_log, (output / 'probe.log').open('w') as probe_log:
        try:
            launch = subprocess.Popen(command, env=control_env, stdout=launch_log,
                stderr=subprocess.STDOUT, start_new_session=True)
            if args.gui:
                with (output / 'camera.log').open('w') as camera_log:
                    camera = subprocess.run([sys.executable, str(HERE / 'camera_view.py')],
                        env=env, stdout=camera_log, stderr=subprocess.STDOUT, timeout=22)
                    manifest['camera_returncode'] = camera.returncode
            probe = subprocess.Popen([sys.executable, str(DYNAMIC / 'diagnostics/cmd_probe.py'),
                str(output / 'native_commands.jsonl')], env=env, stdout=probe_log,
                stderr=subprocess.STDOUT, start_new_session=True)
            runner = subprocess.Popen(runner_command, env=env, stdout=trial_log,
                stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+450
            next_mapping_check = 0.
            while runner.poll() is None and not interrupted:
                if time.monotonic() > deadline or launch.poll() is not None:
                    manifest['supervisor_error'] = 'launch_exited_or_wall_timeout'
                    break
                if time.monotonic() >= next_mapping_check:
                    next_mapping_check = time.monotonic()+.5
                    for proc in Path('/proc').iterdir():
                        if not proc.name.isdigit(): continue
                        try:
                            if (proc/'exe').resolve().name != 'component_container_isolated': continue
                            maps=(proc/'maps').read_text()
                            if 'libmy_dwa_controller.so' not in maps: continue
                            if not mapping_matches(maps, plugin):
                                manifest['supervisor_error']='incorrect_runtime_plugin_mapping'
                                raise RuntimeError('plugin loaded from incorrect path')
                            if 'runtime_plugin_pid' not in manifest:
                                (output/'plugin_maps.txt').write_text(maps)
                                manifest['runtime_plugin_pid']=int(proc.name)
                                manifest['runtime_plugin_sha256']=hashlib.sha256(plugin.read_bytes()).hexdigest()
                                manifest['runtime_mapping_verified']=True
                        except (FileNotFoundError,PermissionError,ProcessLookupError): pass
                time.sleep(.1)
            if not manifest.get('runtime_mapping_verified'):
                manifest['supervisor_error']='runtime_plugin_not_verified'
            manifest['runner_returncode'] = runner.poll()
            manifest['interrupted'] = interrupted
            if (output / 'trial' / 'summary.json').exists() and completed_trial(runner.poll(), json.loads((output / 'trial' / 'summary.json').read_text())):
                metrics = calculate(output / 'trial')
                (output / 'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
                recorded = metrics.get('termination') != 'system_error'
                manifest['recorded_outcome'] = recorded
                print(json.dumps(metrics, indent=2), flush=True)
                if args.gui:
                    inventory = subprocess.run(['xwininfo', '-root', '-tree'], env=env,
                        capture_output=True, text=True, timeout=10)
                    (output / 'x_windows.txt').write_text(inventory.stdout+inventory.stderr)
                hold = args.hold_seconds if args.hold_seconds is not None else (60 if args.gui else 0)
                print(f'Result saved: {output}; holding GUI for {hold}s (Ctrl+C cleans up).', flush=True)
                until = time.monotonic()+hold
                while time.monotonic() < until and not interrupted:
                    time.sleep(.1)
        finally:
            cleanup_processes(runner, launch, env, launch_log, manifest)
            if probe is not None:
                from dynamic.run_suite import stop_process
                stop_process(probe, 3.)
            (output / 'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    return exit_status(manifest, recorded)


if __name__ == '__main__':
    raise SystemExit(main())