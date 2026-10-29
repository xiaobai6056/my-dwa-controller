"""Run one opt-in smoke per original scene using existing launch/trial helpers."""

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

import yaml


# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASELINE = ROOT / 'benchmark/baseline_diagnosis_20261021'
PREFIX = (ARCHIVE_WS / 'install_scan_emergency/my_dwa_controller')
PLUGIN = PREFIX / 'lib/libmy_dwa_controller.so'
sys.path.insert(0, str(BASELINE))
import run_missing  # noqa: E402


def write(path, value):
    """Save a new run's reproducibility metadata."""
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def prepare(case, output):
    """Reuse existing scene/runner selection, changing only opt-in config and prefix."""
    run_missing.PREFIX = PREFIX
    scene, launch, runner = run_missing.commands(case, output)
    config_path = ROOT / 'config/nav2_params.yaml'
    params = yaml.safe_load(config_path.read_text())
    params['controller_server']['ros__parameters']['FollowPath']['path_conflict'] = {
        'enabled': True}
    params['controller_server']['ros__parameters']['FollowPath']['scan_tracker'] = {
        'enabled': True}
    selected = output / 'params_enabled.yaml'
    selected.write_text(yaml.safe_dump(params, sort_keys=False))
    launch_file = Path(launch[1])
    source = launch_file.read_text()
    for original in ('$bench/../config/nav2_params.yaml',
                     '$bench/../../config/nav2_params.yaml'):
        source = source.replace(original, str(selected))
    # Put the selected prefix first even if the caller sourced it before underlay.
    anchor = 'source "${MYDWA_WS}/scripts/_baseline_common.sh"'
    source = source.replace(
        anchor, f'export AMENT_PREFIX_PATH={PREFIX}:$AMENT_PREFIX_PATH\n'
        f'export LD_LIBRARY_PATH={PREFIX}/lib:$LD_LIBRARY_PATH\n' + anchor)
    launch_file.write_text(source)
    if case.startswith('D01'):
        # The old metadata adapter hardcodes a release path; adapt its selection
        # in this process only so its new summary describes the actually loaded DLL.
        entry = (
            f"import sys; sys.path.insert(0, {str(run_missing.DYNAMIC / 'timing_sweep')!r}); "
            f"import rules; rules.RELEASE=rules.Path({str(PREFIX)!r}); "
            "import trial_entry; raise SystemExit(trial_entry.main())")
        runner = [sys.executable, '-c', entry, *runner[2:]]
    return scene, launch, runner


def run(case, tag, domain):
    """Supervise one original trial, retaining failures and domain-aware cleanup."""
    output = HERE / 'runs' / f'{tag}_{case}'
    output.mkdir(parents=True, exist_ok=False)
    scene, launch_cmd, runner_cmd = prepare(case, output)
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain),
               GAZEBO_MASTER_URI=f'http://127.0.0.1:{11345 + domain}',
               MYDWA_DIAG_PREFIX=str(PREFIX), PYTHONDONTWRITEBYTECODE='1')
    manifest = dict(
        case=case, scene=scene, launch_command=launch_cmd, runner_command=runner_cmd,
        domain=domain, gazebo_master_uri=env['GAZEBO_MASTER_URI'],
        started_utc=datetime.now(timezone.utc).isoformat(),
        plugin=str(PLUGIN), plugin_sha256=hashlib.sha256(PLUGIN.read_bytes()).hexdigest(),
        git_head=subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        parameter_changes={
            'FollowPath.path_conflict.enabled': True, 'FollowPath.scan_tracker.enabled': True},
        original_yaml_sha256=hashlib.sha256(
            (ROOT / 'config/nav2_params.yaml').read_bytes()).hexdigest())
    manifest['source_sha256'] = {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for directory in ('include', 'src') for p in (ROOT / directory).rglob('*') if p.is_file()}
    import zipfile
    with zipfile.ZipFile(output / 'sources.zip', 'w', zipfile.ZIP_DEFLATED) as z:
        for folder in ('include', 'src', 'test'):
            for p in (ROOT / folder).rglob('*'):
                if p.is_file():
                    z.write(p, str(p.relative_to(ROOT)))
        for name in ('CMakeLists.txt', 'package.xml'):
            z.write(ROOT / name, name)
    (output / PLUGIN.name).write_bytes(PLUGIN.read_bytes())
    write(output / 'effective_scene.json', scene)
    write(output / 'manifest.json', manifest)
    interrupted = False

    def stop(*_):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    launch = runner = bag = None
    with (output / 'launch.log').open('w') as ll, (output / 'trial.log').open('w') as tl:
        try:
            launch = subprocess.Popen(launch_cmd, env=env, stdout=ll,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            bag = subprocess.Popen(
                ['ros2', 'bag', 'record', '--use-sim-time',
                 '--qos-profile-overrides-path', str(BASELINE / 'bag_qos.yaml'),
                 '-o', str(output / 'bag'), '/clock', '/scan', '/tf', '/tf_static',
                 '/odom', '/plan', '/cmd_vel_nav', '/cmd_vel', '/local_costmap/costmap'],
                env=env, stdout=(output / 'bag.log').open('w'),
                stderr=subprocess.STDOUT, start_new_session=True)
            runner = subprocess.Popen(runner_cmd, env=env, stdout=tl,
                                      stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 90 + scene['timeout'] + 45
            while runner.poll() is None and not interrupted:
                if time.monotonic() > deadline or launch.poll() is not None:
                    manifest['supervisor_error'] = 'launch_exited_or_wall_timeout'
                    break
                if not manifest.get('runtime_mapping_verified'):
                    for proc in Path('/proc').iterdir():
                        if not proc.name.isdigit():
                            continue
                        try:
                            if (proc / 'exe').resolve().name != 'component_container_isolated':
                                continue
                            penv = (proc / 'environ').read_bytes().split(b'\0')
                            if f'ROS_DOMAIN_ID={domain}'.encode() not in penv:
                                continue
                            mappings = (proc / 'maps').read_text()
                            if PLUGIN.name not in mappings:
                                continue
                            if not run_missing.mapping_matches(mappings, PLUGIN):
                                raise RuntimeError('wrong loaded plugin')
                            (output / 'plugin_maps.txt').write_text(mappings)
                            manifest['runtime_mapping_verified'] = True
                        except (FileNotFoundError, ProcessLookupError, PermissionError):
                            pass
                time.sleep(0.2)
            manifest['runner_returncode'] = runner.poll()
        except Exception as exc:
            manifest['supervisor_error'] = repr(exc)
        finally:
            if bag is not None:
                run_missing.stop_process(bag, 10.)
                manifest['bag_returncode'] = bag.returncode
            run_missing.cleanup_processes(runner, launch, env, ll, manifest)
            manifest['interrupted'] = interrupted
            manifest['finished_utc'] = datetime.now(timezone.utc).isoformat()
            summary_path = output / 'trial/summary.json'
            if summary_path.exists():
                summary = json.loads(summary_path.read_text())
                manifest['outcome'] = {key: summary.get(key) for key in (
                    'valid', 'status', 'action_result', 'stop_reason', 'error',
                    'navigation_start_sim_s', 'navigation_end_sim_s')}
            write(output / 'manifest.json', manifest)
    print(json.dumps({'case': case, 'output': str(output), **manifest}, ensure_ascii=False),
          flush=True)
    return (manifest.get('cleanup_returncode') == 0 and not interrupted
            and manifest.get('runtime_mapping_verified', False)
            and not manifest.get('supervisor_error'))


def main():
    """Run requested original scenes once without parameter search."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', nargs='+', required=True,
                        choices=['D01_t2', 'D01_t4', 'D01_t6', 'S04', 'S05'])
    parser.add_argument('--tag', required=True)
    parser.add_argument('--domain', type=int, default=96)
    args = parser.parse_args()
    if Path(args.tag).name != args.tag or args.tag in ('.', '..'):
        parser.error('tag must be a new directory prefix')
    if not 0 <= args.domain <= 200:
        parser.error('domain outside supported range')
    for case in args.cases:
        if not run(case, args.tag, args.domain):
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
