"""Thin acquisition adapter: original scenes, runner, launch and cleanup logic.

Only missing observations are added. Recording starts after the original dynamic
GT-subscriber preflight, so its isolation assertion is retained unchanged.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path

import signal
import shlex
import subprocess
import sys
import time


# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

HERE=Path(__file__).resolve().parent
BENCH=HERE.parent
DYNAMIC=BENCH/'dynamic'
PREFIX=(ARCHIVE_WS / 'install_compile_comparison_20261020/release')
PLUGIN=PREFIX/'lib/libmy_dwa_controller.so'
sys.path.insert(0,str(BENCH))
from run_suite import stop_process
from dynamic.run_suite import cleanup_processes
from scenes import load_scene, resources
sys.path.insert(0,str(DYNAMIC/'timing_sweep'))
from rules import scene as timed_scene, mapping_matches
from scoring import admission_errors

TOPICS=['/clock','/local_costmap/costmap','/local_costmap/costmap_raw',
        '/local_costmap/costmap_updates','/tf','/tf_static','/odom','/scan',
        '/benchmark/model_states','/benchmark/contacts','/gazebo/model_states',
        '/cmd_vel_nav','/cmd_vel','/plan']


def write(path, value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def commands(case, output):
    scene_id=case.split('_')[0]
    if scene_id.startswith('S'):
        scene=load_scene(scene_id)
        source=(BENCH/'start_scene.sh').read_text()
        needle='source "${MYDWA_WS}/install/setup.bash"\n'
        assert source.count(needle)==1
        source=source.replace(needle,needle+f'source {PREFIX}/share/my_dwa_controller/local_setup.bash\n')
        runner=[sys.executable,str(BENCH/'run_trial.py'),'--controller','mydwa',
            '--start',*map(str,scene['start']),'--goal',*map(str,scene['goal']),
            '--timeout',str(scene['timeout']),'--ready-timeout','90',
            '--results-dir',str(output),'--trial-id','trial','--record-navigation',
            '--world-file',str(resources(scene)[0]),'--startup-script',str(BENCH/'start_scene.sh')]
    else:
        source=(DYNAMIC/'timing_sweep/launch_scene.sh').read_text()
        if scene_id=='D01':
            start=int(case.split('_t')[1]);scene=timed_scene(start)
            runner=[sys.executable,str(DYNAMIC/'timing_sweep/trial_entry.py'),
                '--start-time',str(start),'--scene',scene_id,'--controller','mydwa',
                '--output',str(output/'trial')]
        else:
            scene=dict(json.loads((DYNAMIC/'scenarios.json').read_text())[scene_id],id=scene_id)
            runner=[sys.executable,str(DYNAMIC/'run_trial.py'),'--scene',scene_id,
                    '--controller','mydwa','--output',str(output/'trial')]
    source = source.replace(
        'REPO_ROOT=$(git -C "$(dirname -- "${BASH_SOURCE[0]}")" rev-parse --show-toplevel)',
        'REPO_ROOT=' + shlex.quote(str(BENCH.parent)))
    resolved=output/'resolved_launch.sh';resolved.write_text(source)
    return scene,['bash',str(resolved),'mydwa',scene_id,'headless'],runner


def run(case, tag, domain):
    output=HERE/'runs'/f'{tag}_{case}'
    output.mkdir(parents=True,exist_ok=False)
    scene,launch_cmd,runner_cmd=commands(case,output)
    write(output/'effective_scene.json',scene)
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),GAZEBO_MASTER_URI=f'http://127.0.0.1:{11345+domain}',
             MYDWA_DIAG_PREFIX=str(PREFIX),PYTHONDONTWRITEBYTECODE='1')
    control_env=dict(env,LD_PRELOAD=str(HERE/'libdecision.so'),DECISION_DIR=str(output),
                     NAVPERF_MYDWA_LIB=str(PLUGIN))
    bag_cmd=['ros2','bag','record','--use-sim-time','--qos-profile-overrides-path',
             str(HERE/'bag_qos.yaml'),'-o',str(output/'bag'),*TOPICS]
    manifest=dict(case=case,started_utc=datetime.now(timezone.utc).isoformat(),
        effective_scene=scene,launch_command=launch_cmd,runner_command=runner_cmd,bag_command=bag_cmd,
        domain=domain,gazebo_master_uri=env['GAZEBO_MASTER_URI'],
        plugin=str(PLUGIN),plugin_sha256=hashlib.sha256(PLUGIN.read_bytes()).hexdigest(),
        observer_sha256=hashlib.sha256((HERE/'libdecision.so').read_bytes()).hexdigest(),
        recording_start_rule='After original runner dispatch log (original GT subscriber guard retained)',
        bag_source_time_rule='Stamped messages retain source timestamps; ModelStates has no source timestamp.',
        control_changes=False)
    launch=runner=bag=None;interrupted=False
    def stop(*_):
        nonlocal interrupted
        interrupted=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    with (output/'launch.log').open('w') as ll,(output/'trial.log').open('w') as tl,(output/'bag.log').open('w') as bl:
        try:
            launch=subprocess.Popen(launch_cmd,env=control_env,stdout=ll,stderr=subprocess.STDOUT,start_new_session=True)
            runner=subprocess.Popen(runner_cmd,env=env,stdout=tl,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+90+scene['timeout']+45;next_check=0.
            while runner.poll() is None and not interrupted:
                now=time.monotonic()
                if now>deadline or launch.poll() is not None:
                    manifest['supervisor_error']='launch_exited_or_wall_timeout';break
                if 'symbol lookup error' in (output/'launch.log').read_text():
                    manifest['supervisor_error']='observer_link_failure';break
                text=(output/'trial.log').read_text()
                if bag is None and ('dispatch sim=' in text or 'Trial trial:' in text):
                    bag=subprocess.Popen(bag_cmd,env=env,stdout=bl,stderr=subprocess.STDOUT,start_new_session=True)
                    manifest['bag_start_wall_monotonic']=time.monotonic()
                if now>=next_check:
                    next_check=now+.5
                    for proc in Path('/proc').iterdir():
                        if not proc.name.isdigit():continue
                        try:
                            if (proc/'exe').resolve().name!='component_container_isolated':continue
                            penv=(proc/'environ').read_bytes().split(b'\0')
                            if f'ROS_DOMAIN_ID={domain}'.encode() not in penv:continue
                            maps=(proc/'maps').read_text()
                            if PLUGIN.name not in maps:continue
                            if not mapping_matches(maps,PLUGIN):raise RuntimeError('wrong loaded plugin')
                            if not manifest.get('runtime_mapping_verified'):
                                (output/'plugin_maps.txt').write_text(maps)
                                manifest.update(runtime_mapping_verified=True,runtime_plugin_pid=int(proc.name))
                        except (FileNotFoundError,PermissionError,ProcessLookupError):pass
                time.sleep(.1)
            manifest['runner_returncode']=runner.poll()
            if not manifest.get('runtime_mapping_verified'):manifest['supervisor_error']='runtime_plugin_not_verified'
            if bag is None:manifest['recording_error']='task_never_dispatched'
        except Exception as exc:
            manifest['supervisor_error']=repr(exc)
        finally:
            if bag is not None:
                stop_process(bag,10.)
                manifest['bag_returncode']=bag.returncode
                if not (output/'bag/metadata.yaml').exists():manifest['recording_error']='missing_bag_metadata'
            cleanup_processes(runner,launch,env,ll,manifest)
            manifest['interrupted']=interrupted
            manifest['finished_utc']=datetime.now(timezone.utc).isoformat()
            summary=output/'trial/summary.json'
            if summary.exists():
                z=json.loads(summary.read_text())
                manifest['observed_outcome']={k:z.get(k) for k in
                    ('valid','status','action_result','stop_reason','error','navigation_start_sim_s','navigation_end_sim_s')}
                expected={k:v for k,v in json.loads((BENCH/'dynamic/avoidability/runs/diagnostic_t4_linked/trial/summary.json').read_text())['runtime_parameters']['controller_server'].items()}
                manifest['controller_parameters_match_historical']=z.get('runtime_parameters',{}).get('controller_server')==expected
                manifest['acquisition_errors']=admission_errors(z,manifest)
            else:
                manifest['acquisition_errors']=['missing_trial_summary']
            write(output/'manifest.json',manifest)
    print(json.dumps(dict(case=case,run=str(output),**manifest.get('observed_outcome',{}),
        supervisor_error=manifest.get('supervisor_error'),bag=(output/'bag/metadata.yaml').exists(),
        cleanup_returncode=manifest.get('cleanup_returncode')),ensure_ascii=False),flush=True)
    return not manifest['acquisition_errors']


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cases',nargs='+',choices=['S04','S05','D01_t2','D01_t3','D01_t4','D01_t5','D01_t6','D02'],required=True)
    p.add_argument('--tag',required=True);p.add_argument('--domain',type=int,default=93)
    a=p.parse_args()
    if Path(a.tag).name!=a.tag or a.tag in ('.','..'):p.error('tag must be a fresh directory prefix')
    if not 0<=a.domain<=200:p.error('domain outside supported range')
    if os.environ.get('ROS_DISTRO')!='humble':p.error('source Humble and workspace before invocation')
    for case in a.cases:
        if not run(case,a.tag,a.domain):return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
