"""Reuse the calibrated observer, adding only missing phase/initial-pose metadata."""
import difflib
import hashlib
import json
import os
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'dynamic/avoidability'
RELEASE=Path('/home/user/ros2_ws/install_compile_comparison_20261020/release')


def main():
    original=(OLD/'decision_probe.cpp').read_text()
    needle='<<",\\\"best_poses\\\":"<<poses(c.best)'
    replacement=('<<",\\\"rollout_initial_pose\\\":"<<poses(c.last.empty()?'
                 'std::vector<m::Pose2D>{}:std::vector<m::Pose2D>{c.last.front()})'+needle)
    assert original.count(needle)==1
    source=original.replace(needle,replacement)
    needle='<<"{\\\"sampled\\\":"<<c.sampled'
    replacement=('<<"{\\\"goal_phase\\\":\\\""<<c.goal_phase<<"\\\",\\\"weights\\\":["'
                 '<<c.settings.weights.obstacle<<\',\'<<c.settings.weights.speed<<\',\''
                 '<<c.settings.weights.goal<<\',\'<<c.settings.weights.path<<\',\''
                 '<<c.settings.weights.heading<<"],\\\"sampled\\\":"<<c.sampled')
    assert source.count(needle)==1
    source=source.replace(needle,replacement)
    (HERE/'decision_probe.cpp').write_text(source)
    (HERE/'evidence/probe_metadata.diff').write_text(''.join(difflib.unified_diff(
        original.splitlines(True),source.splitlines(True),fromfile=str(OLD/'decision_probe.cpp'),
        tofile=str(HERE/'decision_probe.cpp'))))
    includes=['-I'+str(HERE.parents[1]/'include'),'-I/usr/include/eigen3']
    includes+=['-I'+str(p) for p in sorted(Path('/opt/ros/humble/include').iterdir()) if p.is_dir()]
    command=['g++','-std=c++17','-O2','-shared','-fPIC','-pthread',*includes,
             str(HERE/'decision_probe.cpp'),'-L/opt/ros/humble/lib',
             '-Wl,-rpath,/opt/ros/humble/lib','-lnav2_costmap_2d_core','-ldl',
             '-o',str(HERE/'libdecision.so')]
    build=subprocess.run(command,capture_output=True,text=True)
    (HERE/'evidence/probe_build.log').write_text(build.stdout+build.stderr)
    build.check_returncode()
    env=dict(os.environ,LD_LIBRARY_PATH=str(RELEASE/'lib')+':'+os.environ.get('LD_LIBRARY_PATH',''))
    fixture=OLD/'probe_build/fixture'
    plain=subprocess.run([str(fixture)],env=env,capture_output=True,text=True)
    calibration=HERE/'evidence/probe_calibration';calibration.mkdir(exist_ok=False)
    env.update(DECISION_DIR=str(calibration),NAVPERF_MYDWA_LIB=str(OLD/'probe_build/libfixture.so'),
               LD_PRELOAD=str(HERE/'libdecision.so'))
    observed=subprocess.run([str(fixture)],env=env,capture_output=True,text=True)
    plain.check_returncode();observed.check_returncode()
    assert plain.stdout==observed.stdout,(plain.stdout,observed.stdout)
    records=[json.loads(s) for p in calibration.glob('decisions_*.jsonl') for s in p.read_text().splitlines()]
    assert records and all('rollout_initial_pose' in r for r in records)
    manifest=dict(command=command,source_sha256=hashlib.sha256(source.encode()).hexdigest(),
        original_source_sha256=hashlib.sha256(original.encode()).hexdigest(),
        observer_sha256=hashlib.sha256((HERE/'libdecision.so').read_bytes()).hexdigest(),
        fixture_return_passthrough=plain.stdout==observed.stdout,
        fixture_plain_stdout=plain.stdout,fixture_observed_stdout=observed.stdout,
        calibration_records=len(records),
        additions='Only existing CycleDiagnostics phase/weights and trajectory initial pose are serialized; original methods and returns unchanged.')
    (HERE/'evidence/probe_build.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({k:v for k,v in manifest.items() if k!='command'},indent=2))


if __name__=='__main__':main()
