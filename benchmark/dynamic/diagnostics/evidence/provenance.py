"""Capture source/binary/environment provenance after serialized diagnostic runs."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
HERE=Path(__file__).resolve().parents[1]
ROOT=HERE.parents[2]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
measured=HERE/'evidence/measured_probe'
measured.mkdir(exist_ok=True)
for source in (HERE/'perf_probe.cpp',HERE/'build_probe.sh',HERE/'perf_build/libnavperf.so'):
    dest=measured/source.name
    if dest.exists():assert sha(dest)==sha(source)
    else:shutil.copyfile(source,dest)
(measured/'AMENT_IGNORE').touch()
files=[*measured.glob('*'),Path('/opt/ros/humble/lib/libdwb_core.so'),Path('/opt/ros/humble/include/rclcpp/rclcpp/rate.hpp'),ROOT/'include/my_dwa_controller/controller_diagnostics.hpp',Path('/home/user/ros2_ws/build/my_dwa_controller/CMakeFiles/my_dwa_controller.dir/flags.make'),Path('/home/user/ros2_ws/build/my_dwa_controller/CMakeFiles/trajectory_scorer.dir/flags.make')]
info=dict(files={str(f):sha(f) for f in files if f.is_file()},
    environment={k:os.environ.get(k) for k in ('DISPLAY','WAYLAND_DISPLAY','XDG_RUNTIME_DIR','WSL_DISTRO_NAME')},
    capture_note='Probe source/binary unchanged since actual runs; hashes appended here without rewriting original manifests. Single serialized FollowPath controller only; no concurrent-first-call guarantee.')
for name,command in dict(uname=['uname','-a'],packages=['dpkg-query','-W','ros-humble-nav2-controller','ros-humble-rclcpp','gazebo','ros-humble-dwb-core'],processes=['ps','-eo','pid,comm,args'],build=['cat',str(files[-2]),str(files[-1])]).items():
    result=subprocess.run(command,capture_output=True,text=True)
    (HERE/f'evidence/{name}.txt').write_text(result.stdout+result.stderr)
shutil.copyfile(Path('/opt/ros/humble/include/rclcpp/rclcpp/rate.hpp'),HERE/'evidence/installed_rate.hpp')
info['surviving_simulation_processes']=[l for l in (HERE/'evidence/processes.txt').read_text().splitlines() if any(f' {n} ' in l for n in ('gzserver','gzclient','rviz2','component_contai')) and '<defunct>' not in l]
assert not info['surviving_simulation_processes'],info['surviving_simulation_processes']
(HERE/'evidence/provenance.json').write_text(json.dumps(info,indent=2)+'\n')
latest=sorted((HERE/'perf_build/fixture').glob('calibration_*/probe_calibration.json'),key=lambda f:f.stat().st_mtime)[-1]
shutil.copyfile(latest,HERE/'evidence/probe_calibration_reproduced.json')
print('Source/binary provenance saved; no live simulation remains')