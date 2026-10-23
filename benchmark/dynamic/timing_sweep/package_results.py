"""Archive all new timing-scan raw data/evidence without overwriting prior files."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile
import zipfile
from rules import ROOT,selection
HERE=Path(__file__).resolve().parent
for controller in ('mydwa','dwb'):
    _,source=selection(controller)
    destination=HERE/f'evidence/{controller}_{source.name}'
    if destination.exists():
        assert hashlib.sha256(source.read_bytes()).digest()==hashlib.sha256(destination.read_bytes()).digest()
    else:shutil.copyfile(source,destination)
for source,name in [
 (Path('/home/user/ros2_ws/build_compile_comparison_20261020/release/compile_commands.json'),'release_compile_commands.json'),
 (Path('/home/user/ros2_ws/build_compile_comparison_20261020/release/CMakeCache.txt'),'release_CMakeCache.txt'),
 (Path('/home/user/ros2_ws/build_compile_comparison_20261020/release/CMakeFiles/my_dwa_controller.dir/flags.make'),'release_flags.make'),
 (HERE.parent/'scenarios.json','original_scenarios.json')]:
    shutil.copyfile(source,HERE/'evidence'/name)
versions=subprocess.run(['dpkg-query','-W','ros-humble-nav2-controller','ros-humble-dwb-core','gazebo'],text=True,capture_output=True)
(HERE/'evidence/versions.txt').write_text(versions.stdout+versions.stderr)
archive=HERE/'outputs/raw_timing_sweep.tar.gz'
with tarfile.open(archive,'x:gz') as tf:
    tf.add(HERE/'runs',arcname='runs')
    tf.add(HERE/'sequence_first.json',arcname='sequence_first.json')
    tf.add(HERE/'evidence',arcname='evidence',filter=lambda info:None if '__pycache__' in info.name else info)
with tarfile.open(archive,'r:gz') as tf:
    names=tf.getnames()
    assert sum(n.endswith('/trial/summary.json') for n in names)==30
    assert sum(n.endswith('/manifest.json') for n in names)==30
result=dict(archive=archive.name,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),bytes=archive.stat().st_size,members=len(names),actual_runs=30)
(HERE/'outputs/archive_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
zipdest=Path('/path/to/local-experiment-inputs/d01_timing_sweep_20261020.zip')
with zipfile.ZipFile(zipdest,'x',zipfile.ZIP_DEFLATED) as zf:
    for p in HERE.rglob('*'):
        if p.is_file() and not any(x in p.relative_to(HERE).parts for x in ('runs','__pycache__')):
            zf.write(p,p.relative_to(HERE))
print(json.dumps(result,indent=2));print(zipdest)