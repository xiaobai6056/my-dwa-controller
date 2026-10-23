"""Append-only initial protection snapshot."""
import hashlib,json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];WS=ROOT.parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
out=HERE/'evidence/before.json'
if out.exists():raise RuntimeError('snapshot already exists')
files={str(p):sha(p) for p in ROOT.rglob('*') if p.is_file() and HERE not in p.parents and '.git' not in p.parts and '__pycache__' not in p.parts and p.suffix!='.pyc'}
for base in [WS/'scripts',WS/'install/my_dwa_controller',WS/'install_compile_comparison_20261020/release']:
    files.update({str(p):sha(p) for p in base.rglob('*') if p.is_file() and '__pycache__' not in p.parts})
robot=Path('/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf');files[str(robot)]=sha(robot)
out.write_text(json.dumps(dict(files=files,git_diff=subprocess.check_output(['git','diff','--binary'],cwd=ROOT).decode(),git_status=subprocess.check_output(['git','status','--short'],cwd=ROOT).decode()),indent=2)+'\n')
print(f'Frozen {len(files)} existing files')