"""Snapshot existing files without changing user sources or benchmark records."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[2]
files = [p for p in ROOT.rglob('*') if p.is_file()
         and not any(s in p.parts for s in ('.git', '__pycache__'))
         and HERE not in p.parents and p.suffix != '.pyc']
files += [Path('/home/user/ros2_ws/install/my_dwa_controller/lib/libmy_dwa_controller.so')]
files += list(Path('/home/user/ros2_ws/scripts').glob('*.sh'))
manifest = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
output = HERE / 'evidence' / 'initial_files.json'
if output.exists():
    raise RuntimeError('initial snapshot already exists')
output.write_text(json.dumps(manifest, indent=2)+'\n')
print(f'Frozen {len(manifest)} existing files')