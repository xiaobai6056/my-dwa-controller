"""Freeze all pre-existing source/data and selected binary identities."""
import hashlib,json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
WS=ROOT.parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
dest=HERE/'evidence/before.json'
if dest.exists():raise RuntimeError('snapshot already exists')
files={str(p):sha(p) for p in ROOT.rglob('*') if p.is_file() and HERE not in p.parents and '.git' not in p.parts and '__pycache__' not in p.parts and p.suffix!='.pyc'}
for base in (WS/'scripts', WS/'install/my_dwa_controller',WS/'install_compile_comparison_20261020/release'):
    for p in base.rglob('*'):
        if p.is_file():files[str(p)]=sha(p)
files['/opt/ros/humble/lib/libdwb_core.so']=sha(Path('/opt/ros/humble/lib/libdwb_core.so'))
result=dict(files=files,git_status=subprocess.check_output(['git','status','--short'],cwd=ROOT).decode(),head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT).decode().strip(),git_diff=subprocess.check_output(['git','diff','--binary'],cwd=ROOT).decode())
dest.write_text(json.dumps(result,indent=2)+'\n')
print(f'Frozen {len(files)} existing files')