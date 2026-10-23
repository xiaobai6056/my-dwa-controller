"""Capture current baseline before any build, never overwrite this snapshot."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
WS=ROOT.parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def output(cmd):return subprocess.check_output(cmd,cwd=ROOT).decode()
def main():
    dest=HERE/'evidence/before.json'
    if dest.exists():raise RuntimeError('baseline snapshot already exists')
    existing={str(p):sha(p) for p in ROOT.rglob('*') if p.is_file() and HERE not in p.parents and '.git' not in p.parts and '__pycache__' not in p.parts and p.suffix!='.pyc'}
    installed=WS/'install/my_dwa_controller/lib/libmy_dwa_controller.so'
    existing[str(installed)]=sha(installed)
    for p in (WS/'scripts').glob('*'):
        if p.is_file():existing[str(p)]=sha(p)
    src={str(p.relative_to(ROOT)):sha(p) for d in ('src','include','config','test') for p in (ROOT/d).rglob('*') if p.is_file()}
    src['CMakeLists.txt']=sha(ROOT/'CMakeLists.txt')
    src['package.xml']=sha(ROOT/'package.xml')
    result=dict(existing=existing,source=src,original_plugin_sha256=sha(installed),git_status=output(['git','status','--short']),head=output(['git','rev-parse','HEAD']).strip(),branch=output(['git','branch','--show-current']).strip(),compiler=output(['c++','--version']),uname=output(['uname','-a']))
    (HERE/'evidence/git_before.diff').write_text(output(['git','diff','--binary']))
    shutil.copyfile(installed,HERE/'evidence/original_libmy_dwa_controller.so')
    shutil.copyfile(WS/'build/my_dwa_controller/CMakeCache.txt',HERE/'evidence/original_CMakeCache.txt')
    shutil.copyfile(WS/'build/my_dwa_controller/CMakeFiles/my_dwa_controller.dir/flags.make',HERE/'evidence/original_flags.make')
    dest.write_text(json.dumps(result,indent=2)+'\n')
    print(f'Frozen {len(existing)} existing files, original plugin {sha(installed)}')
if __name__=='__main__':main()