#!/usr/bin/env python3
import hashlib,json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    before=json.loads((HERE/'evidence/before.json').read_text());changed=[]
    for name,expected in before['files'].items():
        p=Path(name)
        if not p.is_file() or sha(p)!=expected:changed.append(name)
    model=json.loads((HERE/'evidence/actual_model.json').read_text());assert sha(Path(model['path']))==model['sha256']
    diff=subprocess.check_output(['git','diff','--binary'],cwd=ROOT).decode();status=subprocess.check_output(['git','status','--short'],cwd=ROOT).decode()
    result=dict(frozen_existing_files=len(before['files']),changed_existing_files=changed,git_diff_unchanged=diff==before['git_diff'],git_status_unchanged=status==before['git_status'],git_status=status,actual_robot_model_hash_matches=True)
    (HERE/'evidence/final_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));assert not changed and result['git_diff_unchanged'] and result['git_status_unchanged']
if __name__=='__main__':main()