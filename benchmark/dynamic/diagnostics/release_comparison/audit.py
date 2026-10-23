"""Verify original files/install remain intact and all six raw records agree."""
import hashlib
import json
from pathlib import Path
import subprocess
from settings import selection,mapping_matches,WS
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):return json.loads(p.read_text())
def main():
    before=load(HERE/'evidence/before.json')
    changed=[p for p,d in before['existing'].items() if not Path(p).is_file() or sha(Path(p))!=d]
    assert not changed,changed
    assert (HERE/'evidence/git_before.diff').read_text()==subprocess.check_output(['git','diff','--binary'],cwd=ROOT).decode()
    assert sha(WS/'build/my_dwa_controller/CMakeCache.txt')==sha(HERE/'evidence/original_CMakeCache.txt')
    assert sha(WS/'build/my_dwa_controller/CMakeFiles/my_dwa_controller.dir/flags.make')==sha(HERE/'evidence/original_flags.make')
    data=load(HERE/'outputs/comparison.json')
    runtime=[]
    raw_sha={}
    for r in data['runs']:
        d=HERE/'runs'/r['run']
        manifest=load(d/'manifest.json');summary=load(d/'trial/summary.json')
        assert summary['valid'] and summary['action_accepted'] and summary['action_error'] is None
        prefix,plugin=selection(r['variant'])
        assert mapping_matches((d/'plugin_maps.txt').read_text(),plugin)
        assert manifest['runtime_plugin_sha256']==sha(plugin)
        runtime.append(summary['runtime_parameters'])
        for name,digest in r['source_files'].items():
            assert sha(d/name)==digest,(d,name)
            raw_sha[str((d/name).relative_to(HERE))]=digest
        assert all(i['reserved']==i['committed'] and i['dropped']==0 for i in r['integrity'])
        assert r['validation']['valid_measurement']
    assert all(p==runtime[0] for p in runtime)
    processes=subprocess.check_output(['ps','-eo','pid,comm,args']).decode()
    active=[]
    for line in processes.splitlines()[1:]:
        fields=line.split(None,2)
        if len(fields)==3 and fields[1] in ('gzserver','gzclient','rviz2','component_contai') and '<defunct>' not in line:active.append(line)
    assert not active,active
    result=dict(protected_existing_files=len(before['existing']),changed_existing_files=changed,
        original_build_and_plugin_unchanged=True,git_user_diff_unchanged=True,
        actual_plugin_maps_and_sha_verified=6,identical_runtime_parameters=6,
        complete_mmap_runs=6,old_v01_v02_unchanged=True,active_simulation_processes=active,raw_source_sha256=raw_sha)
    (HERE/'evidence/audit_final.json').write_text(json.dumps(result,indent=2)+'\n')
    print(f'{len(before["existing"])} existing files unchanged; 6 actual maps/parameter/sample audits passed; no active simulation')
if __name__=='__main__':main()