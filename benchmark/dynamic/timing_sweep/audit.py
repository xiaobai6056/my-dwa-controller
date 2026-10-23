"""Read-only consistency/freeze audit for all 30 timing-scan measurements."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from rules import ROOT,scene,selection,original_scene,verify_motion,mapping_matches
HERE=Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):return json.loads(p.read_text())
def jsonl(p):return [json.loads(line) for line in p.read_text().splitlines() if line]

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--tag',default='first');args=parser.parse_args()
    if Path(args.tag).name!=args.tag or args.tag in ('.','..'):parser.error('invalid tag')
    before=load(HERE/'evidence/before.json')
    changed=[p for p,digest in before['files'].items() if not Path(p).is_file() or sha(Path(p))!=digest]
    assert not changed,changed
    assert subprocess.check_output(['git','diff','--binary'],cwd=ROOT).decode()==before['git_diff']
    assert scene(4)==original_scene()
    result=load(HERE/('outputs' if args.tag=='first' else f'outputs_{args.tag}')/'results.json')
    assert result['complete'] and len(result['records'])==30
    sequence=load(HERE/f'sequence_{args.tag}.json')
    assert len(sequence['scheduled'])==len(sequence['runs'])==30 and 'finished' in sequence
    assert all(r['returncode']==0 and r['cleanup_returncode']==0 and r['motion_valid'] for r in sequence['runs'])
    runtime={};source_hashes={};actual=[]
    for row in result['records']:
        assert row['valid'] and row['motion_validation']['valid']
        d=HERE/'runs'/row['run'];summary=load(d/'trial/summary.json');original=load(d/'trial/summary_from_original_runner.json')
        for key,value in original.items():
            if key=='mydwa_library_sha256':assert summary['original_underlay_mydwa_sha256']==value
            else:assert summary[key]==value,(row['run'],key)
        _,plugin=selection(row['controller'])
        assert mapping_matches((d/'plugin_maps.txt').read_text(),plugin)
        assert summary['selected_plugin_sha256']==row['runtime_plugin_sha256']==sha(plugin)
        assert summary['effective_scene']==scene(row['start_time_s'])==load(d/'effective_scene.json')
        runtime.setdefault(row['controller'],summary['runtime_parameters'])
        assert runtime[row['controller']]==summary['runtime_parameters']
        v=verify_motion(jsonl(d/'trial/ground_truth.jsonl'),jsonl(d/'trial/motion.jsonl'),summary['effective_scene'],summary['navigation_start_sim_s'])
        assert v==row['motion_validation']==load(d/'motion_validation.json')
        assert row['metrics']==load(d/'metrics.json')
        if row['metrics']['success']:
            assert summary['action_result']=='SUCCEEDED' and summary['action_status_code']==4
        if row['contact']:
            contacts=jsonl(d/'trial/contacts.jsonl')
            assert min(c['stamp'] for c in contacts)==row['contact']['source_stamp']
        for name,digest in row['sources_sha256'].items():
            assert sha(d/name)==digest,(row['run'],name)
            source_hashes[str((d/name).relative_to(HERE))]=digest
        actual.append(dict(run=row['run'],configured_start_s=row['start_time_s'],observed_start_s=v['observed_first_gt_motion_s'],plugin_sha256=sha(plugin)))
    for start in (2,3,4,5,6):
        for controller in ('mydwa','dwb'):
            assert sum(r['start_time_s']==start and r['controller']==controller for r in result['records'])==3
    ps=subprocess.check_output(['ps','-eo','pid,comm,args']).decode()
    active=[line for line in ps.splitlines()[1:] if len(line.split(None,2))==3 and line.split(None,2)[1] in ('gzserver','gzclient','rviz2','component_contai') and '<defunct>' not in line]
    assert not active,active
    output=dict(existing_files_unchanged=len(before['files']),changed_existing_files=changed,user_git_diff_unchanged=True,
        complete_valid_actual_trials=30,old_D01_v01_v02_release_comparison_unchanged=True,
        runtime_parameters_equal_per_controller=True,real_plugin_mapping_and_hash_checks=30,
        actual_GT_motion_checks=actual,raw_source_sha256=source_hashes,active_simulations=active)
    (HERE/f'evidence/audit_{args.tag}.json').write_text(json.dumps(output,indent=2)+'\n')
    print(f'{len(before["files"])} existing hashes unchanged; 30 real plugin/motion/action/source audits passed; no active simulation')
if __name__=='__main__':main()