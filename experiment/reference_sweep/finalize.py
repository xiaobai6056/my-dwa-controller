"""Archive and verify only; this file never invokes the rolling simulator."""
import pathlib,subprocess,hashlib,json,shutil,os,xml.etree.ElementTree as ET
from common import ROOT,rows
os.environ['GIT_OPTIONAL_LOCKS']='0'
local=pathlib.Path('/path/to/local-experiment-inputs/reference_sweep_20261029')
src=ROOT/'source';folder=src/'experiment/rolling'
parent=ROOT.parent/'locked_reference_20261029/source'
formal=pathlib.Path('/home/user/ros2_ws/src/my_dwa_controller')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def git(p,*args):return subprocess.check_output(['git','-C',str(p),*args],text=True).strip()
def write(name,v):(ROOT/name).write_text(json.dumps(v,indent=2)+'\n')
protected=json.loads((ROOT/'protected_before.json').read_text())
changed=[p for p,h in protected.items() if not pathlib.Path(p).is_file() or sha(pathlib.Path(p))!=h]
assert not changed,changed
assert git(formal,'rev-parse','main')=='f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446'
assert not git(formal,'status','--porcelain')
assert not git(src,'diff','f7b6a45','--','src','include','config','test')
assert git(src,'branch','--show-current')=='codex/reference-sweep-20261029'
for name in ['rolling_model.cpp','rolling_model.hpp','strategy.hpp','replay_helpers.hpp']:
    assert (folder/name).read_bytes()==(parent/'experiment/rolling'/name).read_bytes(),name
old=(parent/'experiment/rolling/rolling.cpp').read_text();new=(folder/'rolling.cpp').read_text()
for a,b in [('      for (const auto & cmd:commands)','      const auto ordinary_ranked'),
            ('      Velocity2D selected{}','      auto risk=yield.update'),
            ('      if (!fallback_ok)','      if (entry<0')]:
    assert old[old.index(a):old.index(b)]==new[new.index(a):new.index(b)],a

analysis=json.loads((ROOT/'analysis.json').read_text());assert analysis['matrix_complete']
assert analysis['selected_for_confirmation'] is None and not (ROOT/'results/phase2').exists()
repro=json.loads((ROOT/'baseline_reproduction.json').read_text())
assert all(x['all_inherited_fields_equal'] for x in repro.values())
trials=[]
original=json.loads((ROOT/'input.json').read_text())
for d in sorted((ROOT/'results/phase1').iterdir()):
    records=rows(d);assert len(records)==240
    inp=json.loads((d/'input.json').read_text());settings=inp.pop('reference_parameters');assert inp==original
    assert all(abs(r['t']-(4+i*.05))<1e-8 for i,r in enumerate(records))
    assert all(r['fallback_ok'] and r['executed_clearance']>0 and 'abort' not in r for r in records)
    for r in records:
        expected=.05*(settings['heading_multiplier'] if r['guided_scoring'] else 1)
        assert r['effective_heading_weight']==expected
        assert r['score_is_held_proposal']==r['command_held']
        if r['score'] is not None:
            for k,w in [('heading',expected),('goal',.6),('path',.2),('speed',.1),('obstacle',.1)]:
                assert abs(r['weighted_score'][k]-w*r['score'][k])<1e-12
            assert abs(sum(r['weighted_score'].values())-r['score']['total'])<1e-12
    locked=[r for r in records if r['reference']['phase']==1]
    if locked:
        assert len({tuple(r['reference']['target']) for r in locked})==1
        assert len({tuple(r['zone']) for r in locked})>1
        first=locked[0];assert first['reference']['votes']==4
        target=first['reference']['target'];assert abs(target[0]-first['zone'][1]-settings['forward_offset'])<1e-12
        assert abs(first['zone'][2]-target[1]-settings['extra_clearance'])<1e-12
        preceding=[r for r in records if r['t']<=first['t']+1e-8 and r['reference']['votes']>0]
        assert len({r['scan_t'] for r in preceding})==4
    if (d/'score_audit.json').exists():
        for item in json.loads((d/'score_audit.json').read_text()):
            r=min(records,key=lambda r:abs(r['t']-item['t']))
            assert abs(item['choices']['selected_proposal']['guided_terms']['total']-r['score']['total'])<1e-12
            assert item['choices']['selected_proposal']['admitted']
    trials.append({'name':d.name,'cycles':len(records),'trace_sha256':sha(next(d.glob('start_*.jsonl'))),'input_sha256':sha(d/'input.json')})
assert len(trials)==13
tests=[]
for exe in ['model_test','test_strategy','test_reference','test_reference_tuning']:
    xml=ROOT/f'logs/final_{exe}.xml'
    with (ROOT/f'logs/final_{exe}.log').open('w') as f:
        subprocess.run([str(ROOT/'build'/exe),f'--gtest_output=xml:{xml}'],stdout=f,stderr=subprocess.STDOUT,check=True)
    e=ET.parse(xml).getroot();assert int(e.attrib['failures'])==0
    tests.append({'name':exe,'tests':int(e.attrib['tests']),'failures':int(e.attrib['failures'])})
with (ROOT/'logs/final_ctest.log').open('w') as f:
    subprocess.run(['ctest','--test-dir',str(ROOT/'build'),'--output-on-failure'],stdout=f,stderr=subprocess.STDOUT,check=True)
assert sum(t['tests'] for t in tests)==31
write('verification.json',{'tests':tests,'protected_files':len(protected),'changed_protected_files':changed,
 'formal_main':git(formal,'rev-parse','main'),'formal_status':git(formal,'status','--porcelain'),
 'production_code_diff_vs_formal':'empty: src/include/config/test','safety_blocks_byte_identical':True,
 'original_A_B_480_cycles_exact':True,'trial_count':13,'total_cycles':3120,'trials':trials,
 'parameters_applied_and_weighted_scores_match':True,'four_distinct_source_scan_admission':True,
 'locked_point_fixed_and_zone_updates':True,'score_audit_matches_selected':True,
 'no_phase2':True,'gazebo_run':False,'runtime_modified':False,'new_simulations_during_finalization':0})

for name in ['README.md','build.sh']:shutil.copy2(local/name,ROOT/name)
for p in local.glob('*.py'):shutil.copy2(p,ROOT/'scripts'/p.name)
plan=(local/'PLAN.md').read_text().replace('- [ ]','- [x]')
plan+='\nCompleted: fixed 12 B + A matrix; conditional 6/8 checks skipped because none eligible. No tuning after results.\n'
(ROOT/'PLAN.md').write_text(plan);shutil.copy2(ROOT/'PLAN.md',local/'PLAN.md')
archive=src/'experiment/reference_sweep'
for p in (ROOT/'scripts').glob('*.py'):shutil.copy2(p,archive/p.name)
for name in ['README.md','REPORT.md','PLAN.md','build.sh']:shutil.copy2(ROOT/name,archive/name)
shutil.copy2(ROOT/'params/matrix.json',archive/'matrix.json')
subprocess.run(['git','-C',str(src),'add','experiment/rolling','experiment/reference_sweep'],check=True)
subprocess.run(['git','-C',str(src),'add','-f',*[str(archive/n) for n in ['README.md','REPORT.md','PLAN.md']]],check=True)
subprocess.run(['git','-C',str(src),'commit','-m','experiment: sweep strong reference Heading and conservative point margins'],check=True)
shutil.copy2(ROOT/'build/rolling',ROOT/'logs/rolling_executed')
link=(ROOT/'build/CMakeFiles/rolling.dir/link.txt').read_text();(ROOT/'logs/rolling_link.txt').write_text(link)
ldd=subprocess.check_output(['ldd',str(ROOT/'logs/rolling_executed')],text=True);(ROOT/'logs/rolling_ldd.log').write_text(ldd)
libraries={}
for token in link.split():
    p=pathlib.Path(token)
    if token.startswith('/') and '/baseline_install/' in token and p.is_file():libraries[token]=sha(p)
assert len(libraries)>=8
write('version_manifest.json',{'formal_main':git(formal,'rev-parse','main'),'parent':git(parent,'rev-parse','HEAD'),
 'branch':git(src,'branch','--show-current'),'commit':git(src,'rev-parse','HEAD'),'source_status':git(src,'status','--porcelain'),
 'all_13_trials_executable_sha256':sha(ROOT/'logs/rolling_executed'),'current_executable_sha256':sha(ROOT/'build/rolling'),
 'linked_baseline_libraries':libraries,'runner_source_sha256':{n:sha(folder/n) for n in ['rolling.cpp','reference.cpp','reference.hpp','rolling_model.cpp','rolling_model.hpp','strategy.hpp']},
 'diagnostic_only_audit_source_changed_after_trials':True,'simulation_runner_unchanged_during_sweep':True})
subprocess.run(['git','-C',str(src),'bundle','create',str(ROOT/'source.bundle'),'HEAD'],check=True)
subprocess.run(['git','-C',str(src),'bundle','verify',str(ROOT/'source.bundle')],check=True)
assert not git(src,'status','--porcelain')
assert all(pathlib.Path(p).is_file() and sha(pathlib.Path(p))==h for p,h in protected.items())
write('final_state.json',{'result':'No qualifying improvement; stronger Heading makes early rear displacement larger but degrades full passage/recovery',
 'preferred_new_runtime_parameter':None,'conservative_offline_alternative':{'heading_multiplier':1,'paired_margin':.2},
 'phase2':False,'runtime_recommended':False,'gazebo':False,'next_search_expanded':False})
manifest={str(p.relative_to(ROOT)):sha(p) for p in ROOT.rglob('*') if p.is_file() and
 not p.is_relative_to(src/'.git') and not p.is_relative_to(ROOT/'build') and '__pycache__' not in p.parts and p.name!='evidence_manifest.json'}
write('evidence_manifest.json',manifest)
for name in ['README.md','REPORT.md','analysis.json','verification.json','version_manifest.json','final_state.json','evidence_manifest.json','comparison.png']:
    shutil.copy2(ROOT/name,local/name)
print(json.dumps({'tests':31,'trials':13,'cycles':3120,'protected_files':len(protected),'commit':git(src,'rev-parse','HEAD'),'evidence_files':len(manifest),'no_phase2':True}))
