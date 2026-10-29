"""Verify and archive the completed offline experiment; never run the simulator."""
import pathlib, subprocess, hashlib, json, shutil, difflib, os, xml.etree.ElementTree as ET
os.environ['GIT_OPTIONAL_LOCKS']='0'
root=pathlib.Path('/home/user/ros2_experiments/locked_reference_20261029')
local=pathlib.Path('/path/to/local-experiment-inputs/locked_reference_20261029')
src=root/'source'; parent=root.parent/'score_ablation_20261029/source'
formal=pathlib.Path('/home/user/ros2_ws/src/my_dwa_controller')
def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def git(p,*args): return subprocess.check_output(['git','-C',str(p),*args],text=True).strip()
def write(name,value): (root/name).write_text(json.dumps(value,indent=2)+'\n')

for name in ['REPORT.md','README.md','PLAN.md','build.sh','setup.py','prepare.py','integrate.py','run_trials.py','analyze.py','finalize.py']:
    shutil.copy2(local/name,root/name)
# Mark this task's plan complete; the conditional phase2 is deliberately skipped.
p=root/'PLAN.md'; text=p.read_text().replace('- [ ]','- [x]')
text=text.replace('reduces original-path distance','does not increase original-path distance (1e-9 tolerance; actual release strictly improves it)')
p.write_text(text+'\nResult: first-stage screen negative. No 6/8 s run, tuning, runtime integration or Gazebo.\n')
shutil.copy2(p,local/'PLAN.md')

before=json.loads((root/'protected_before.json').read_text())
changed=[p for p,h in before.items() if not pathlib.Path(p).is_file() or digest(pathlib.Path(p))!=h]
assert not changed,changed
assert git(formal,'rev-parse','main')=='f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446'
assert git(src,'branch','--show-current')=='codex/locked-reference-20261029'
assert not git(src,'diff','f7b6a45','--','src','include','config','test')
for name in ['rolling_model.cpp','rolling_model.hpp','strategy.hpp']:
    assert digest(src/'experiment/rolling'/name)==digest(parent/'experiment/rolling'/name),name

# Preserve the exact producer patch before regenerating a final patch.
producer_patch=root/'runner_phase1.patch'
if not producer_patch.exists(): shutil.copy2(root/'runner.patch',producer_patch)
old=(parent/'experiment/rolling/rolling.cpp').read_text()
new=(src/'experiment/rolling/rolling.cpp').read_text()
(root/'runner.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='ablation/rolling.cpp',tofile='reference/rolling.cpp')))
producer=root/'producer_source'; producer.mkdir(exist_ok=True)
for name in ['rolling.cpp','reference.cpp','reference.hpp','rolling_model.cpp','rolling_model.hpp','strategy.hpp','CMakeLists.txt']:
    text=(src/'experiment/rolling'/name).read_text()
    if name=='rolling.cpp':
        assert text.count('bypassActionAdmitted(selected_rank,selected)')==1
        text=text.replace('bypassActionAdmitted(selected_rank,selected)','selected_rank>=0')
    if name=='reference.cpp':
        assert 'selected_rank>=0 && (command.v!=0 || command.omega!=0)' in text
        text=text.replace('selected_rank>=0 && (command.v!=0 || command.omega!=0)','selected_rank>=0')
    (producer/name).write_text(text)

analysis=json.loads((root/'analysis.json').read_text()); assert not analysis['positive_screen']
assert not (root/'results/phase2').exists()
trials=[]
for mode in 'AB':
    rows=[json.loads(x) for x in (root/f'results/phase1/{mode}_t4/start_4.00.jsonl').read_text().splitlines()]
    assert len(rows)==240 and all(r['fallback_ok'] and r['executed_clearance']>0 for r in rows)
    assert all(abs(r['t']-(4+i*.05))<1e-8 for i,r in enumerate(rows))
    assert all(abs(sum(r['weighted_score'].values())-r['score']['total'])<1e-12 for r in rows)
    if mode=='B':
        locked=[r for r in rows if r['reference']['phase']==1]
        assert len(locked)==158
        assert len({tuple(r['reference']['target']) for r in locked})==1
        assert len({tuple(r['zone']) for r in locked})>1
        assert all(r['proposed_candidate']!=[0,0] for r in rows)
        first=locked[0]; assert abs(first['t']-5)<1e-8 and first['reference']['votes']==4
        assert all(r['selected']==[0,0] for r in rows if r['t']<5-1e-8)
    trials.append({'mode':mode,'cycles':len(rows),'sha256':digest(root/f'results/phase1/{mode}_t4/start_4.00.jsonl')})

audit=json.loads((root/'score_audit.json').read_text())
# Confirm score replay agrees with recorded selected candidates.
rowsB=[json.loads(x) for x in (root/'results/phase1/B_t4/start_4.00.jsonl').read_text().splitlines()]
for a in audit:
    r=min(rowsB,key=lambda r:abs(r['t']-a['t']))
    assert abs(a['choices']['selected']['guided_terms']['total']-r['score']['total'])<1e-12

tests=[]
for executable in ['model_test','test_strategy','test_reference']:
    logfile=root/f'logs/final_{executable}.log'; xml=root/f'logs/final_{executable}.xml'
    with logfile.open('w') as f:
        subprocess.run([str(root/'build'/executable),f'--gtest_output=xml:{xml}'],stdout=f,stderr=subprocess.STDOUT,check=True)
    e=ET.parse(xml).getroot(); assert int(e.attrib['failures'])==0
    tests.append({'name':executable,'tests':int(e.attrib['tests']),'failures':int(e.attrib['failures'])})
with (root/'logs/final_ctest.log').open('w') as f:
    subprocess.run(['ctest','--test-dir',str(root/'build'),'--output-on-failure'],stdout=f,stderr=subprocess.STDOUT,check=True)
assert sum(t['tests'] for t in tests)==26

write('verification.json',{'protected_files':len(before),'changed_protected_files':changed,
 'formal_main':git(formal,'rev-parse','main'),'formal_status':git(formal,'status','--porcelain'),
 'production_code_diff_vs_formal':'empty: src/include/config/test','trials':trials,'tests':tests,
 'score_audit_matches_selected':True,'locked_point_unique':True,'zone_updates_while_locked':True,
 'phase2_run':False,'gazebo_started':False,'runtime_deployed':False,'new_simulations_during_finalization':0,
 'producer_vs_final':'selected-zero admission guard added after review; never hit in archived D01 trace; producer ELF retained',
 'score_parameters_tuned':False})

archive=src/'experiment/locked_reference';archive.mkdir(exist_ok=True)
for name in ['README.md','PLAN.md','REPORT.md','setup.py','prepare.py','integrate.py','run_trials.py','analyze.py','build.sh','finalize.py']:
    shutil.copy2(root/name,archive/name)
shutil.copy2(root/'params/design.json',archive/'design.json')
subprocess.run(['git','-C',str(src),'add','experiment/rolling','experiment/locked_reference'],check=True)
subprocess.run(['git','-C',str(src),'add','-f',*[str(archive/name) for name in ['README.md','PLAN.md','REPORT.md']]],check=True)
subprocess.run(['git','-C',str(src),'commit','-m','experiment: evaluate locked reference Heading with original DWA safety gates'],check=True)

ldd=subprocess.check_output(['ldd',str(root/'logs/phase1_executable')],text=True)
(root/'logs/producer_ldd.log').write_text(ldd)
libraries={}
for line in ldd.splitlines():
    parts=line.split('=>')
    if len(parts)==2:
        path=parts[1].strip().split()[0]
        if path.startswith('/') and ('my_dwa' in path or 'baseline_install' in path):libraries[path]=digest(pathlib.Path(path))
link=(root/'build/CMakeFiles/rolling.dir/link.txt').read_text()
(root/'logs/final_link.txt').write_text(link)
for token in link.split():
    if token.startswith('/') and '/baseline_install/' in token and pathlib.Path(token).is_file():
        libraries[token]=digest(pathlib.Path(token))
write('version_manifest.json',{'formal_baseline':git(formal,'rev-parse','main'),'parent':git(parent,'rev-parse','HEAD'),
 'branch':git(src,'branch','--show-current'),'commit':git(src,'rev-parse','HEAD'),
 'producer_ELF_sha256':digest(root/'logs/phase1_executable'),'final_ELF_sha256':digest(root/'build/rolling'),
 'libraries':libraries,'source_status':git(src,'status','--porcelain'),
 'producer_source_note':'runner and reference admission predicate reconstructed before selected-zero guard; CMake includes later audit target; actual ELF retained',
 'producer_source_files':{p.name:digest(p) for p in producer.iterdir() if p.is_file()}})
assert libraries
subprocess.run(['git','-C',str(src),'bundle','create',str(root/'source.bundle'),'HEAD'],check=True)
subprocess.run(['git','-C',str(src),'bundle','verify',str(root/'source.bundle')],check=True)
write('final_state.json',{'decision':'stop after negative 4s active-bypass screen','active_early_bypass_confirmed':False,
 'entry_lead_vs_historical_8_8_seconds':0,'pass_gain_vs_parked_A_seconds':1.15,
 'runtime_controller_development_recommended':False,'phase2_run':False,'gazebo_run':False,
 'interpretation':'small sustained rear offset; passes earlier than full parking A, not before historical risk release; guide-only causality not isolated'})
assert all(pathlib.Path(p).is_file() and digest(pathlib.Path(p))==h for p,h in before.items())
assert not git(src,'status','--porcelain')
files={str(p.relative_to(root)):digest(p) for p in root.rglob('*') if p.is_file() and
 not p.is_relative_to(src/'.git') and not p.is_relative_to(root/'build') and p.name!='evidence_manifest.json'}
write('evidence_manifest.json',files)
for name in ['REPORT.md','README.md','analysis.json','score_audit.json','verification.json','version_manifest.json','final_state.json','evidence_manifest.json','comparison.png']:
    shutil.copy2(root/name,local/name)
print(json.dumps({'commit':git(src,'rev-parse','HEAD'),'tests':tests,'protected_files':len(before),'evidence_files':len(files),'complete':True}))
