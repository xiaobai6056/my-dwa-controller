import pathlib,json,hashlib,subprocess,shutil,os,xml.etree.ElementTree as ET
os.environ['GIT_OPTIONAL_LOCKS']='0'
r=pathlib.Path('/home/user/ros2_experiments/polyline_bypass_20261029');src=r/'source';folder=src/'experiment/rolling'
local=pathlib.Path('/path/to/local-experiment-inputs/polyline_bypass_20261029');parent=r.parent/'reference_sweep_20261029/source'
formal=pathlib.Path('/home/user/ros2_ws/src/my_dwa_controller')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def git(p,*a):return subprocess.check_output(['git','-C',str(p),*a],text=True).strip()
def write(n,o):(r/n).write_text(json.dumps(o,indent=2)+'\n')
protected=json.loads((r/'protected_before.json').read_text());assert all(pathlib.Path(p).is_file() and sha(pathlib.Path(p))==h for p,h in protected.items())
assert git(formal,'rev-parse','main')=='f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446' and not git(formal,'status','--porcelain')
assert not git(src,'diff','f7b6a45','--','src','include','config','test')
old=(parent/'experiment/rolling/rolling.cpp').read_text();new=(folder/'rolling.cpp').read_text()
for a,b in [('      for (const auto & cmd:commands)','      const auto ordinary_ranked'),('      Velocity2D selected{}','      auto risk=yield.update'),('      if (!fallback_ok)','      if (entry<0')]:assert old[old.index(a):old.index(b)]==new[new.index(a):new.index(b)]
for n in ['rolling_model.cpp','rolling_model.hpp','strategy.hpp','replay_helpers.hpp']:assert (folder/n).read_bytes()==(parent/'experiment/rolling'/n).read_bytes()
tests=[]
for exe in ['model_test','test_strategy','test_reference','test_reference_tuning','test_polyline']:
 xml=r/f'logs/final_{exe}.xml'
 with (r/f'logs/final_{exe}.log').open('w') as f:subprocess.run([str(r/'build'/exe),f'--gtest_output=xml:{xml}'],stdout=f,stderr=subprocess.STDOUT,check=True)
 e=ET.parse(xml).getroot();assert int(e.attrib['failures'])==0;tests.append({'name':exe,'tests':int(e.attrib['tests']),'failures':0})
with (r/'logs/final_ctest.log').open('w') as f:subprocess.run(['ctest','--test-dir',str(r/'build'),'--output-on-failure'],stdout=f,stderr=subprocess.STDOUT,check=True)
assert sum(t['tests'] for t in tests)==41
allrows=[]
for d in (r/'results/phase1').iterdir():
 rows=[json.loads(x) for x in next(d.glob('start_*.jsonl')).read_text().splitlines()];allrows+=rows
 assert len(rows)==(64 if d.name=='E' else 240)
 for row in rows:
  assert abs(sum(row['weighted_score'].values())-row['score']['total'])<1e-12
  if 'abort' not in row:assert row['fallback_ok'] and row['executed_clearance']>0
 lines=[row['polyline'] for row in rows if row['polyline'] and row['reference']['phase']==1]
 if lines:assert all(x==lines[0] for x in lines)
 inp=json.loads((d/'input.json').read_text());inp.pop('reference_parameters');assert inp==json.loads((r/'input.json').read_text())
assert sum('abort' in x for x in allrows)==1 and len(allrows)==1504
assert (r/'stopped.json').exists() and not (r/'results/phase2').exists() and not (r/'results/upper').exists()
assert not json.loads((r/'analysis.json').read_text())['selected']
write('verification.json',{'tests':tests,'protected_files_unchanged':len(protected),'formal_main':git(formal,'rev-parse','main'),'formal_status':'clean','production_code_diff':'empty src/include/config/test','hard_gate_blocks_byte_identical':True,'A0_B0_reproduction':json.loads((r/'baseline_reproduction.json').read_text()),'complete_trials':6,'aborted_trials':1,'executed_cycles':1503,'abort_rows':1,'E_stop_time':7.15,'not_run':['F','G','H','12x','16x','6s','8s'],'zero_new_simulations_during_finalization':True})
for p in local.glob('*.py'):shutil.copy2(p,r/'scripts'/p.name)
shutil.copy2(local/'README.md',r/'README.md')
plan=(local/'PLAN.md').read_text().replace('- [ ]','- [x]')+'\n停止：E名义制动禁行失败；矩阵F/G/H及条件复核均未运行；不是完成八组。\n'
(r/'PLAN.md').write_text(plan);shutil.copy2(r/'PLAN.md',local/'PLAN.md')
archive=src/'experiment/polyline_bypass';archive.mkdir(exist_ok=True)
for p in (r/'scripts').glob('*.py'):shutil.copy2(p,archive/p.name)
for n in ['PLAN.md','REPORT.md','README.md','build.sh']:shutil.copy2(r/n,archive/n)
subprocess.run(['git','-C',str(src),'add','experiment/rolling','experiment/polyline_bypass'],check=True)
subprocess.run(['git','-C',str(src),'add','-f',*[str(archive/n) for n in ['PLAN.md','REPORT.md','README.md']]],check=True)
subprocess.run(['git','-C',str(src),'commit','-m','experiment: evaluate two-point polyline and stop on fallback exclusion failure'],check=True)
shutil.copy2(r/'build/rolling',r/'logs/rolling_executed')
link=(r/'build/CMakeFiles/rolling.dir/link.txt').read_text();(r/'logs/rolling_link.txt').write_text(link)
libs={t:sha(pathlib.Path(t)) for t in link.split() if t.startswith('/') and '/baseline_install/' in t and pathlib.Path(t).is_file()}
write('version_manifest.json',{'formal_main':git(formal,'rev-parse','main'),'parent':git(parent,'rev-parse','HEAD'),'branch':git(src,'branch','--show-current'),'commit':git(src,'rev-parse','HEAD'),'status':git(src,'status','--porcelain'),'executed_ELF_sha256':sha(r/'logs/rolling_executed'),'libraries':libs,'source_sha256':{n:sha(folder/n) for n in ['rolling.cpp','reference.cpp','reference.hpp','rolling_model.cpp','rolling_model.hpp','strategy.hpp']},'post_trial_changes':'only additional tests and read-only failure diagnostic; rolling execution source/ELF unchanged'})
subprocess.run(['git','-C',str(src),'bundle','create',str(r/'source.bundle'),'HEAD'],check=True);subprocess.run(['git','-C',str(src),'bundle','verify',str(r/'source.bundle')],check=True)
assert not git(src,'status','--porcelain')
assert all(sha(pathlib.Path(p))==h for p,h in protected.items())
write('final_state.json',{'decision':'stop after E fallback exclusion failure','qualified':False,'new_runtime_development':False,'matrix_complete':False,'gazebo':False,'formal_modified':False,'phase2':False,'upper_search':False})
manifest={str(p.relative_to(r)):sha(p) for p in r.rglob('*') if p.is_file() and not p.is_relative_to(src/'.git') and not p.is_relative_to(r/'build') and '__pycache__' not in p.parts and p.name!='evidence_manifest.json'};write('evidence_manifest.json',manifest)
assert all(sha(r/p)==h for p,h in manifest.items())
for n in ['REPORT.md','README.md','analysis.json','comparison.png','verification.json','version_manifest.json','final_state.json','evidence_manifest.json']:shutil.copy2(r/n,local/n)
print(json.dumps({'tests':41,'executed_cycles':1503,'protected_files':len(protected),'commit':git(src,'rev-parse','HEAD'),'manifest_files':len(manifest),'stopped_E':True}))
