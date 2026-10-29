"""Verify and archive the offline-only rolling analysis."""
import pathlib,json,hashlib,subprocess,shutil,shlex,xml.etree.ElementTree as ET,py_compile
root=pathlib.Path('/home/user/ros2_experiments/rolling_bypass_20261029')
local=pathlib.Path('/path/to/local-experiment-inputs/rolling_bypass_20261029')
source=root/'source'; tools=source/'experiment/rolling'
def cmd(args,cwd=None):return subprocess.check_output(args,cwd=cwd,text=True).strip()
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
for name in ['setup.py','build.sh','analyze.py','archive.py','README.md','PLAN.md','REPORT.md']:
    shutil.copy2(local/name,tools/name)
    shutil.copy2(local/name,root/name)
for name in ['setup.py','analyze.py','archive.py']:
    py_compile.compile(str(tools/name),doraise=True,cfile=str(root/'logs'/f'{name}.pyc'))
cmd(['bash','-n',str(tools/'build.sh')])
(root/'logs/final_ctest.log').write_text(cmd(['ctest','--test-dir',str(root/'build'),'--output-on-failure']))
(root/'logs/prior_geometry_test.log').write_text(cmd(['/home/user/ros2_experiments/simplified_bypass_20261029/probe_build/geometry_test']))
analysis=json.loads((root/'analysis.json').read_text())
assert len(analysis['trials'])==5
assert all(t['passed'] for t in analysis['trials'])
assert sum(t['cycles'] for t in analysis['trials'])==715
assert sum(t['sampled'] for t in analysis['trials'])==300240
assert not analysis['any_rear_entry_before_historical_release']
test=ET.parse(root/'logs/model.xml').getroot().attrib
assert test['tests']=='9' and test['failures']=='0'
(root/'test_summary.json').write_text(json.dumps({'model':test,'prior_geometry_rerun':'14/14 passed',
    'rolling_cycles':715,'sampled_candidates':300240,'all_fallback_and_geometry_checks_passed':True},indent=2))
before=json.loads((root/'protected_before.json').read_text())
changed=[p for p,d in before.items() if not pathlib.Path(p).is_file() or sha(pathlib.Path(p))!=d]
assert not changed,changed
(root/'preservation_after.json').write_text(json.dumps({'checked':len(before),'changed':changed},indent=2))
link=(root/'build/CMakeFiles/rolling.dir/link.txt').read_text()
libs={p:sha(pathlib.Path(p)) for p in shlex.split(link) if pathlib.Path(p).is_file() and ('.so' in p or p.endswith('.a'))}
(root/'version.json').write_text(json.dumps({'formal_main':cmd(['git','-C','/home/user/ros2_ws/src/my_dwa_controller','rev-parse','main']),
    'parent_experiment':'8285416c604cda2d8f16eba8745010387691bf1c',
    'offline_binary_sha256':{n:sha(root/'build'/n) for n in ['rolling','audit_selected','model_test']},
    'linked_libraries':libs,'link_command':link,'ldd':cmd(['ldd',str(root/'build/rolling')]),
    'runtime_controller_plugin_mapping':None,'gazebo_runs':0,
    'note':'Only offline executables run; no new actual Controller plugin identity.'},indent=2))
(root/'REVIEW.md').write_text('''独立只读审查 rolling_review：9项模型测试独立重跑通过。
独立复算715周期，dynamic_available和fallback_ok全真，selected_rank均0。
五组入区8.95/9.00/9.10/9.25/9.45；通过12.95/13.00/13.10/13.25/13.45。
最小近似实体净距0.0890899m，端点禁区违反0。
审查要求披露：clearance()的未来场景真值是候选准入而非仅事后监测，累计7171次拒绝；
ranked统计在最终队列后备检查之前，不能把全部备用负转候选称为最终可执行。
报告已披露。未发现推翻本轮条件模拟结果的直接实现缺陷，不支持真实闭环安全外推。
''')
runtime=cmd(['git','diff','f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446','--','src','include','test','config','CMakeLists.txt','package.xml'],cwd=source)
assert not runtime
plan=(root/'PLAN.md').read_text().replace('- [ ] 最终保护文件哈希、源码提交/二进制SHA和报告归档。','- [x] 最终保护文件哈希、源码提交/二进制SHA和报告归档。')
(root/'PLAN.md').write_text(plan);(tools/'PLAN.md').write_text(plan)
docs=source/'docs/experiments';docs.mkdir(exist_ok=True,parents=True)
shutil.copy2(root/'REPORT.md',docs/'rolling_bypass_20261029.md')
cmd(['git','add','experiment/rolling'],cwd=source)
cmd(['git','add','-f','docs/experiments/rolling_bypass_20261029.md'],cwd=source)
cmd(['git','diff','--cached','--check'],cwd=source)
print(cmd(['git','commit','-m','experiment: analyze multi-cycle DWA rear passage with original scoring'],cwd=source))
commit=cmd(['git','rev-parse','HEAD'],cwd=source)
assert not cmd(['git','status','--porcelain'],cwd=source)
cmd(['git','bundle','create',str(root/'rolling_bypass.bundle'),'--all'],cwd=source)
state={'status':'offline_analysis_complete','commit':commit,'branch':cmd(['git','branch','--show-current'],cwd=source),
       'parent':'8285416c604cda2d8f16eba8745010387691bf1c','runtime_controller_changed':False,
       'gazebo_runs':0,'merge_push_deploy':False,'protected_files':len(before),'protected_changes':changed,
       'geometry_channel_exists_at_all_starts':True,'conditional_rolling_passes':5,'trials':5,
       'earliest_selected_rear_nominal':8.4,'earliest_actual_rear_entry':8.95,'earliest_pass':12.95,
       'real_closed_loop_verified':False,'future_truth_geometry_gate':True,'worktree_clean':True}
(root/'FINAL_STATE.json').write_text(json.dumps(state,indent=2))
manifest={str(p.relative_to(root)):sha(p) for p in root.rglob('*') if p.is_file() and '.git' not in p.parts and p.name!='evidence_manifest.json'}
(root/'evidence_manifest.json').write_text(json.dumps(manifest,indent=2))
for name in ['REPORT.md','PLAN.md','README.md','input.json','analysis.json','selected_candidate_audit.json','test_summary.json',
             'version.json','FINAL_STATE.json','isolation.json','preservation_after.json','REVIEW.md','evidence_manifest.json',
             'rolling_timeseries.png','geometry_and_paths.png']:
    shutil.copy2(root/name,local/name)
print(json.dumps(state,indent=2))
