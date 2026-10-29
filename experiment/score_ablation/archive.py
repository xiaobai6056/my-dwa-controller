"""Archive the negative ablation result without touching original experiments."""
import pathlib,json,hashlib,shutil,subprocess,shlex,xml.etree.ElementTree as ET,py_compile
root=pathlib.Path('/home/user/ros2_experiments/score_ablation_20261029')
local=pathlib.Path('/path/to/local-experiment-inputs/score_ablation_20261029')
base=root.parent/'rolling_bypass_20261029';source=root/'source'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def cmd(args,cwd=None):return subprocess.check_output(args,cwd=cwd,text=True).strip()
tools=source/'experiment/score_ablation';tools.mkdir(exist_ok=True)
for name in ['setup.py','prepare.py','build.sh','run_trials.py','analyze.py','archive.py','PLAN.md','REPORT.md','README.md']:
    shutil.copy2(local/name,root/name);shutil.copy2(local/name,tools/name)
for name in ['setup.py','prepare.py','run_trials.py','analyze.py','archive.py']:
    py_compile.compile(str(root/name),doraise=True,cfile=str(root/'logs'/f'{name}.pyc'))
cmd(['bash','-n',str(root/'build.sh')])
(root/'logs/final_ctest.log').write_text(cmd(['ctest','--test-dir',str(root/'build'),'--output-on-failure']))
for name in ['model_test','test_strategy']:
    (root/'logs'/f'{name}.log').write_text(cmd([str(root/'build'/name),'--gtest_output=xml:'+str(root/'logs'/f'{name}.xml')]))
assert ET.parse(root/'logs/model_test.xml').getroot().attrib['failures']=='0'
assert ET.parse(root/'logs/test_strategy.xml').getroot().attrib['failures']=='0'
analysis=json.loads((root/'analysis.json').read_text());assert not analysis['shortlist']
assert not (root/'results/phase2').exists()
cycles=0;weighted_error=0;min_goal_distance=1e100
for scheme in 'ABCDEFG':
    folder=root/'results/phase1'/f'{scheme}_t4'
    rows=list(map(json.loads,(folder/'start_4.00.jsonl').read_text().splitlines()))
    params=json.loads((folder/'parameters.json').read_text());w=params['weights']
    assert len(rows)==200 and abs(rows[-1]['t']+.05-14)<1e-8
    for r in rows:
        assert r['scheme']==scheme and r['fallback_ok'] and r['dynamic_available'] and r['executed_clearance']>.05
        expected={'heading':w['H']*r['score']['heading'],'path':w['P']*r['score']['path'],
            'goal':w['G']*r['score']['goal'],'speed':w['V']*r['score']['speed'],'obstacle':w['O']*r['score']['obstacle']}
        assert expected==r['weighted_score']
        weighted_error=max(weighted_error,abs(sum(expected.values())-r['score']['total']))
        min_goal_distance=min(min_goal_distance,((r['pose'][0]-4)**2+r['pose'][1]**2)**.5)
    cycles+=len(rows)
assert weighted_error<1e-12 and min_goal_distance>.25
parity=json.loads((root/'results/phase1/A_t4/baseline_parity.json').read_text())
assert not parity['mismatches'] and parity['original_prefix_cycles']==179
for names in ['CD','CE']:
    assert analysis['sequence_comparisons'][names]['command_mismatches']==0
    assert analysis['sequence_comparisons'][names]['pose_mismatches']==0
unchanged_helpers=['rolling_model.cpp','rolling_model.hpp','model_test.cpp']
for n in unchanged_helpers:
    assert sha(source/'experiment/rolling'/n)==sha(base/'source/experiment/rolling'/n)
assert sha(source/'experiment/probe_geometry.hpp')==sha(base/'source/experiment/probe_geometry.hpp')
runtime_diff=cmd(['git','diff','f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446','--',
                 'src','include','test','config','CMakeLists.txt','package.xml'],cwd=source)
assert not runtime_diff
before=json.loads((root/'protected_before.json').read_text())
changed=[p for p,d in before.items() if not pathlib.Path(p).is_file() or sha(pathlib.Path(p))!=d]
assert not changed,changed
(root/'preservation_after.json').write_text(json.dumps({'checked':len(before),'changed':changed},indent=2))
verification={'model_tests':9,'weight_tests':3,'ctest':'2/2 passed','red_weight_tests':'3/3 failed before mapping implementation',
 'baseline_original_prefix_cycles':179,'baseline_field_mismatches':0,'fixed_end_time':14,'total_cycles':cycles,
 'maximum_weighted_sum_error':weighted_error,'minimum_distance_to_final_goal':min_goal_distance,
 'original_helpers_identical':unchanged_helpers+['probe_geometry.hpp'],'production_diff':runtime_diff,
 'phase2_run':False,'reason':'No meaningful rearward displacement or earlier passage in phase one.'}
(root/'verification.json').write_text(json.dumps(verification,indent=2))
link=(root/'build/CMakeFiles/rolling.dir/link.txt').read_text()
libraries={p:sha(pathlib.Path(p)) for p in shlex.split(link) if pathlib.Path(p).is_file() and (p.endswith('.a') or '.so' in p)}
(root/'version.json').write_text(json.dumps({'formal_main':'f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446',
    'parent':'5cd10fc0a0e00c9b81b9296a936ef362b7b331e8','binary_sha256':sha(root/'build/rolling'),
    'linked_libraries_sha256':libraries,'link_command':link,'runtime_controller_plugin_mapping':None,'gazebo_runs':0},indent=2))
(root/'REVIEW.md').write_text('''独立只读审查 ablation_review：未发现阻断性缺陷。
复核全部1400周期；Heading flag维持true、权重无归一化，原门禁/helper/采样/时序不变。
A的179周期原字段精确复现，C/D/E200周期动作位姿完全一致，42个关键周期与原日志吻合。
所有组完整运行至14s；G正向横移未被误算为后侧，24次queue_backup只是候选拒绝。
支持无合格方案、保留A、停止第二阶段。限制：仅t4/A-G/完整场景及未来真值条件模型，
未做t6/t8不得声称跨启动一致，G二维不停滞不等于有效前进。
''')
plan=(root/'PLAN.md').read_text().replace('- [ ]','- [x]')
plan+='\nResult: no scheme qualified; the conditional t=6/8 step was deliberately skipped, not claimed completed as trials.\n'
(root/'PLAN.md').write_text(plan);(tools/'PLAN.md').write_text(plan)
docs=source/'docs/experiments';docs.mkdir(parents=True,exist_ok=True)
shutil.copy2(root/'REPORT.md',docs/'score_ablation_20261029.md')
cmd(['git','add','experiment/rolling/rolling.cpp','experiment/rolling/CMakeLists.txt',
     'experiment/rolling/strategy.hpp','experiment/rolling/test_strategy.cpp','experiment/score_ablation'],cwd=source)
cmd(['git','add','-f','docs/experiments/score_ablation_20261029.md',
     'experiment/score_ablation/PLAN.md','experiment/score_ablation/README.md','experiment/score_ablation/REPORT.md'],cwd=source)
cmd(['git','diff','--cached','--check'],cwd=source)
print(cmd(['git','commit','-m','experiment: evaluate fixed-window A-G dynamic bypass score ablations'],cwd=source))
commit=cmd(['git','rev-parse','HEAD'],cwd=source);assert not cmd(['git','status','--porcelain'],cwd=source)
cmd(['git','bundle','create',str(root/'score_ablation.bundle'),'--all'],cwd=source)
state={'status':'offline_ablation_complete_negative_result','commit':commit,
 'branch':cmd(['git','branch','--show-current'],cwd=source),'phase1_schemes':list('ABCDEFG'),'start_times':[4],
 'phase2_run':False,'preferred':None,'backup':None,'retain_baseline':'A','qualified':[],
 'cycles':1400,'end_time':14,'protected_files':len(before),'protected_changes':changed,
 'production_changes':False,'four_scan_unlock_implemented':False,'release_logic_changes':False,
 'gazebo_runs':0,'merge_push_deploy':False,'worktree_clean':True}
(root/'FINAL_STATE.json').write_text(json.dumps(state,indent=2))
manifest={str(p.relative_to(root)):sha(p) for p in root.rglob('*') if p.is_file() and '.git' not in p.parts and p.name!='evidence_manifest.json'}
(root/'evidence_manifest.json').write_text(json.dumps(manifest,indent=2))
for n in ['REPORT.md','PLAN.md','README.md','analysis.json','comparison.csv','comparison.png','key_cycles.json',
          'verification.json','version.json','REVIEW.md','preservation_after.json','FINAL_STATE.json','evidence_manifest.json']:
    shutil.copy2(root/n,local/n)
print(json.dumps(state,indent=2))
