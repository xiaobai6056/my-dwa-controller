#!/usr/bin/env python3
"""Read-only historical baseline inventory; writes only its two named output files."""
from pathlib import Path


import collections, datetime, gzip, hashlib, json, os, re, subprocess

# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))
ARCHIVE_REPO = Path(__file__).resolve().parents[2]

ROOT=ARCHIVE_REPO
OUT=ROOT/'benchmark/baseline_diagnosis_20261021/outputs'
def rel(p):
    try:return str(Path(p).relative_to(ROOT))
    except ValueError:return str(p)
def read(p):
    try:return json.loads(Path(p).read_text())
    except (OSError,ValueError):return {}
def sha(p):
    p=Path(p)
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
def command(args):
    r=subprocess.run(args,cwd=ROOT,capture_output=True,text=True)
    return {'command':args,'returncode':r.returncode,'stdout':r.stdout.strip(),'stderr':r.stderr.strip()}
def flatten(obj,prefix=''):
    out={}
    for k,v in obj.items():
        name=f'{prefix}.{k}' if prefix else k
        if isinstance(v,dict):out.update(flatten(v,name))
        else:out[name]=v
    return out
def diff(a,b):
    a,b=flatten(a),flatten(b)
    return {k:{'reference':a.get(k),'observed':b.get(k)} for k in sorted(set(a)|set(b)) if a.get(k)!=b.get(k)}
ref_path=ROOT/'benchmark/results/v1/v1_1_S04_001/mydwa/S04/trial_001/summary.json'
ref=read(ref_path).get('runtime_parameters',{})
records=[]
for q in sorted((ROOT/'benchmark').rglob('summary.json')):
    if 'baseline_diagnosis_20261021' in q.parts:continue
    j=read(q); a=j.get('arguments',{}); scene=j.get('scene') or q.parent.parent.name
    ctrl=j.get('controller') or a.get('controller')
    if scene not in ('S04','S05','D01','D02') or ctrl not in ('mydwa','dwb'):continue
    path=rel(q.parent)
    generation=('static_v1_1' if '/v1_1_' in path else 'static_original_or_other' if scene.startswith('S') else 'D01_release_timing_sweep' if '/timing_sweep/' in path else 'D01_candidate_probe' if 'diagnostic_t' in path and 'linked' in path else 'dynamic_historical_other')
    files={x.name:{'bytes':x.stat().st_size} for x in sorted(q.parent.iterdir()) if x.is_file()}
    par=j.get('runtime_parameters',{})
    m=read(q.parent/'metrics.json')
    records.append({'path':path,'scene':scene,'controller':ctrl,'generation':generation,'valid':j.get('valid'),'status':j.get('status'),'action_result':j.get('action_result'),'stop_reason':j.get('stop_reason'),'error':j.get('error') or j.get('action_error') or j.get('motion_error'),'navigation_start_sim_s':j.get('navigation_start_sim_s'),'navigation_end_sim_s':j.get('navigation_end_sim_s'),'recoveries':j.get('max_feedback_recoveries'),'library_sha256_stored':j.get('selected_plugin_sha256') or j.get('mydwa_library_sha256'),'underlay_sha256_stored':j.get('original_underlay_mydwa_sha256'),'runtime_controller_parameters_equal_v1_1':par.get('controller_server')==ref.get('controller_server') if par else None,'runtime_controller_differences':diff(ref.get('controller_server',{}),par.get('controller_server',{})) if par else None,'runtime_all_parameter_differences':diff(ref,par) if par else None,'files':files,'metrics':m,'summary_sha256':sha(q),'effective_start_time_s':j.get('effective_start_time_s')})
# Every formal repeat remains listed; representative selection never filters by outcome.
sweep=read(ROOT/'benchmark/dynamic/timing_sweep/outputs/results.json')
formal=sweep.get('records',[])
selection={}
for scene in ('S04','S05'):
    candidates=sorted(r['path'] for r in records if r['scene']==scene and r['controller']=='mydwa' and r['generation']=='static_v1_1')
    selection[scene]=candidates[0] if candidates else None
for t in (2,3,4,5,6):
    candidates=sorted([r for r in formal if r['controller']=='mydwa' and r['start_time_s']==t],key=lambda x:(x['repeat'],x['order'],x['run']))
    selection[f'D01_t{t}']='benchmark/dynamic/timing_sweep/runs/'+candidates[0]['run']+'/trial' if candidates else None
selection['D02']='benchmark/results/dynamic/formal_v0_1_20261020/D02/mydwa/trial_001'
# Record all run manifests, including infrastructure failures and abandoned diagnostics.
manifest_records=[]
for q in sorted((ROOT/'benchmark/dynamic').rglob('manifest.json')):
    j=read(q)
    if 'arguments' not in j:continue
    manifest_records.append({'path':rel(q),'manifest':j,'trial_summary_present':(q.parent/'trial/summary.json').exists(),'candidate_files':[rel(x) for x in q.parent.glob('decisions_*.jsonl')],'infrastructure_failure':bool(j.get('error') or j.get('cleanup_error') or j.get('cleanup_returncode',0) or (not (q.parent/'trial/summary.json').exists() and j.get('runner_returncode',0)))})
missing_run_dirs=[]
for base in ('benchmark/dynamic/avoidability/runs','benchmark/dynamic/diagnostics/runs','benchmark/dynamic/timing_sweep/runs'):
    for q in sorted((ROOT/base).iterdir()):
        if q.is_dir() and not (q/'manifest.json').exists():missing_run_dirs.append({'path':rel(q),'files':[x.name for x in sorted(q.iterdir()) if x.is_file()],'note':'No manifest; not counted as a validated formal result.'})
before=read(ROOT/'benchmark/dynamic/diagnostics/release_comparison/evidence/before.json')
source_comparison={k:{'recorded':v,'current':sha(ROOT/k),'equal':v==sha(ROOT/k)} for k,v in before.get('source',{}).items()}
identity=read(ROOT/'benchmark/dynamic/diagnostics/release_comparison/evidence/build_identity.json')
libs={str(q):sha(q) for q in [ARCHIVE_WS/'install/my_dwa_controller/lib/libmy_dwa_controller.so',ARCHIVE_WS/'install_compile_comparison_20261020/release/lib/libmy_dwa_controller.so',ROOT/'benchmark/v1_1/evidence/libmy_dwa_controller.so.tested',ROOT/'benchmark/v1_1/evidence/original/libmy_dwa_controller.so.snapshot',ROOT/'benchmark/dynamic/avoidability/probe_build/libdecision.so']}
probes=[]
for run in ('diagnostic_t2_linked','diagnostic_t4_linked'):
    for q in sorted((ROOT/'benchmark/dynamic/avoidability/runs'/run).glob('decisions_*.jsonl')):
        n=feasible=allc=0; keys=set(); candidate_keys=set(); points=collections.Counter(); scored=0
        with q.open() as f:
            for line in f:
                if not line.strip():continue
                j=json.loads(line);n+=1;keys.update(j);points[len(j.get('best_poses',[]))]+=1
                for c in j.get('candidates',[]):
                    allc+=1;feasible+=bool(c.get('free'));scored+=c.get('scores') is not None;candidate_keys.update(c)
        probes.append({'run':run,'file':rel(q),'sha256':sha(q),'cycles':n,'evaluated_candidates':allc,'feasible_candidates':feasible,'scored_candidates':scored,'cycle_keys':sorted(keys),'candidate_keys':sorted(candidate_keys),'selected_trajectory_point_counts':dict(points)})
# Verify bag absence over the entire ros2_ws; do not follow symlinked directories.
bags=[]
for folder,dirs,files in os.walk(ARCHIVE_WS,followlinks=False):
    for name in files:
        if name.endswith(('.db3','.mcap')) or name=='metadata.yaml':bags.append(str(Path(folder)/name))
packages=command(['dpkg-query','-W','-f=${Package}\t${Version}\n','ros-humble-nav2-controller','ros-humble-nav2-bringup','ros-humble-nav2-dwb-controller','gazebo'])
result={'generated_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'Read-only inventory. No build, simulation, rosbag replay, git mutation or modification of existing project files.','selection_rule':'First formal repeat in each explicitly named source generation, regardless of success/collision. S04/S05 use v1.1 generation; D01 uses timing_sweep first tag; D02 formal_v0_1. All repeats/infrastructure records remain below. Candidate-probe linked runs are separate diagnostics, not formal repeats.','representative_trials':selection,'trial_records':records,'timing_sweep_complete':sweep.get('complete'),'timing_sweep_records':formal,'run_manifests':manifest_records,'infrastructure_failure_files':[{'path':rel(q),'record':read(q)} for q in sorted((ROOT/'benchmark').rglob('infrastructure_failure.json')) if 'baseline_diagnosis_20261021' not in q.parts],'unmanifested_run_directories':missing_run_dirs,'git_head':command(['git','rev-parse','HEAD']),'git_status':command(['git','status','--short']),'release_comparison_build_identity':identity,'release_comparison_before_head':before.get('head'),'release_comparison_source_hash_comparison':source_comparison,'libraries_current_sha256':libs,'v1_1_frozen_controller_sha256':sha(ROOT/'benchmark/v1_1/evidence/controller.cpp.tested.snapshot'),'current_controller_sha256':sha(ROOT/'src/controller.cpp'),'v1_1_original_controller_sha256':sha(ROOT/'benchmark/v1_1/evidence/original/controller.cpp.snapshot'),'current_vs_v1_1_controller_bytes_equal':sha(ROOT/'src/controller.cpp')==sha(ROOT/'benchmark/v1_1/evidence/controller.cpp.tested.snapshot'),'packages':packages,'frozen_packages':(ROOT/'benchmark/v1_1/evidence/package_versions.txt').read_text(),'candidate_probe_inventory':probes,'existing_candidate_decision_summary':read(ROOT/'benchmark/dynamic/avoidability/outputs/decision_summary.json'),'rosbag_search':{'root':str(ARCHIVE_WS),'follow_symlinks':False,'patterns':['*.db3','*.mcap','metadata.yaml'],'found':bags},'evidence_limits':['Hash equality establishes file identity only; differing binaries or source generations do not establish behavioral equivalence.','Historical summary-stored default library hashes do not prove actual process mapping. Timing-sweep manifest/plugin_maps provides stronger mapping evidence.','No native TopK, all-candidate poses or per-cell histogram in existing probe; predictor/map based reconstruction is required.','Static S04/S05 v1.1 lack sensors.jsonl.gz and ground_truth.jsonl; historical localization and plans cannot replace raw map/candidate evidence.','Completed schedules alone are not evidence: this inventory requires present summaries/manifests and retains errors.']}
OUT.mkdir(exist_ok=True)
(OUT/'inventory.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
lines=['# 历史基线与诊断能力清单','','本次仅枚举、读取和散列既有证据；没有启动仿真、重放、构建或修改既有文件。完整逐 trial、参数差异、manifest 与失败记录见 `inventory.json`。','','## 固定选择规则','','按明确的源码/库代际选择首个正式 repeat，不按成功与否筛选；所有 repeat 和基础设施失败仍保留。S04/S05 采用 v1.1，D01 采用 timing_sweep/first，D02 采用 formal_v0_1。linked 候选探针属于额外诊断，不能冒充正式 repeat。','','| 条件 | 代表 trial |','|---|---|']
for k,v in selection.items():lines.append(f'| {k} | `{v}` |')
lines+=['','## 已有覆盖','','| 条件 | 本代正式 MyDWA repeat 数 | 原始局部地图/GT | 每周期候选细节 |','|---|---:|---|---|','| S04 v1.1 | 5 | 无（有 trajectory、localization、plans） | 无 |','| S05 v1.1 | 2 | 无（有 trajectory、localization、plans） | 无 |']
for t in (2,3,4,5,6):
    rr=[r for r in formal if r['controller']=='mydwa' and r['start_time_s']==t]
    outcomes=collections.Counter(r.get('metrics',{}).get('termination') for r in rr)
    lines.append(f'| D01 t{t} | {len(rr)}，结局 {dict(outcomes)} | 有 sensors.jsonl.gz / ground_truth / localization / plans | '+('另有 linked 探针，非正式 repeat' if t in (2,4) else '无')+' |')
lines+=['| D02 formal_v0_1 | 3 | 有 sensors.jsonl.gz / ground_truth / localization / plans | 无 |','','timing_sweep/results.json 实际有 '+str(len(formal))+' 条记录（MyDWA '+str(sum(r['controller']=='mydwa' for r in formal))+' 条），complete='+str(sweep.get('complete'))+'。旧 static_v1、stage_a_remaining、tuning_r1 等另列在 JSON，不与 v1.1 混算。基础设施失败如 `avoidability/runs/validation_t2_early` 的 Gazebo readiness timeout 被保留；`diagnostic_t2` 的 infrastructure_failure.json 记录 ELF 探针缺少 nav2_costmap_2d_core 链接，首次 compute 时 component_container exited127、无 decision；其碰撞结局 unknown，不能当已完成结果。','','## 身份与可比性','','- Git HEAD：`'+result['git_head']['stdout']+'`；既有 `.gitignore`、`controller.hpp`、`controller.cpp` 修改及 untracked benchmark 予以保留。','- v1.1 冻结 controller：`'+result['v1_1_frozen_controller_sha256']+'`；当前 controller：`'+result['current_controller_sha256']+'`，字节不相同。','- v1.1 tested/D02 历史库：`976c629c8dcf112d80cad7cd994b7840b1f6bfac518e1a9b51bb3fc630e7d4c7`。D02 summary 自带此散列，S04/S05 依赖 v1.1 冻结清单，不能声称每个静态 trial 都有进程映射证明。','- 当前默认库与独立 Release 库详见 JSON。Release：`cceb8c0630836b22916da5e839176497acddabc38cb65028fa3283e5daca85ee`；unoptimized：`59a57620cf721bd9569937718d9e9074d81b4094f9dccfb2f946bdcc8b49d22b`。','- `benchmark/dynamic/diagnostics/release_comparison/evidence/build_identity.json` 记录业务编译命令仅差 `-O3 -DNDEBUG`；其 `before.json/source` 所有已列 src/include/config/test 散列均与当前文件一致。该证据不能证明与较早 v1.1 源码语义等价。','- 运行时 controller_server 参数按 v1.1 首 trial 逐项比较，完整差异在每条 trial 的 runtime_controller_differences。地图路径等全节点差异另存 runtime_all_parameter_differences，避免把场景差异误当算法参数改动。','- 已安装 ROS Humble / Nav2 包版本、v1.1 冻结版本见 JSON；Nav2 1.1.20。','','## 既有诊断能做什么','','- `src/controller_diagnostics.cpp:134–211` 已输出 best、best_forward、fastest_valid 的原始五项 cost、权重、total、rotation_cost、map_mean/map_max、采样/拒绝数量以及分阶段耗时；INFO/WARN 各自按 steady clock 1000ms 节流。不是每周期完整候选列表。','- `benchmark/dynamic/avoidability/decision_probe.cpp` 的 ABI interposition 调用原函数，逐周期记录所有已评估候选的 v/w、free/rejection、端点、mean/max、已评分候选的五项 cost/total、原始地图和 best_poses；best_index 为被选候选。没有所有候选 poses、TopK 或逐栅格统计。','- 原 predictor 按运行参数 1.7s / 0.05s 积分；典型完整轨迹35点。可据输入 pose、候选命令与原地图离线重建候选轨迹和访问栅格；必须检查数值/拒绝一致性，不能把重建字段说成直接采集。']
for x in probes:lines.append(f"- `{x['run']}`：{x['cycles']}周期，{x['evaluated_candidates']}个已评估候选，{x['feasible_candidates']}个 free、{x['scored_candidates']}个带 score；字段与轨迹点数见 JSON。")
lines+=['','现有 linked 仅覆盖 D01 t2/t4。S04/S05、D01 t3/t5/t6、D02 缺候选细节；需额外采集才能直接回答这些条件的候选排序/地图惩罚问题。本清单不启动补采。','','## 复用集成约束','','- 启动脚本 source Humble、默认 workspace 和独立前缀，Release 需要 MYDWA_DIAG_PREFIX；必须使用 plugin_maps 与实际进程库散列确认 overlay，不能只看环境变量。','- `benchmark/run_trial.py:430–497` 先检查生命周期 active、use_sim_time、FollowPath plugin/BT 绑定、/clock/TF/odom、命令链端点和 action server；dynamic 还检查物理起点、obstacle service/contact。','- `benchmark/dynamic/run_trial.py:95–105` 拒绝 /benchmark/model_states 上任何非 /benchmark_collector 订阅者。额外 GT 观察器会导致 preflight 失败，应复用既有 collector；先确认允许拓扑。','- `benchmark/dynamic/run_suite.py:136–158` 与公共 baseline process helper 完成进程停止/清理；清理失败阻止下一 trial。公共 helper 使用 UID/domain 锁，并按当前用户、ROS_DOMAIN_ID、进程组及启动时间筛选目标。同域另一个 baseline 会被视为旧仿真；独立 ROS_DOMAIN_ID 与 GAZEBO_MASTER_URI 仍要遵循既有清理机制。','- ABI 探针依赖符号与函数签名，改变源码、优化链接或版本可能破坏拦截；其开销应与原控制计算分列，不能把 observed compute 当无观测器运行时延。','','## rosbag 与证据边界','','对 `${MYDWA_WS}` 全树（不跟随目录符号链接）查找 `.db3`、`.mcap`、`metadata.yaml`：发现 '+str(len(bags))+' 个。现有离线证据是 JSONL/CSV/配置快照与库/源码散列；不存在可宣称的完整 rosbag 重放链。','']
(OUT/'inventory_findings.md').write_text('\n'.join(lines),encoding='utf-8')
print(json.dumps({'trials':len(records),'manifests':len(manifest_records),'source_hash_entries':len(source_comparison),'source_hash_mismatches':sum(not x['equal'] for x in source_comparison.values()),'probe_inventory':probes,'rosbags':len(bags),'representatives':selection},ensure_ascii=False))
