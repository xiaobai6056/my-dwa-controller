#!/usr/bin/env python3
"""Validate all six recorded trials and derive descriptive build comparisons."""
import csv
import hashlib
import json
from pathlib import Path
import statistics
import sys
import contextlib
import io
from settings import selection,completed_trial,mapping_matches
HERE=Path(__file__).resolve().parent
DIAG=HERE.parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(DIAG))
import read_perf
from analyze import distribution
sys.path.insert(0,str(ROOT/'benchmark'))
from dynamic.evaluation import calculate

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):return json.loads(p.read_text())
def write(p,obj):p.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')
def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])
def f(v):return 'NA' if v is None else f'{v:.3f}'

def main():
    output=HERE/'outputs';output.mkdir(exist_ok=True)
    sequence=load(HERE/'sequence_first.json')
    builds=load(HERE/'evidence/build_identity.json')
    params=load(DIAG/'runs/perf_mydwa_001/trial/summary.json')['runtime_parameters']
    results=[];raw={'unoptimized':[],'release':[]};steady={'unoptimized':[],'release':[]}
    for planned in sequence['runs']:
        d=HERE/'runs'/planned['run']
        manifest=load(d/'manifest.json');summary=load(d/'trial/summary.json')
        prefix,plugin=selection(planned['variant'])
        assert completed_trial(manifest['runner_returncode'],summary),d
        assert manifest['cleanup_returncode']==0 and not manifest.get('supervisor_error'),d
        assert manifest.get('runtime_mapping_verified') and mapping_matches((d/'plugin_maps.txt').read_text(),plugin),d
        assert manifest['runtime_plugin_sha256']==manifest['current_plugin_sha256']==sha(plugin)==builds[planned['variant']]['plugin_sha256'],d
        assert manifest['preload_sha256']==builds['probe_sha256'],d
        assert summary['runtime_parameters']==params,d
        metrics=calculate(d/'trial')
        assert metrics['termination']!='system_error',d
        write(d/'metrics.json',metrics)
        with contextlib.redirect_stdout(io.StringIO()):perf=read_perf.analyze(d)
        samples=[]
        for path in d.glob('perf_*.bin'):
            data,integrity=read_perf.read(path)
            assert integrity['reserved']==integrity['committed'] and integrity['dropped']==0,d
            samples+=data
        assert all(r['controller']==1 for r in samples),d
        samples.sort(key=lambda r:r['start_ns'])
        raw[planned['variant']]+=samples
        early=[r for r in samples if not r['threw'] and 1.5<=r['ros_stamp']-summary['navigation_start_sim_s']<3.5]
        assert early,d
        steady[planned['variant']]+=early
        row=dict(run=planned['run'],order=planned['order'],variant=planned['variant'],repeat=planned['repeat'],
            **perf,metrics=metrics,call_period_ms=distribution([(b['start_ns']-a['start_ns'])*1e-6 for a,b in zip(samples,samples[1:])]),
            source_files={str(p.relative_to(d)):sha(p) for p in d.rglob('*') if p.is_file() and p.name in ('summary.json','manifest.json','plugin_maps.txt','native_commands.jsonl') or p.is_file() and p.suffix=='.bin'},
            validation=dict(valid_measurement=True,runtime_parameters_equal_v02=True,actual_mapping_matches=True,
                raw_wrapper_exit_code=planned['returncode'],raw_trial_exit_code=manifest['runner_returncode'],
                wrapper_exit_false_negative=planned['returncode']!=0,
                note='Raw wrapper/sequence/summary untouched. Trial exit2 is expected completed non-success navigation; all records validated uniformly.'))
        write(d/'measurement_validation.json',row['validation'])
        results.append(row)
    assert len(results)==6 and all(sum(r['variant']==v for r in results)==3 for v in raw)
    groups={}
    for variant,samples in raw.items():
        group=[r for r in results if r['variant']==variant]
        returns=[r for r in samples if not r['threw']]
        groups[variant]=dict(runs=3,wall_ms=distribution([r['wall_ms'] for r in samples]),
            cpu_ms=distribution([r['thread_cpu_ms'] for r in samples]),
            normal_return_wall_ms=distribution([r['wall_ms'] for r in returns]),
            exceptions=sum(r['threw'] for r in samples),score_ms=distribution([r['score'] for r in samples if r['score'] is not None]),
            stages_ms={s:distribution([r[s] for r in samples if r[s] is not None]) for s in read_perf.STAGES},
            score_normal_ms=distribution([r['score'] for r in returns if r['score'] is not None]),
            run_wall_mean_ms=[r['wall_ms']['mean'] for r in group],
            run_mean_range_ms=[min(r['wall_ms']['mean'] for r in group),max(r['wall_ms']['mean'] for r in group)],
            run_call_hz=[r['call_hz_steady'] for r in group],
            run_warnings=[r['warnings'] for r in group],
            collisions=sum(r['metrics']['collision'] for r in group),
            successes=sum(r['metrics']['success'] for r in group),
            pre_motion_wall_ms=distribution([r['wall_ms'] for r in steady[variant]]),
            pre_motion_score_ms=distribution([r['score'] for r in steady[variant] if r['score'] is not None]),
            normal_scored_candidates=dict(minimum=min(r['scored'] for r in returns),maximum=max(r['scored'] for r in returns),mean=statistics.mean(r['scored'] for r in returns)))
    pairs=[]
    for repeat in (1,2,3):
        u=next(r for r in results if r['variant']=='unoptimized' and r['repeat']==repeat)
        r=next(r for r in results if r['variant']=='release' and r['repeat']==repeat)
        pairs.append(dict(repeat=repeat,wall_mean_speedup=u['wall_ms']['mean']/r['wall_ms']['mean'],
            normal_return_mean_speedup=u['returning_wall_ms']['mean']/r['returning_wall_ms']['mean'],
            score_mean_speedup=u['stage_ms']['score']['mean']/r['stage_ms']['score']['mean']))
    speedup=groups['unoptimized']['wall_ms']['mean']/groups['release']['wall_ms']['mean']
    total=dict(classification='build_comparison_only_not_v01_success_rate',runs=results,groups=groups,pairs=pairs,pooled_mean_speedup=speedup)
    write(output/'comparison.json',total)
    columns=['run','variant','n','mean','median','p95','p99','maximum','over_budget_fraction','call_hz','nav_cmd_hz','smoothed_hz','warnings','score_mean_ms','score_p95_ms','exceptions','collision','action']
    with (output/'comparison.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,columns);writer.writeheader()
        for r in results:writer.writerow(dict(run=r['run'],variant=r['variant'],**r['wall_ms'],call_hz=r['call_hz_steady'],nav_cmd_hz=r['output_hz']['/cmd_vel_nav']['steady_receipt_hz'],smoothed_hz=r['output_hz']['/cmd_vel']['steady_receipt_hz'],warnings=r['warnings'],score_mean_ms=r['stage_ms']['score']['mean'],score_p95_ms=r['stage_ms']['score']['p95'],exceptions=r['exceptions'],collision=r['metrics']['collision'],action=r['action_result']))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for variant,samples in raw.items():
        values=sorted(r['wall_ms'] for r in samples)
        axes[0].step(values,[(i+1)/len(values) for i in range(len(values))],label=f'{variant} n={len(values)}')
        group=[r for r in results if r['variant']==variant]
        axes[1].plot([r['order'] for r in group],[r['wall_ms']['mean'] for r in group],'o-',label=variant)
    axes[0].axvline(50,color='crimson',ls='--');axes[0].set(xlabel='computeVelocityCommands wall ms',ylabel='empirical CDF')
    axes[1].set(xlabel='preregistered run order',ylabel='per-run mean wall ms')
    for ax in axes:ax.grid();ax.legend()
    fig.suptitle('D01 Headless / same source + V0.2 probe / empty build type vs Release')
    fig.tight_layout();fig.savefig(output/'comparison.png',dpi=140);plt.close(fig)
    render_report(total,builds)
    print(table(['variant','n','mean ms','P95 ms','Hz each run','warnings each run','score mean ms'],[[v,g['wall_ms']['n'],f(g['wall_ms']['mean']),f(g['wall_ms']['p95']),','.join(f(x) for x in g['run_call_hz']),g['run_warnings'],f(g['score_ms']['mean'])] for v,g in groups.items()]))
    print(f'Pooled mean speedup {speedup:.3f}x; paired speedups {[round(p["wall_mean_speedup"],3) for p in pairs]}')


def render_report(data,builds):
    groups=data['groups'];results=data['runs']
    summary=table(['构建','调用数','均值 ms','中位 ms','P95 ms','P99 ms','最大 ms','>50 ms'],[[v,g['wall_ms']['n'],*[f(g['wall_ms'][k]) for k in ('mean','median','p95','p99','maximum')],f"{g['wall_ms']['over_budget_fraction']:.2%}"] for v,g in groups.items()])
    per_run=table(['顺序/运行','n','均值 ms','中位 ms','P95 ms','P99 ms','最大 ms','>50ms','调用Hz','missed','score均值ms','碰撞'],[[r['run'],r['wall_ms']['n'],*[f(r['wall_ms'][k]) for k in ('mean','median','p95','p99','maximum')],f"{r['wall_ms']['over_budget_fraction']:.1%}",f(r['call_hz_steady']),r['warnings'],f(r['stage_ms']['score']['mean']),r['metrics']['collision']] for r in results])
    score=table(['构建/score','n','均值 ms','中位 ms','P95 ms','P99 ms','最大 ms','>50ms'],[[v,g['score_ms']['n'],*[f(g['score_ms'][k]) for k in ('mean','median','p95','p99','maximum')],f"{g['score_ms']['over_budget_fraction']:.1%}"] for v,g in groups.items()])
    rates=table(['运行','调用Hz/steady','cmd_vel_nav Hz/steady','cmd_vel Hz/steady','入口周期P95 ms','异常调用','终态'],[[r['run'],f(r['call_hz_steady']),f(r['output_hz']['/cmd_vel_nav']['steady_receipt_hz']),f(r['output_hz']['/cmd_vel']['steady_receipt_hz']),f(r['call_period_ms']['p95']),r['exceptions'],r['action_result']] for r in results])
    normal=table(['构建/正常返回','n','均值ms','P95ms','score均值ms','全调用CPU均值ms','每轮全调用均值范围ms'],[[v,g['normal_return_wall_ms']['n'],f(g['normal_return_wall_ms']['mean']),f(g['normal_return_wall_ms']['p95']),f(g['score_normal_ms']['mean']),f(g['cpu_ms']['mean']),'–'.join(f(x) for x in g['run_mean_range_ms'])] for v,g in groups.items()])
    pairs=table(['配对','整体均值加速','正常返回均值加速','score均值加速'],[[p['repeat'],*[f(p[k])+'×' for k in ('wall_mean_speedup','normal_return_mean_speedup','score_mean_speedup')]] for p in data['pairs']])
    early_table=table(['构建/导航后1.5–3.5s','n','整体均值ms','score均值ms'],[[v,g['pre_motion_wall_ms']['n'],f(g['pre_motion_wall_ms']['mean']),f(g['pre_motion_score_ms']['mean'])] for v,g in groups.items()])
    release=groups['release'];u=groups['unoptimized']
    report=f'''# MyDWA Release 编译优化性能对照

日期：2026-10-20；六轮实际D01 Headless诊断，未混入V0.1正式基准。

## 结论

Release已成功构建并在三轮运行中实际加载。相同当前源码的空build type与Release相比，实际业务编译命令仅多 `-O3 -DNDEBUG`。整体调用均值从 **{u['wall_ms']['mean']:.3f} ms降至{release['wall_ms']['mean']:.3f} ms，合并均值加速{data['pooled_mean_speedup']:.2f}×**。每轮波动和配对加速见下表；该变化不能用三轮正常运行波动解释，但本实验不分离-O3与-DNDEBUG的各自贡献。

Release三轮实际调用频率为 **{', '.join(f(x) for x in release['run_call_hz'])} Hz**，missed警告 **{release['run_warnings']}**，函数超过50ms比例 **{release['wall_ms']['over_budget_fraction']:.1%}**。这支持在本次三轮短时D01中维持目标20Hz，不构成长时间/所有场景实时性保证。

D01物理接触：未优化 **{u['collisions']}/3**，Release **{release['collisions']}/3**。这是附带观察，不是构建优化验收条件，也不作为新的正式成功率推断。性能改善不等于动态避障问题已解决。

## 1. 冻结与隔离

工作分支feature/dynamic-obstacle。开始时Git已有 .gitignore、controller.hpp、controller.cpp修改及未跟踪benchmark；原样保留。初始Git差异、源码/配置/旧记录SHA、构建缓存、flags和原插件备份在evidence。没有git add/commit/push。没有修改CMakeLists、业务C++、YAML、D01尺寸/路径/运动触发和评价阈值。

两个独立CMake build/install prefix分别为 `${{MYDWA_WS}}/{{build,install}}_compile_comparison_20261020/{{unoptimized,release}}`，原build/install未覆盖。两者都从同一工作区源码、GCC、BUILD_TESTING=ON构建，所有package内部静态库随相同build type编译，没有重编译ROS/Nav2/DWB系统包。unoptimized是空build type（无-O），不是带-g的Debug。

原安装插件SHA256：`c38de8839ea2f22bfdd88f484316897dd4dc9b97f31b61052687ed1b90031c35`。

- 新未优化插件：`{builds['unoptimized']['plugin_sha256']}`。
- Release插件：`{builds['release']['plugin_sha256']}`。
- 同一V0.2探针：`{builds['probe_sha256']}`。

每轮从 `/proc/<PID>/maps`保存真实映射，严格拒绝错误路径、双副本或deleted映射；manifest保存运行时SHA且与已归档对应.so一致。NAVPERF_MYDWA_LIB也指向同一选定路径，防止探针偷偷调用原安装。六轮全部通过加载核对，不能只凭插件类名声称Release已加载。

Release编译flags：`-O3 -DNDEBUG -fPIC -Wall -Wextra -Wpedantic`；未优化flags：`-fPIC -Wall -Wextra -Wpedantic`。capture_build核对所有业务编译命令，剔除-O3/-DNDEBUG后逐项一致；完整verbose build日志、compile_commands、缓存和二进制在evidence。没有-fast-math、LTO、-march=native或任何Scorer源级优化。

## 2. 公平性与实际运行

预先固定顺序U1,R1,R2,U2,U3,R3，配对顺序交替；全为D01/mydwa/headless，同一WSL宿主、ROS_DOMAIN_ID91、Gazebo master11436、V0.2原探针、参数文件与原导航/运动/评价程序。每次完整重启仿真、独立初始定位和目标，结束后统一3秒空档。编译及功能测试完成后才测量，无并发构建或其他仿真实例。

六份runtime_parameters与V0.2对应基线全字典一致；原始参数、场景和V0.1/V0.2文件由audit逐SHA复核。sequence_first.json保留真实顺序、UTC、CPU affinity、/proc/stat和loadavg。WSL未提供cpufreq读数，不能保证CPU频率/热状态和Windows后台负载严格相同；这是同一机器连续交替实测，不是假称锁定全部硬件因素。

外部探针保持原版本，每次只mmap记录原函数wall steady_clock及thread CPU，原调用转发一次，复制原有MyDWA分段；无新增控制输入。沿用V0.2约1.26µs/调用合成校准，未假设它是Release实际开销的普遍上界。探针单Controller串行使用，不宣称并发初始化安全。函数计时不等于整个Nav2周期；20Hz对应50ms，missed是原ControllerServer WallRate截止点判断。

**基础设施问题保留**：初版包装器继承V0.2的runner exit0假设；实际原run_trial.py对“完整记录的非成功导航”返回2。因此六轮的包装器/批量退出码非零，但不是导航未执行。没有改写这些原始manifest/sequence/summary，而是统一校验valid、finished、Action accepted、无action_error、参数一致、实际插件映射正确、mmap完整后追加measurement_validation.json及派生metrics。六轮均合格，全数纳入；没有因碰撞丢弃或挑选重跑。独立包装器返回码规则已修复并红绿回归，原V0.2文件未动。

## 3. computeVelocityCommands 整体分布

{summary}

单位ms，百分位为全部调用样本线性插值，超预算严格>50ms。合并调用分布按样本数加权，不冒充3个独立实验的置信区间；下面保留每轮，另列正常返回子集，避免异常快速返回扭曲比较。

{per_run}

## 4. score、CPU与正常返回子集

{score}

{normal}

完整threadCPU中位/P95/P99/最大、各原有分段及候选数分布在outputs/comparison.json。异常调用没有删除；正常返回子集另列。速度采样无独立可靠计时，剩余项不写成0。已保存每周期processed/valid/scored计数；闭环接触前后的无可行轨迹调用、候选有效数及路径可随运行不同，因此同时列正常返回对照。未做离线固定输入微基准，不能把所有差异都拆解成单个函数的纯指令执行差异。

{pairs}

合并整体均值加速 **{data['pooled_mean_speedup']:.3f}×**。每轮均值范围与配对数值说明构建效应的量级；仅3轮，不做显著性检验或跨场景普遍排名。

## 5. 控制频率与输出频率

{rates}

调用Hz为原函数入口首末steady跨度，输出Hz为独立订阅器原生消息接收steady跨度，限定导航窗口。原始JSON另含sim clock频率；/cmd_vel是平滑后的20Hz底盘输出，不能据此误认原插件也20Hz。入口周期P95描述调度间隔，不能直接当函数耗时。所有六轮accepted目标，真实物理接触后测试器取消Action，终态CANCELED；失败全部保留。

![六轮耗时分布及顺序](outputs/comparison.png)

## 6. 功能、原始记录与回退

两套构建分别运行现有9个功能CTest目标（8个C++ GTest目标及benchmark Python），并执行相关全部52项Python测试，最终日志见evidence/*functional_tests_final.log与python_tests_final.log；仅功能测试，不声称全部lint通过。Release构建不修改源算法，-DNDEBUG是标准Release条件的一部分。

直接CMake安装setup位于share/my_dwa_controller/local_setup.bash，早期脚本错误地查找根local_setup并中断；只修复测试启动路径，不改CMake/源码，原错误日志保留。最终测试在正确selected prefix下重跑通过。原始数据在runs（bin、指令、传感器、contact、GT、summary、maps与日志），逐样本CSV及耗时汇总也保留；outputs/给出逐轮CSV/JSON和图。归档包含二进制/编译参数/原始数据。

默认安装完全不变，退出诊断子进程并在干净终端source原ROS和原workspace setup即可回到原版本；现有默认启动器仍加载原.so。只读audit核验当前源码、旧记录、原安装未变。具体复现命令见README。

## 7. 是否继续优化Scorer

Release已大幅降低原先主要计算开销；判断是否继续优化应看当前score绝对耗时与50ms预算余量，而非只看相对占比。本次Release的score平均14.75ms，仍占整体约96.1%，但只占50ms周期预算的约29.5%；它仍是主要热点，已不构成本次20Hz超时瓶颈，源级Scorer优化不再是恢复20Hz的首要任务。仍可为更大路径/采样规模做后续叶函数采样与长时压力验证，但此次没有改代码来“验证猜想”。D01碰撞机制需要独立逐周期轨迹/评分/实际Costmap证据，不能靠编译提速直接下结论。
'''
    (HERE/'REPORT_RELEASE.md').write_text(report)

if __name__=='__main__':main()