#!/usr/bin/env python3
"""Regenerate V0.2 tables/plots/report from existing records without running ROS."""
import csv
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
OUT=HERE/'outputs'

def load(p):return json.loads(p.read_text())
def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,row))+' |' for row in rows])
def fmt(v,places=1):return '未检出' if v is None else f'{v:.{places}f}'

def main():
    events=load(OUT/'v0_1_analysis/all_events.json')
    perf={c:load(HERE/f'runs/perf_{c}_001/perf_summary.json') for c in ('mydwa','dwb')}
    with (OUT/'events.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['run',*[f'T{i}' for i in range(8)],'response_kind','warnings'])
        writer.writeheader()
        for r in events:writer.writerow(dict(run=r['run'],**r['events'],response_kind=r['response_kind'],warnings=r['warnings']))
    with (OUT/'performance.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['controller',*perf['mydwa']['wall_ms'],'call_hz_steady','warnings','exceptions'])
        writer.writeheader()
        for c,r in perf.items():writer.writerow(dict(controller=c,**r['wall_ms'],**{k:r[k] for k in ('call_hz_steady','warnings','exceptions')}))
    (OUT/'performance.json').write_text(json.dumps(perf,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for c in perf:
        with (HERE/f'runs/perf_{c}_001/perf_samples.csv').open() as f: rows=list(csv.DictReader(f))
        wall=[float(r['wall_ms']) for r in rows]
        axes[0].plot(sorted(wall),[(i+1)/len(wall) for i in range(len(wall))],label=f'{c}: n={len(wall)}')
        axes[1].scatter(wall,[float(r['thread_cpu_ms']) for r in rows],s=12,alpha=.6,label=c)
    axes[0].axvline(50,color='crimson',ls='--',label='20 Hz budget')
    axes[0].set(xlabel='outer computeVelocityCommands wall ms',ylabel='empirical CDF')
    axes[1].plot([0,85],[0,85],ls='--',color='gray')
    axes[1].set(xlabel='wall ms',ylabel='thread CPU ms')
    for ax in axes:ax.grid();ax.legend()
    fig.suptitle('D01 / perf_mydwa_001 vs perf_dwb_001 / current workspace diagnostics')
    fig.text(.02,.01,'Sources: committed perf_*.bin, steady_clock + CLOCK_THREAD_CPUTIME_ID. Separate from V0.1 formal data.',fontsize=8)
    fig.tight_layout(rect=[0,.04,1,.95]);fig.savefig(OUT/'performance.png');plt.close(fig)
    event_table=table(['正式 D01 运行','T1','T2','T3*','T4*','T5','T6','T7','显著响应'],[
        [r['run'],*[fmt(r['events'][f'T{i}']) for i in range(1,8)],r['response_kind'] or '未检出'] for r in events])
    perf_table=table(['当前诊断','n','平均 ms','中位 ms','P95 ms','P99 ms','最大 ms','>50 ms'],[
        [c,r['wall_ms']['n'],*[fmt(r['wall_ms'][k],3) for k in ('mean','median','p95','p99','maximum')],f"{r['wall_ms']['over_budget_fraction']:.1%}"] for c,r in perf.items()])
    rate_table=table(['当前诊断','调用 Hz/steady','/cmd_vel_nav Hz/steady','同话题 Hz/sim','/cmd_vel Hz/steady','警告','结果'],[
        [c,fmt(r['call_hz_steady'],2),fmt(r['output_hz']['/cmd_vel_nav']['steady_receipt_hz'],2),fmt(r['output_hz']['/cmd_vel_nav']['sim_receipt_hz'],2),fmt(r['output_hz']['/cmd_vel']['steady_receipt_hz'],2),r['warnings'],'collision / CANCELED'] for c,r in perf.items()])
    stage_table=table(['MyDWA 已有分段','n','平均 ms','P95 ms'],[
        [s,r['n'],fmt(r['mean'],4),fmt(r['p95'],4)] for s,r in perf['mydwa']['stage_ms'].items()])
    contact_table=table(['正式运行','附近机器人 XY m','附近盒 XY m','附近实际 v m/s','附近实际 ω rad/s'],[
        [r['run'],','.join(fmt(x,3) for x in r['locations']['T7']['robot'][:2]),','.join(fmt(x,3) for x in r['locations']['T7']['obstacle'][:2]),fmt(r['locations']['T7']['actual_vx'],3),fmt(r['locations']['T7']['actual_wz'],3)] for r in events])
    runs=[]
    for d in sorted((HERE/'runs').iterdir()):
        if not (d/'metrics.json').exists():continue
        m=load(d/'metrics.json');s=load(d/'trial/summary.json')
        runs.append([d.name,'GUI' if d.name.startswith('gui') else 'Headless+probe',m['controller'],m['termination'],m['action_result'],fmt(m['navigation_sim_s'],1),str(s['valid'])])
    runs.append(['v02_headless_compat_001','原 V0.1 Headless 入口','dwb','collision','CANCELED','6.8','True'])
    run_table=table(['补充记录','模式','插件','终止','Action','任务仿真秒','有效'],runs)
    report='''# Dynamic Benchmark V0.2 诊断报告

日期：2026-10-20；分支 feature/dynamic-obstacle。本次只新增诊断工具与记录，没有修改 Controller、Nav2 参数、D01/D02 场景或运动脚本，没有 commit/push，也没有重跑 12 次正式基准。

## 首先回答三个问题

**D01 为什么碰撞？已确定的是“早期能观测到静止盒，随后机器人和横穿盒在相近时空继续接近，已有控制反应不足以避免物理接触”。尚不能确定内部选择原因。** 六轮 V0.1 都有盒对应的激光与致命 Costmap 栅格，碰撞源时间相对导航约 6.8–7.0 秒。MyDWA 第1轮在碰撞前保持 0.26 m/s，未出现满足定义的持续显著转向；DWB 第1轮在约6.3秒开始显著转向，但约6.8秒仍碰撞。不能据此认定感知完全正常、轨迹预测有错或评分必然有错：没有每周期内部 Costmap 快照、全部候选轨迹及评分。

**当前工作区 MyDWA 构建确有计算性能瓶颈。** 同口径外部实测：83次调用平均76.25 ms、96.4%超过50 ms；线程CPU平均76.24 ms。DWB 135次平均1.64 ms，全部小于50 ms。MyDWA 已有计时将主要开销定位到 score 段，平均74.20 ms（约占整体97.3%）。这不是仅靠警告数量推断；也不证明性能瓶颈就是碰撞原因，DWB 在20 Hz下仍然碰撞。

**下一步**：优先补齐临近碰撞的逐周期候选轨迹/评分和实际使用的 Costmap；再定位 score 内部叶函数与现有构建条件的影响；最后用可控的交互时序实验区分动态预测、制动/转向空间与感知更新。此次均未修改算法或调参。

## 1. 数据与冻结边界

正式时序分析只读取 `benchmark/results/dynamic/formal_v0_1_20261020/D01/{mydwa,dwb}/trial_00{1,2,3}`。24张图和各轮 events.json 在 `outputs/v0_1_analysis`；汇总 CSV 为 `outputs/events.csv`。每轮 events.json 保存原始文件 SHA256、绝对源/接收时钟、关联计数和附近 Ground Truth。

开始时工作区已有未提交的 controller.cpp/controller.hpp 重命名重构，未触碰。当前已安装 MyDWA 二进制 SHA256 为 `c38de8839ea2f22bfdd88f484316897dd4dc9b97f31b61052687ed1b90031c35`；与 V0.1 的 `976c629c8dcf112d80cad7cd994b7840b1f6bfac518e1a9b51bb3fc630e7d4c7` 不同。因此补充运行全部标记为 current_workspace_diagnostic_not_v0.1，不能合并成正式基准样本。V0.1 本身稀疏 diagnostics 也已有约78–80 ms 的计算记录，支持旧版本也慢；未对旧二进制做全调用分布测量。

`evidence/integrity_audit.json` 验证2582个既有文件（含原始正式记录、场景资产、参数、源码、静态场景、工作区启动脚本及当前插件）均与初始快照一致。快照另有1个本次新建测试文件，后来移动至 diagnostics/test_diagnostics.py，明确排除。五轮新诊断运行保存的全部 runtime_parameters 与各自 V0.1 同插件第1轮逐项完全相同，日志确认实际加载对应插件。Ground Truth 仅由原测试观察器评分；新 cmd_probe 仅订阅话题，性能探针只转发原调用与复制统计，不向控制器注入真值。

## 2. GUI 与实际运行

新增 run_once.sh/run_once.py、launch_scene.sh、dynamic.rviz、camera_view.py。复用原 Gazebo/Nav2 启动与 baseline_preflight/清理函数、场景资产、原 run_trial.py 和运动程序；GUI 只改变 headless/use_rviz/rviz_config_file 及 GUI 相机。相机通过 Gazebo GUI 消息调整俯视视角，不修改物理或障碍运动。

当前 WSLg DISPLAY=:0、WAYLAND_DISPLAY=wayland-0，Gazebo11.10.2 与 RViz 可启动。实际检查 Gazebo 的机器人/白盒/激光、RViz Global Status Ok、TF/RobotModel/Scan/Map/Local Costmap/Global Path；截图见 `outputs/gui`。初次 Gazebo 视角被高墙遮挡，之后使用自动俯视相机完成第二次 MyDWA 验证。DWB 通过 GUI transport 调整为相同视角验证，自动相机代码随后在 MyDWA 验证。没有新增预测轨迹发布接口。

'''+run_table+'''

表中碰撞是有效失败记录，不是导航成功。达到物理接触后原测试器取消 Action，故终态 CANCELED。GUI 首轮/复核轮、两轮性能诊断、一次旧入口兼容性，共6轮补充单次运行；均独立进程、顺序执行，ROS_DOMAIN_ID=91、专用 Gazebo master 11436、原 preflight 锁与清理机制。没有重跑正式12轮。GUI结束后默认保留60秒，可配置；记录永久保留。D02选项沿用同一资产/启动逻辑，已做参数与语法检查，本轮没有再运行 D02 GUI，也没有深入其 Recovery。

## 3. T0–T7 检测规则与分辨率

所有表格相对 T0，保留一位小数。JSON保留原数值用于核对，不表示毫秒级的跨话题定位精度。

- T0：summary.navigation_start_sim_s，发送目标时最新接收的 /clock；Action确实 accepted，但没有精确 acceptance 源时间。
- T1：首条盒 Y 相对初值变化>0.005 m 的 Ground Truth。运动计划固定T0+4.0秒，观测约4.1秒；接收时钟约0.1秒量化。
- T2：固定名义直线路径 x∈[0,4]、机器人横向半宽0.153 m，与盒半宽0.25 m 的扫掠区域首次重叠，即 |盒y|≤0.403。约6.8秒；它是狭义物理走廊进入条件，并非最早能预测到危险的时刻，也不等于实际机器人位置碰撞。
- T3*：**首条保留的**盒关联 LaserScan 源时间。利用已存 laser_world/obstacle_world 做射线与盒矩形相交，距离与预期入射面相差≤0.08 m，至少3条相邻有效射线，遵守传感器量程（默认float32的0.12–20 m）。不是最近非零读数。
- T4*：**首条保留的**盒关联 OccupancyGrid 源时间。将格心转换到盒坐标，物理投影内至少3个值恰为100的栅格；99仅为内切膨胀值，不计作致命障碍。[Nav2 官方转换定义](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_costmap_2d/src/costmap_2d_publisher.cpp)。修正旧的>=99离线命名后，六轮T4不变，首帧均有13个100值栅格。
- T5：原 /cmd_vel_nav 在 trajectory.csv 中≤10 Hz采样，排除启动加速（只查T0+4秒后）及首次接触前最后0.1秒；连续采样跨度≥0.2秒满足 v≤0.21 m/s 或 |ω|≥0.1 rad/s。分别检查减速/转向，再选最早者。它是显著行为变化指标，不能证明避障意图；短暂减速或较小转向仍可能存在。
- T6：T5之后实际GT速度满足对应类型的同一阈值并持续≥0.2秒；无T5或不满足时不确定。T5/T6接收时钟及采样误差约0.1秒，不能把相同时间格理解成无底盘延迟。
- T7：contacts.jsonl 中最早物理接触消息的 **stamp**，名义100 Hz。不是cancel返回时刻。T7相对T0仍继承T0的接收时钟误差。

原观察器在T0才启用记录，盒早已在侧方静止且可见。**真正首次感知时刻不可确定；T3*/T4*只是上界证据，甚至可能在记录前已存在，不能用T4−T3估计感知延迟。** 扫描约5 Hz、GT约50 Hz但接收clock约10 Hz、Controller抽样≤10 Hz；Costmap发布帧率另有限制。图中短横条仅示接收clock/接触采样尺度，不是检测起始时间区间或置信区间。

'''+event_table+'''

T0均为0。未检出表示在规定窗口与持续阈值下未检出，不表示机器人从未变化。所有T7均为真实物理接触，六轮失败都保留。

## 4. D01 数值证据与解释

第1轮：MyDWA 首条保留扫描的关联最近量程1.666 m，附近外轮廓近似净间距1.451 m；DWB为1.643 m和1.451 m。这是激光原点量程与包络距离的两个不同概念，不是模型中心距离，也不是运动危险刚产生时的距离。T4分别约0.7/0.6秒，距接触约6.2秒；此时盒尚未启动，不能把这6秒称为反应迟滞。

MyDWA第1轮T0+4秒后到接触前0.1秒的原始控制指令一直v=0.26 m/s，最大|ω|=0.034 rad/s；没有本定义下持续减速或转向。DWB第1轮原始指令最小v=0.233 m/s，最大|ω|=0.263 rad/s，在约6.3秒有持续转向，仍保持前进，没有降至≤0.21 m/s的持续制动。第2/3轮存在短暂零速/角速度尖峰等差异，因此检查了全部六轮，未只展示第1轮；MyDWA第3轮和DWB第2轮也检出持续转向。短暂零速不等于有效停车，接触后的大变化不能用来证明接触前避障。

V0.1 MyDWA 低频日志确有一些周期 sampled=420、valid_forward>0、best.v=0.26，说明那些被记录周期仍有前进候选并选择前进。它不支持“所有临碰周期所有候选都安全”，也无法证明碰撞那一周期为何这样评分。现有扫描和发布的Costmap记录不等于插件每周期实际读取的内部地图。

以下位置/速度来自T7附近最近的GT **接收clock** 样本，可能相差约0.1秒；不是接触瞬间精确状态。盒运动计划此阶段向+y为0.25 m/s，GT位置也显示横移；盒源接触时刻速度没有独立精确采样。

'''+contact_table+'''

机器人外轮廓采用原V0.1保守矩形 x=[−0.1965,0.077]、y=[−0.153,0.153]（相对base_footprint），盒0.5×0.5 m矩形；gap为两个变换后多边形的最短边界距离，重叠截为0。它不是完整三维碰撞几何的精确真值。接触标记处图上gap仍可能略正：附近GT与contact源异步，不能据此推翻物理接触或声称严格同步。

**事实**：两者均早有对应观测/栅格、在盒横移时继续接近、最终物理接触；部分轮次有转向但未避开。**推断**：当前反应强度/时机不足以完成本交互，单纯“完全看不到盒”不足以解释所有记录。**无法判断**：内部地图滞后程度、候选预测是否涵盖盒未来位置、权重/足迹是否为主因、控制延迟对接触贡献大小。不能仅凭外部可见性认定Controller错误。

代表图（全部六轮24张均可在同级目录打开）：

| 正式第1轮 | 速度指令/实际速度 | XY与包络 | 净间距 | 事件轴 |
| --- | --- | --- | --- | --- |
| MyDWA | [speed](outputs/v0_1_analysis/mydwa/trial_001/speed.png) | [XY](outputs/v0_1_analysis/mydwa/trial_001/trajectory.png) | [gap](outputs/v0_1_analysis/mydwa/trial_001/clearance.png) | [T0–T7](outputs/v0_1_analysis/mydwa/trial_001/timeline.png) |
| DWB | [speed](outputs/v0_1_analysis/dwb/trial_001/speed.png) | [XY](outputs/v0_1_analysis/dwb/trial_001/trajectory.png) | [gap](outputs/v0_1_analysis/dwb/trial_001/clearance.png) | [T0–T7](outputs/v0_1_analysis/dwb/trial_001/timeline.png) |

## 5. 周期警告的实际含义

安装 nav2_controller1.1.20、rclcpp16.0.21。核对[对应官方 ControllerServer 源码](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_controller/src/controller_server.cpp)：computeControl 创建 WallRate(controller_frequency_)，循环会等待Costmap current、更新路径、获取机器人姿态/进度/里程计、调用插件、反馈/发布及GoalChecker，最后调用loop_rate.sleep()；无法按Rate的下一个截止点睡眠时产生Control loop missed警告。安装的rate.hpp证实WallRate使用steady_clock，不使用ROS仿真时钟；其相位追赶/落后重置规则意味着警告数不必等于插件>50 ms次数。

当前冻结参数 controller_frequency=20 Hz，目标周期50 ms。TF等待、Costmap同步、调度、其他循环开销都可能参与，因此原先警告本身不能证明插件慢；本次另测完整插件函数才进一步定位。源码证据与安装头、包版本、构建flags见 evidence/provenance.json、installed_rate.hpp、packages.txt、build.txt。

## 6. 同口径性能观测

外部libnavperf.so通过Linux x86_64 ELF interposition覆盖两种插件**同一个外层签名**的computeVelocityCommands，精确转发原实现一次，保留返回值/输入/异常。使用steady_clock和CLOCK_THREAD_CPUTIME_ID；每次仅写固定mmap内存槽，首次创建映射，无每周期同步文件I/O/INFO。插件原二进制不重建。MyDWA通过原有diagnostics.report复制既有分段计时，不新增控制分支；DWB没有同样内部分段，标记缺失而非0。

5对合成转发校验每对4002次调用（含2个保留异常），原始函数调用数/返回值/输入不变；新增开销中位约1.26 µs/调用，复验约1.29 µs。空函数校准不是实际场景的普遍误差上界；复制分段和首次解析/映射、缓存、观测调度仍有扰动。全体运行单Controller，Nav2 computeControl持有互斥锁串行执行；**此探针尚不支持并发首次初始化**，不得推广到并发插件/多Controller。

原MyDWA当前构建CMAKE_BUILD_TYPE为空，core/scorer flags未含-O优化（−fPIC −Wall −Wextra −Wpedantic）。这是已存在的构建条件，此次没有切换Release或重建算法。外部观察器自身用-O2编译；不能将其误写为对Controller优化，也不能将当前结果视为优化生产构建的固有效率。DWB使用已安装系统包。

'''+perf_table+'''

MyDWA包括3个原函数抛异常的调用（均保留，平均1.667 ms）；80个正常返回调用平均79.046 ms，中位79.146 ms，P95=82.112 ms，P99=82.662 ms，最大82.898 ms，全部>50 ms。DWB135次均正常返回。记录槽reserved=committed、dropped=0，原始mmap和逐样本CSV均保留。

线程CPU：MyDWA平均76.242、中位79.075、P95=81.952、P99=82.651、最大82.896 ms；DWB平均1.639、中位1.549、P95=2.020、P99=2.612、最大3.221 ms。wall≈thread CPU支持该轮主要为线程计算消耗，而非长时间被调度出去；不能排除WSL负载、缓存、CPU频率、Gazebo资源竞争及首次测量开销，也不提供跨机器普遍结论。

'''+rate_table+'''

调用Hz使用首末原函数入口steady跨度，输出Hz使用独立订阅器原生消息的接收steady跨度（另列sim clock口径），限定导航窗口，包含必要停止输出。/cmd_vel为平滑后的底盘输出，20 Hz并不表示插件20 Hz；MyDWA /cmd_vel_nav仅约11.93 Hz。旧trajectory.csv最多10 Hz，不能从旧采样数据声称精确20 Hz。原始完整频率字典包括样本数与两种话题sim频率，见 outputs/performance.json。

'''+stage_table+'''

score约97.3%的整体均值，已可定位为主要段；predict约1.37 ms、obstacle检查约0.28 ms、map锁等待很短。prefix_to_report是wrapper入口至report调用前的近似区间，不是独立阶段，不能与其余行求和。state_wait是原结构中的计时名称，不代表所有TF等待；速度采样没有独立可靠分段值，余项也不能当0。源码 src/trajectory_scorer.cpp 中每候选路径距离遍历预测点并投影到local_path，提示可调查重复路径工作；尚无叶函数采样证据，不能直接断言就是该内层操作的全部代价。两轮短诊断都碰撞，统计只描述本次调用分布，不是新的成功率估计。

![同口径性能分布与线程CPU](outputs/performance.png)

## 7. 验证、工具修复与交付

- Python现有静态/动态与新增规则共49项通过；工作区现有9个功能CTest目标通过（含benchmark及8个C++测试目标），未重建核心库。语法compileall与三份shell bash -n通过；探针真实构建、转发校准及原始样本读取通过。没有声称完成全部lint目标。
- GUI两种插件均真实导航并保存失败结果；原V0.1入口独立Headless兼容测试完成；静态S01–S06数据/源码哈希不变。
- 独立只读代码复核发现并修正两处新工具问题：Costmap99不能称lethal（改100并离线重算）；run_once失败可能exit0（增加返回码/监督错误/完整结果/中断检查）。先构造失败回归，再修正后通过。有效collision记录的工具exit0仍不代表导航成功。
- 原始Contact最早stamp与六份events逐项一致；原始输入SHA核对、五轮运行实际插件加载/全runtime_parameters核对通过；没有遗留活跃Gazebo/RViz/Nav2实例。截图人工核验，24张离线图已生成并抽查速度/轨迹/时间轴。
- 原始新运行在 runs/；压缩归档 outputs/diagnostic_runs.tar.gz 另包含旧入口兼容运行，V0.1原始记录保持原位置；工具运行日志/校准/冻结审计在 evidence/。README含干净终端命令和限制。

## 8. 后续研究（本次未实施）

1. 临碰前逐周期记录原始候选轨迹、分项评分、拒绝理由、使用的Costmap时间/足迹，验证“仍选择前进”的具体机制。
2. 对score段做叶函数级采样，单独研究现有构建优化条件与重复路径计算的成本；保持对照和观测版本明确。
3. 后续另立实验固定交互时序并扫描少量相遇条件，区分未来动态占据、制动/转向可行空间与感知更新造成的差异；不从本次3次样本宣称某Controller全面更优。
'''
    (HERE/'REPORT_V0_2.md').write_text(report)
    print('Generated events.csv, performance.csv/json/png, REPORT_V0_2.md')

if __name__=='__main__':main()