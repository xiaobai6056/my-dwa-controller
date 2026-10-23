"""Render evidence-limited descriptive timing-scan report from derived records."""
from collections import Counter
import json
from pathlib import Path
import statistics
HERE=Path(__file__).resolve().parent

def fmt(v,n=1):return 'NA' if v is None else f'{v:.{n}f}'
def table(headers,rows):return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,row))+' |' for row in rows])

def render(rows,groups,complete,output_name='outputs',tag='first'):
    valid=[r for r in rows if r['valid']]
    high=[s for s in (2,3,4,5,6) if all(any(g['start_time_s']==s and g['controller']==c and g['valid']==3 and g['collisions']>=2 for g in groups) for c in ('mydwa','dwb'))]
    safe=[s for s in (2,3,4,5,6) if all(any(g['start_time_s']==s and g['controller']==c and g['valid']==3 and g['collisions']==0 for g in groups) for c in ('mydwa','dwb'))]
    no_interaction=[r for r in valid if r['category'].startswith('no_interaction')]
    counts=Counter(r['category'] for r in valid)
    summary=table(['启动s','Controller','有效/计划','Nav2成功','物理碰撞','成功均时s','最小gap/中位最小gap m','实际首动范围s'],[
        [g['start_time_s'],g['controller'],f"{g['valid']}/3",f"{g['successes']}/{g['valid']}",f"{g['collisions']}/{g['valid']}",fmt(g['success_mean_sim_s']),fmt(g['min_gap_approx_m'],3)+' / '+fmt(g['median_min_gap_approx_m'],3),str([round(x,1) for x in g['actual_start_range_s']]) if g['actual_start_range_s'] else 'NA'] for g in groups])
    details=table(['顺序/运行','Action/终止','任务持续s','最小gap m','首减速s','首转向s','接触s','交互分类'],[
        [r['run'],r['metrics']['action_result']+'/'+r['metrics']['termination'],fmt(r['metrics']['navigation_sim_s']),fmt(r['metrics']['min_gap_approx_m'],3),fmt(r['features']['brake_s']),fmt(r['features']['turn_s']),fmt(r['contact']['relative_s']) if r['contact'] else 'NA',r['category_label']] for r in rows])
    contacts=table(['运行','附近机器人XY m','附近盒XY m','实际v/ω','接触对象'],[
        [r['run'],','.join(fmt(x,2) for x in r['contact']['robot'][:2]),','.join(fmt(x,2) for x in r['contact']['obstacle'][:2]),fmt(r['contact']['actual_vx'],2)+' / '+fmt(r['contact']['actual_wz'],2),'dynamic_obstacle' if any('dynamic_obstacle' in p for p in r['contact']['pair']) else str(r['contact']['pair'])] for r in rows if r['contact']])
    paragraphs=[]
    for start in (2,3,4,5,6):
        for controller in ('mydwa','dwb'):
            group=[r for r in valid if r['start_time_s']==start and r['controller']==controller]
            if not group:continue
            available=sum(r['features']['command_evidence_complete'] for r in group)
            brake=sum(r['features']['brake_s'] is not None and r['features']['command_evidence_complete'] for r in group)
            turn=sum(r['features']['turn_s'] is not None and r['features']['command_evidence_complete'] for r in group)
            wait=sum(r['features']['waiting_before_zone_s']>=.5 for r in group)
            labels=Counter(r['category_label'] for r in group)
            order=[]
            for r in group:
                rw,ow=r['features']['robot_windows'],r['features']['obstacle_windows']
                if rw and ow:
                    separation=min(w[0] for w in ow)-max(w[1] for w in rw)
                    margin='末端截断，真实退出时刻未知' if r['features']['robot_window_right_censored'] else f'{separation:.1f}s'
                    order.append(f"r{r['repeat']}:机器人x区{[[round(x,1) for x in w] for w in rw]}、盒路线区{[[round(x,1) for x in w] for w in ow]}，机器人先出的时间裕量{margin}")
            figures=', '.join(f"[r{r['repeat']}图]({output_name}/figures/{r['run']}.png)" for r in group)
            paragraphs.append(f"### {start}s / {controller}\n\n指令有效覆盖{available}/{len(group)}；显著减速{brake}/{len(group)}，显著转向{turn}/{len(group)}，交叉区前连续静止≥0.5s为{wait}/{len(group)}。分类：{dict(labels)}。这些是观测描述，不等于Controller避障意图。\n\n"+'；'.join(order)+f'。\n\n{figures}\n')
    contact_ranges=table(['启动s','全部接触附近机器人x范围m'],[[start, '–'.join(fmt(x,2) for x in (min(r['contact']['robot'][0] for r in rows if r['start_time_s']==start and r['contact']),max(r['contact']['robot'][0] for r in rows if r['start_time_s']==start and r['contact'])))] for start in (2,3,4,5,6) if any(r['start_time_s']==start and r['contact'] for r in rows)])
    report=f'''# D01 动态横穿启动时序扫描报告

状态：{'完整30轮' if complete else '中途派生结果，尚未完成30轮'}；有效记录{len(valid)}/{len(rows)}。日期：2026-10-20。本轮独立实验，不覆盖V0.1/V0.2/Release性能对照，也不混入它们的成功率。

## 主要结果

预定30轮：2/3/4/5/6秒 × Release MyDWA/官方DWB ×3。本报告保留全部运行及失败；原4秒D01完全保留，本轮4秒另做新测试，避免与未优化MyDWA旧基准混用。

本次两种Controller都出现高碰撞比例（至少2/3）的**离散测试点为 {high} 秒**；两者均无碰撞的完整测试点为 **{safe} 秒**。这不是已测得连续区间边界；只扫描整数秒点，不能将未测中间点或2秒之前外推成确定窗口。

共{len(no_interaction)}轮按显式规则标记为“无交叉区交互通过”。这指机器人/盒在定义的交叉区未同时占用、未检出持续减速/等待/明显横移；**仍可能有近距离经过和传感器观测，不能称作已证明成功避障**。其余分类计数：`{dict(counts)}`。结论按每格3次描述，不做显著性或全面优劣声明。

## 1. 冻结条件与加载证据

不修改Controller C++业务/采样/预测/评分/碰撞算法、Nav2 YAML/BT/Planner/Costmap、D01地图/机器人起终点/盒尺寸和速度。原D01 start_time=4.0s保持字节不变；rules.scene(4)与原字典相等，其他点仅平移运动关键帧时间及相同轨迹结束时刻。

盒x=1.6m，y=-1.1→+1.1m，0.25m/s，0.5×0.5×0.7m，移动历时8.8s；机器人[0,0,0]→[4,0,0]，任务超时90s。地图、物理模型和原Motion/Observer/Collector/Action/评价器直接复用。trial_entry仅绑定进程内场景加载结果，run/effective_scene.json明确记录实际配置；原configs保留的scenarios.json是实现快照，不能拿其4秒值冒充本轮有效时机。

MyDWA统一使用独立Release安装，实际业务编译参数-O3/-DNDEBUG，库SHA256 `cceb8c0630836b22916da5e839176497acddabc38cb65028fa3283e5daca85ee`；DWB为原系统包，库SHA见evidence/selected_binaries.json。每轮/proc/PID/maps确认正确路径且无第二份Controller库，运行时哈希一致。原runner硬编码的旧安装哈希保存在summary_from_original_runner.json及original_underlay_mydwa_sha256；新summary明确写实际selected_plugin_path/sha256，不用旧哈希假称Release。

两者使用各自既有冻结YAML，全部运行时参数逐字典与原同插件正式记录相同；同一启动、观察、评价条件，不是把两种算法参数强行改成相同。Ground Truth只进入观察/评价，不注入控制；原runner的真值订阅者保护仍启用。未使用性能LD_PRELOAD探针，两者都只增加同样的原生cmd_vel观察器。

实验顺序预先固定并保留sequence：第1轮时机升序，第2轮降序，第3轮4,2,6,3,5，交替Controller先后。每轮完整重启仿真/AMCL/任务/障碍脚本，domain91/master11436、结束清理与2秒空档。没有选择性重跑。Git用户修改/旧记录保护见evidence/before.json和audit_first.json；无commit/push。

## 2. 障碍实际启动验证

不是只看配置：逐轮读取motion请求和Gazebo Ground Truth，要求首条|y−初始y|>0.005m的观测位于配置时刻[-0.05,+0.35]s内；启动前不提前移动、x恒为1.6m；全段已记录实际y落在接收时钟t±0.2s的预定轨迹包络内再加0.01m误差。碰撞提前终止后未执行/未观测的余段不做验证。请求位置逐条与有效轨迹插值核对，原SetEntityState发送/确认/error保留。

GT名义约50Hz，但t来自最新收到的/clock，约0.1s量化；因此典型首动记录为配置后0.1s，不应解释为精确物理延迟。motion_validation.json保留每轮观测、偏差、全轨迹违规列表及样本数。若实际未运动或时序不符，标为测试异常，不能冒充有效避障。本轮各点实际范围列于下表。

## 3. 汇总与碰撞率图

{summary}

成功率只由真实Nav2 SUCCEEDED且原评价器判为success计算；碰撞以物理contact，不用scan阈值或距离到目标代替。成功完成均时只包含成功运行；无成功时为NA。最小净间距是每轮最小值的组内最小/中位数，CSV保留全部样本。

![横穿启动时间与碰撞/成功率及净间距]({output_name}/collision_rate.png)

## 4. 各时机交互行为

“显著变化”是导航后1.5s起的原生cmd_vel_nav连续样本跨度≥0.2s，v≤0.21m/s或|ω|≥0.1rad/s，>0.35s采样空档会重置。分别记录指令减速、转向及实际速度满足对应阈值的时刻。窗口在首次contact前0.15s或机器人包络已离开盒x区后0.3m处截止，排除终点减速、碰撞后急停与取消指令。先检查行为窗口指令覆盖：两端0.2s内有观测、至少3个不同clock样本、最大间隙≤0.35s；缺失/缺段时分类未知，不能把None当作没有减速。该修复经红绿回归，不改变导航成功或物理接触结果。它是行为指标，可能包含常规路径跟踪，不能推断内部避障意图。

分类依据机器人实际旋转包络占用盒x区[1.35,1.85]的时间窗、盒包络占用名义路线y[-0.153,0.153]的时间窗。无同时占用且两窗相隔>0.2s、近似gap>0.05m，并无持续减速/实际减速/区前等待/横移≥0.15m，才称“无交叉区交互通过”。这是近距错时通过也可满足的狭义定义，不表示没有感知或控制影响。初步离线规则曾用0.3m空间阈值；在最终汇总前改用上述“正间距+时间分离”的解释，并保留红绿回归日志，全部30轮统一按同一最终规则处理；导航成功/碰撞阈值和原始记录完全未改。

有区前连续静止≥0.5s且盒先离开者优先标“观测到等待后通过”；显著转向+实际横移≥0.15m+gap<0.6m只标“转向并横移后通过（意图未证实）”；实际/指令减速且时间分离标“减速后错时通过”；证据不足为未知。碰撞中的转向不称成功避让。区间为接收clock采样范围；若末端仍占用则标右截断，不把记录结束冒充实际离开。接近边界时不作精细先后因果判断。

'''+ '\n'.join(paragraphs)+f'''
## 5. 全部逐轮结果

{details}

上表任务持续时间是dispatch至终态，不把失败持续时间叫成功完成时间。全部30轮CSV/JSON另有实际运动开始、真实/命令反应、等待时长、横向偏移、时空占用窗、plugin路径/hash及原始来源SHA。

## 6. 首次碰撞附近位置

{contacts}

接触时刻使用最早contact源stamp，而非Action cancel完成。位置/速度为该stamp附近最近GT接收clock样本，同clock重复时选最后一条；误差约0.1s，位置不是精确同步接触点或接触力位置。机器人二维包络相对base_footprint x=[-.1965,.077]、y=[-.153,.153]，盒0.5m矩形，gap为边界多边形距离且重叠截0，是原保守近似，不冒充真实三维几何；近接触处正小gap不能推翻物理接触。

{contact_ranges}

盒中心x固定1.6m。较晚接触可能发生在机器人基座已越过盒中心之后；基座越线不代表后部轮廓已经退出盒x区，应按外轮廓与实际接触记录判断，而不是中心点。

## 7. 能确定什么、不能确定什么

**实测事实**：每轮原始Action、physical contact、GT、cmd_vel及实际启动验证均可追溯；汇总只描述本次离散时机和各3次重复。若6秒等较晚点安全且机器人先离开交叉区，可以确认“较晚启动改变相遇几何并避免此次接触”，不能据此认定Controller学会动态避障。若较早点仍有碰撞，则当前扫描不支持“更早启动一定更安全”，2秒之前尚未测试。

**有支持但非唯一解释**：高风险点与机器人/横穿盒占用交叉区的时间重叠一致，部分运行虽有转向或减速仍接触。两种Controller使用相同启动时刻，但实际推进/路径不同，遇到的交互状态可能不同；这属于待记录的实验因素，不是强制二者遇到完全同一相对位姿。

**未知**：原始记录没有每周期所有候选轨迹/评分及Controller实际读取Costmap时刻，不能据最终命令确认评分缺陷、预测缺陷或碰撞唯一原因；单靠盒在雷达/Costmap可见也不能确认完整控制链正确。行为分类是离线描述，非意图识别。WSL/AMCL/物理/调度有波动，没有锁定宿主全部负载；每点3次、不做显著性或全面性能优劣结论。整数秒扫描不能精确确定风险边界。

## 8. 验证、复现与下一步

参数化/错误启动时序/缺少实际运动/持续反应/保守分类有回归测试；58项相关Python测试、9个现有Release功能CTest目标（含8个C++ GTest目标）和语法检查通过，见evidence日志。原有3575个文件（源码、参数、旧数据、原安装、Release和DWB）与用户Git差异由最终audit复核。脚本/有效配置/全部原始日志、CSV/JSON和30张交互图在本目录，归档另附。命令见README。

建议最多三项：

1. 在本次整数点的风险转换附近做更细时序扫描，并保留当前各点，仍区分错时通过与主动反应。
2. 对碰撞前逐周期候选/评分及实际地图读取时间补证，区分运动预测、有效候选拒绝和评分选择；本次不改算法。
3. 后续采用明确相遇状态触发或配对进度记录，研究控制推进差异；这会改变实验定义，应单独立项，不混入本轮。
'''
    (HERE/('REPORT_TIMING_SWEEP.md' if tag=='first' else f'REPORT_{tag}.md')).write_text(report)