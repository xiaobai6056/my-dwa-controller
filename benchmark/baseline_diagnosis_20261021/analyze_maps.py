#!/usr/bin/env python3
"""Read-only map diagnosis; writes only baseline_diagnosis outputs/maps*.

python3 -B analyze_maps.py [--trial PATH --label NAME [--sensor-stream FILE]]
Defaults include all FIRST MyDWA repeats, linked t2/t4 diagnostics and D02 r1-r3.
Additional stream must contain costmap rows in observer schema (stamp,t,frame,
origin,resolution,width,height,data). No ABI raw map is accepted as a source.
"""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import temporal

HERE=Path(__file__).resolve().parent
BENCH=HERE.parent
OUT=HERE/'outputs'
FIG=OUT/'figures'

def records(path):
    if not path.exists():return []
    opener=gzip.open if path.suffix=='.gz' else open
    with opener(path,'rt') as stream:return [json.loads(line) for line in stream if line.strip()]

def jsonfile(path,default=None):
    return json.loads(path.read_text()) if path.exists() else default

def rel(value,epoch):return value-epoch if value is not None else None

def write_csv(path,rows):
    if not rows:return
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        writer.writerows({k:json.dumps(v,ensure_ascii=False) if isinstance(v,(list,dict)) else v for k,v in row.items()} for row in rows)

def analyze(trial,label,stream=None):
    meta=jsonfile(trial/'summary.json',{})
    scene=meta.get('scene') or meta.get('arguments',{}).get('scene') or 'unknown'
    events=records(trial/'dynamic_events.jsonl');gt=records(trial/'ground_truth.jsonl')
    contacts=records(trial/'contacts.jsonl');motion=records(trial/'motion.jsonl')
    local=[r for r in records(trial/'localization.jsonl') if r.get('source')=='odom']
    epoch=next((r['t'] for r in events if r.get('event')=='motion_epoch'),meta.get('navigation_start_sim_s'))
    if epoch is None:epoch=motion[0]['t'] if motion else 0.
    initial_y=motion[0]['y'] if motion else (gt[0]['obstacle'][1] if gt else None)
    gt_motion=next((r['t'] for r in gt if initial_y is not None and abs(r['obstacle'][1]-initial_y)>.005),None)
    command_motion=next((r['t'] for r in motion if initial_y is not None and abs(r['y']-initial_y)>.005),None)
    contact=min((r['stamp'] for r in contacts),default=None)
    roi=[1.15,-1.4,2.05,1.4] if scene=='D01' else ([1.55,-.4,2.45,1.4] if scene=='D02' else None)
    source=stream or trial/'sensors.jsonl.gz'
    raw=[r for r in records(source) if r.get('type')=='costmap']
    history=temporal.History(roi=roi)
    rows=[];rejects=[]
    for n,r in enumerate(raw):
        try:rows.append(history.feed(r))
        except (ValueError,KeyError,TypeError) as error:
            rejects.append(dict(index=n,stamp=r.get('stamp'),reason=str(error)))
    unique=[r for r in rows if r['status']!='duplicate_stamp']
    pairs=[r for r in unique if r['status']=='pair']
    motion_pair=next((r for r in pairs if r['motion_candidate']),None)
    direction=next((r for r in pairs if r['sustained_direction']),None)
    axis=next((r for r in pairs if r['axis_motion_candidate']),None)
    axis_direction=next((r for r in pairs if r['sustained_axis_direction']),None)
    # This marker is purely a comparison point, NEVER a braking feasibility claim.
    robot_marker=next((r['t'] for r in gt if r['robot'][0]>=1.15),None)
    key_times={'gt_first_motion_receipt':gt_motion,'first_contact_source_stamp':contact,'gt_robot_x_ge_1_15_receipt_marker_only':robot_marker}
    for event in events:
        if event.get('event')=='corridor_clear':key_times['corridor_clear_receipt']=event['t']
    linked=jsonfile(BENCH/'dynamic/avoidability/outputs/decision_summary.json',[])
    decision=next((d for d in linked if d['run']==label),None)
    if decision:
        safe=decision.get('latest_observed_conditional_safe_cycle')
        if safe:key_times['latest_observed_conditional_safe_cycle']=epoch+safe['t']
        slow=decision.get('first_successful_plugin_output_le_0_21')
        if slow:key_times['first_successful_plugin_output_le_0_21']=epoch+slow['t']
        for example in decision.get('examples',[]):key_times[f"plugin_example_{example['index']}"]=epoch+example['t']
    stationary_pairs=[]
    for pair in pairs:
        if not gt:continue
        a=min(gt,key=lambda r:abs(r['t']-pair['previous_stamp']))
        b=min(gt,key=lambda r:abs(r['t']-pair['stamp']))
        truth_delta=[b['obstacle'][i]-a['obstacle'][i] for i in (0,1)]
        stationary=math.hypot(*truth_delta)<.005
        stationary_pairs.append(dict(previous_stamp=pair['previous_stamp'],stamp=pair['stamp'],
                                     gt_receipt_samples=[a['t'],b['t']],gt_delta_xy=truth_delta,
                                     approximately_stationary=stationary,raw_candidate=pair['motion_candidate'],
                                     positive_y_candidate=pair['axis_motion_candidate'],
                                     positive_y_sustained=pair['sustained_axis_direction']))
    keys=[]
    for name,t in key_times.items():
        if t is None:continue
        result=temporal.availability(rows,t)
        result.update(label=label,key=name,relative_s=t-epoch)
        keys.append(result)
    dts=[r['dt_s'] for r in pairs]
    align=None
    if gt and local:
        g=gt[0];o=min(local,key=lambda r:abs(r['receipt_sim_s']-g['t']))
        align=dict(gt_receipt=g['t'],odom_receipt=o['receipt_sim_s'],odom_source_stamp=o['source_stamp_s'],
                   world_minus_odom_xy=[g['robot'][i]-o[k] for i,k in enumerate(('x','y'))],
                   world_minus_odom_yaw=g['robot'][2]-o['yaw'],
                   note='Initial numerical alignment only; receipt times are quantized and no exact world-to-odom transform is established.')
    safe_key=next((r for r in keys if r['key']=='latest_observed_conditional_safe_cycle'),{})
    contact_key=next((r for r in keys if r['key']=='first_contact_source_stamp'),{})
    summary=dict(label=label,scene=scene,trial=str(trial),source=str(source),
                 source_sha256=hashlib.sha256(source.read_bytes()).hexdigest() if source.exists() else None,
                 epoch_s=epoch,start_condition_s=meta.get('effective_start_time_s'),outcome=meta.get('stop_reason') or meta.get('action_result'),
                 raw_maps=len(raw),unique_maps=len(unique),duplicate_stamps=sum(r['status']=='duplicate_stamp' for r in rows),
                 repeated_content=sum(r.get('repeated_content',False) for r in unique),rejected_maps=len(rejects),
                 frame=unique[0]['frame'] if unique else None,roi=roi,
                 interval_min_s=min(dts) if dts else None,interval_median_s=statistics.median(dts) if dts else None,
                 interval_max_s=max(dts) if dts else None,gaps=sum(r['gap'] for r in pairs),
                 first_stamp_relative_s=rel(unique[0]['stamp'],epoch) if unique else None,
                 final_stamp_relative_s=rel(unique[-1]['stamp'],epoch) if unique else None,
                 unknown_cells_total=sum(r['unknown_n'] for r in unique),
                 coverage_entered_cells=sum(r['entered_coverage_n'] for r in pairs),coverage_left_cells=sum(r['left_coverage_n'] for r in pairs),
                 known_to_unknown_cells=sum(r['known_to_unknown_n'] for r in pairs),unknown_to_known_cells=sum(r['unknown_to_known_n'] for r in pairs),
                 motion_candidates=sum(r['motion_candidate'] for r in pairs),direction_pairs=sum(r['sustained_direction'] for r in pairs),
                 first_motion_pair_source_relative_s=rel(motion_pair['stamp'],epoch) if motion_pair else None,
                 first_motion_pair_receipt_relative_s=rel(motion_pair['receipt'],epoch) if motion_pair else None,
                 first_motion_pair_support_relative_s=[s-epoch for s in motion_pair['support_stamps']] if motion_pair else None,
                 first_motion_pair_delta_xy=motion_pair['centroid_displacement_xy'] if motion_pair else None,
                 first_direction_source_relative_s=rel(direction['stamp'],epoch) if direction else None,
                 first_direction_receipt_relative_s=rel(direction['receipt'],epoch) if direction else None,
                 first_direction_support_relative_s=[s-epoch for s in direction['support_stamps']] if direction else None,
                 first_direction_xy=direction['direction_xy'] if direction else None,
                 first_positive_y_pair_relative_s=rel(axis['stamp'],epoch) if axis else None,
                 first_positive_y_sustained_relative_s=rel(axis_direction['stamp'],epoch) if axis_direction else None,
                 first_positive_y_support_relative_s=[t-epoch for t in axis['axis_support_stamps']] if axis else None,
                 first_positive_y_sustained_support_relative_s=[t-epoch for t in axis_direction['axis_support_stamps']] if axis_direction else None,
                 approximately_stationary_pair_count=sum(r['approximately_stationary'] for r in stationary_pairs),
                 raw_candidates_in_stationary_pairs=sum(r['approximately_stationary'] and r['raw_candidate'] for r in stationary_pairs),
                 positive_y_candidates_in_stationary_pairs=sum(r['approximately_stationary'] and r['positive_y_candidate'] for r in stationary_pairs),
                 positive_y_sustained_in_stationary_pairs=sum(r['approximately_stationary'] and r['positive_y_sustained'] for r in stationary_pairs),
                 raw_candidates_before_gt_motion=sum(r['motion_candidate'] and gt_motion is not None and r['stamp']<gt_motion-.1-1e-9 for r in pairs),
                 raw_sustained_before_gt_motion=sum(r['sustained_direction'] and gt_motion is not None and r['stamp']<gt_motion-.1-1e-9 for r in pairs),
                 positive_y_candidates_before_gt_motion=sum(r['axis_motion_candidate'] and gt_motion is not None and r['stamp']<gt_motion-.1-1e-9 for r in pairs),
                 safe_positive_y_pair_lead_s=safe_key.get('axis_motion_pair_lead_s'),
                 safe_positive_y_sustained_lead_s=safe_key.get('axis_direction_lead_s'),
                 at_safe_positive_y_pairs=safe_key.get('available_axis_motion_pairs'),
                 at_safe_positive_y_sustained_pairs=safe_key.get('available_axis_direction_pairs'),
                 observed_gt_motion_receipt_relative_s=rel(gt_motion,epoch),motion_command_receipt_relative_s=rel(command_motion,epoch),
                 contact_source_relative_s=rel(contact,epoch),
                 at_safe_available_maps=safe_key.get('available_unique_maps'),at_safe_motion_pairs=safe_key.get('available_motion_pairs'),
                 at_safe_direction_pairs=safe_key.get('available_sustained_direction'),safe_motion_pair_lead_s=safe_key.get('motion_pair_lead_s'),
                 safe_direction_lead_s=safe_key.get('sustained_direction_lead_s'),
                 contact_motion_pair_lead_s=contact_key.get('motion_pair_lead_s'),contact_direction_lead_s=contact_key.get('sustained_direction_lead_s'),
                 initial_alignment=align)
    return dict(summary=summary,rows=rows,rejections=rejects,keys=keys,
                motion_schedule=motion,gt_obstacle_receipt=[dict(t=r['t'],xy=r['obstacle'][:2]) for r in gt],
                contacts=contacts,events=events,post_extraction_gt_stationary_audit=stationary_pairs)

def plots(results):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for item in results:
        summary=item['summary'];epoch=summary['epoch_s'];rows=[r for r in item['rows'] if r['status']!='duplicate_stamp']
        fig,axs=plt.subplots(3,1,figsize=(11,8),sharex=True,layout='constrained')
        for r in rows:
            t=r['stamp']-epoch
            if r['roi_lethal_xy']:axs[0].scatter([t]*len(r['roi_lethal_xy']),[p[1] for p in r['roi_lethal_xy']],s=7,c='black',alpha=.35)
            if r['status']=='pair':
                if r['added_xy']:axs[1].scatter([t]*len(r['added_xy']),[p[1] for p in r['added_xy']],s=18,c='#008866',marker='+')
                if r['removed_xy']:axs[1].scatter([t]*len(r['removed_xy']),[p[1] for p in r['removed_xy']],s=16,c='#cc5533',marker='x')
                axs[2].plot(t,r['added_n'],'+',color='#008866');axs[2].plot(t,-r['removed_n'],'x',color='#cc5533')
        schedule=item['motion_schedule']
        if schedule:axs[0].plot([r['t']-epoch for r in schedule],[r['y'] for r in schedule],'--',color='#3366aa',lw=1,label='scenario commanded y (world; approximate odom alignment)')
        for ax in axs:
            for field,color in [('first_positive_y_pair_relative_s','#008866'),('first_positive_y_sustained_relative_s','#7744bb'),('contact_source_relative_s','#cc2222')]:
                value=summary[field]
                if value is not None:ax.axvline(value,color=color,lw=1,alpha=.7)
            ax.grid(alpha=.2)
        for key in item['keys']:
            if key['key']=='latest_observed_conditional_safe_cycle':
                for ax in axs:ax.axvline(key['relative_s'],c='#d28c00',lw=2)
        axs[0].set_ylabel('Lethal cell center y [odom m]')
        if schedule:axs[0].legend(fontsize=7,loc='best')
        axs[1].set_ylabel('Known overlap changes y [m]')
        axs[2].set_ylabel('Added (+) / removed (-) cells');axs[2].set_xlabel('Source map stamp minus motion epoch [s]')
        fig.suptitle(summary['label']+' | lethal=100; green add / orange remove; purple sustained +y; red contact')
        fig.savefig(FIG/f"maps_{summary['label']}.png",dpi=150);plt.close(fig)
    first=[i for i in results if i['summary']['label'].startswith('first_') and i['summary']['start_condition_s'] is not None]
    if first:
        fig,ax=plt.subplots(figsize=(11,7),layout='constrained')
        for y,item in enumerate(sorted(first,key=lambda d:(d['summary']['start_condition_s'],d['summary']['label']))):
            s=item['summary'];epoch=s['epoch_s'];times=[r['stamp']-epoch for r in item['rows'] if r['status']!='duplicate_stamp']
            ax.plot(times,[y]*len(times),'.',c='#999999',ms=4)
            for field,marker,color in [('observed_gt_motion_receipt_relative_s','|','#3366aa'),('first_positive_y_pair_relative_s','>','#008866'),('first_positive_y_sustained_relative_s','D','#7744bb'),('contact_source_relative_s','x','#cc2222')]:
                if s[field] is not None:ax.plot(s[field],y,marker,c=color,ms=7)
        ordered=sorted(first,key=lambda d:(d['summary']['start_condition_s'],d['summary']['label']))
        ax.set_yticks(range(len(ordered)),[i['summary']['label'] for i in ordered],fontsize=7)
        ax.set_xlabel('Seconds relative to motion epoch');ax.grid(axis='x',alpha=.2)
        ax.set_title('FIRST all repeats: blue observed motion | green first +y pair | purple sustained +y | red contact')
        fig.savefig(FIG/'maps_first_all_repeats.png',dpi=180);plt.close(fig)

def fmt(value):
    if value is None:return '—'
    return f'{value:.3f}' if isinstance(value,float) else str(value)

def report(results):
    summaries=[i['summary'] for i in results]
    lines=['# Costmap 时序离线诊断','',
      '## 事实与方法','',
      '主时序流只取 sensors.jsonl.gz 的实际 OccupancyGrid header stamp，不使用没有真实 map 时间戳的 ABI decision raw map。stamp 是地图源时间；t 是 Observer 接收时看到的 /clock，约 0.1 s 量化。GT ModelStates 没有源时间戳；contacts 的 stamp 为接触源时间。',
      '',
      '全部输入地图为 odom。按首张地图格心定义公共 odom 网格；检查分辨率一致、origin 位移为整数格（容许 float32 分辨率引起的 1e-5 格误差），仅比较相邻地图共同覆盖且两边已知的单元。lethal 只取 100，99 为 inscribed 单列，不把膨胀、unknown 转换或滚动新覆盖误报为 lethal 移动；拒绝冲突重复时间戳、倒序时间戳、frame/resolution 改变、非整数位移和无共同覆盖。相同时间戳只算一次；相同内容的新时间戳单列，不自动作为移动证据；间隔 >1 s 标为 gap 且不跨 gap 推断运动。',
      '',
      '固定 ROI 是场景先验（其他场景用全图共同覆盖）：D01 [x=1.15..2.05, y=-1.4..1.4] m，D02 [x=1.55..2.45, y=-0.4..1.4] m。它不跟随 GT、obstacle_grid_pose 或 robot 位置。初始 world/odom 数值近似对齐仅作背景核验，并没有建立精确 world→odom TF。ROI 中每对共同已知覆盖至少 2 个 lethal 添加和 2 个移除、lethal 质心位移 ≥0.02 m 才记 motion candidate；连续两对方向余弦 ≥0.5 才记 sustained direction（至少 3 张不同源时间戳地图）。这是表面占据变化及方向证据，并非对象速度的无偏估计。',
      '',
      '坐标、添加/移除、质心、支持时间戳、unknown 和 coverage 数量完整保存在 maps_details.json；maps_pairs.csv 为逐对证据。每个 key 的可用性要求 stamp≤key 且 receipt t+0.1 s≤key；lead 按 max(stamp,t+0.1) 计算。这只表示 Observer 侧保守可用，不能证明控制器读取了这些历史地图。最新 conditional safe 周期直接来自原 avoidability/outputs/decision_summary.json；其安全定义的条件性与几何近似继续保留。GT x≥1.15 只作统一比较标记，不等于可安全制动时刻。',
      '', '## 结果：保留全部重复','',
      '|Run|移动开始 GT接收(s)|地图数|地图间隔中位(s)|首移动对(s)|首持续方向(s)|接触源(s)|接触前移动对lead(s)|接触前方向lead(s)|',
      '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for s in summaries:
        lines.append('|'+ '|'.join(fmt(s[k]) for k in ['label','observed_gt_motion_receipt_relative_s','unique_maps','interval_median_s','first_motion_pair_source_relative_s','first_direction_source_relative_s','contact_source_relative_s','contact_motion_pair_lead_s','contact_direction_lead_s'])+'|')
    lines+=['','|Linked run|safe key 前地图数|motion pair数|sustained pair数|首移动对lead(s)|首持续方向lead(s)|','|---|---:|---:|---:|---:|---:|']
    for s in summaries:
        if s['at_safe_available_maps'] is not None:
            lines.append('|'+ '|'.join(fmt(s[k]) for k in ['label','at_safe_available_maps','at_safe_motion_pairs','at_safe_direction_pairs','safe_motion_pair_lead_s','safe_direction_lead_s'])+'|')
    for item in results:
        s=item['summary']
        if s['at_safe_available_maps'] is None:continue
        safe=next(k for k in item['keys'] if k['key']=='latest_observed_conditional_safe_cycle')
        lines+=['',f"{s['label']}：conditional safe key={safe['relative_s']:.3f} s，此前保守可用地图 {safe['available_unique_maps']} 张，最后源地图相对时刻={safe['last_source_stamp']-s['epoch_s']:.3f} s；首 +y 对={fmt(s['first_positive_y_pair_relative_s'])} s，首持续 +y={fmt(s['first_positive_y_sustained_relative_s'])} s。safe key 前 +y 对={safe['available_axis_motion_pairs']}，持续 +y 对={safe['available_axis_direction_pairs']}。"]
    lines+=['','## 结果：方向约束与静态阶段反例' ,'',
      '下表 +y 单对要求原 motion candidate 之外，dy 至少一个分辨率单元（约 0.05 m），且 dy≥|dx|；+y 持续要求连续两对满足条件。该阈值来自网格分辨率，不用 GT 调整；D01 对应朝通道横穿方向，D02 对应释放离开通道方向。D02 进入阻塞的 −y 运动仍见原逐对质心与增删记录。下表“静态阶段原候选”只在提取后用 GT 首次移动接收时刻减 0.1 s 做评估，不进入特征筛选。',
      '', '|Run|首 +y 对(s)|首持续 +y(s)|静态阶段原候选数|静态阶段原持续数|safe前 +y对数|safe前持续 +y数|safe前首 +y lead(s)|',
      '|---|---:|---:|---:|---:|---:|---:|---:|']
    for s in summaries:
        lines.append('|'+ '|'.join(fmt(s[k]) for k in ['label','first_positive_y_pair_relative_s','first_positive_y_sustained_relative_s','raw_candidates_before_gt_motion','raw_sustained_before_gt_motion','at_safe_positive_y_pairs','at_safe_positive_y_sustained_pairs','safe_positive_y_pair_lead_s'])+'|')
    lines+=['','|Run|GT近似静止的地图对数|静止时原候选|静止时 +y候选|静止时 +y持续|','|---|---:|---:|---:|---:|']
    for s in summaries:
        lines.append('|'+ '|'.join(fmt(s[k]) for k in ['label','approximately_stationary_pair_count','raw_candidates_in_stationary_pairs','positive_y_candidates_in_stationary_pairs','positive_y_sustained_in_stationary_pairs'])+'|')
    lines+=['',
      '这张审计表在特征提取完成后，按各地图 stamp 最近的 GT 接收样本比较障碍位置；端点位移 <0.005 m 记近似静止，时间量化和源/接收时差仍存在。D02 阻塞中停住的阶段也纳入，避免把静态墙体/可见面变化误当作释放。',
      '',
      '结果包含明确反例：障碍尚静止时，机器人运动改变可见面与 lethal 标记，已知共同覆盖内仍会产生添加/移除及质心变化。因此原“首移动对/持续方向”字段只应读作占据表面变化及原始方向候选，不等同已确认障碍运动。方向约束表单独给出更保守的场景轴证据，仍不提供对象跟踪或因果证明。',
      '',  '## 新采集对照与采样限制','',
      '新增采集的首 +y 对及持续方向见上表。若某次未满足连续两对的保守持续方向规则，不能据此说地图完全没有运动信息。新/旧采集包含不同运行时机、发布采样相位和探针/录包负载，现有差异不能证明探针改变或没有改变导航行为；还应结合独立计算耗时和循环日志。',
      '',
      'S04/S05 使用 bag 导出的实际 OccupancyGrid 源时间戳流，原 trial 没有动态 Observer sensor 文件。静态场景没有动态障碍 GT，因此不提供静止真值审计或运动开始值，完整公共图的变化只作静态场景对照。bag 开始记录晚于 dispatch，首张地图之前的时间窗缺失，不得用它证明更早没有观测。若 bag 提供 origin_yaw 则要求其为 0；原 Observer 不保存 orientation 的限制仍存在。精确 map-stamp TF 的可用性与近似未打源时间戳 GT 投影由独立 bag_export 输出报告，本文格心差分不使用这些真值投影。',
      '', '## 解释与假设边界','', 

      '地图上出现向通道方向持续变化，且在某个 key 前留有时间，只支持“历史观测包含可能可用于动态估计的信息”。不能单凭离线差分证明当时算法能够可靠识别、预测并躲避，也不能把碰撞唯一归因于缺少历史。可见面变化、遮挡、栅格量化、传感/融合延迟均可影响质心，尤其 D02 墙体及窄通道 ROI。',
      '',
      'Observer 的 obstacle_grid_pose 用接收时最新 TF，将最新 GT 相对机器人位置投影进地图；不是 map stamp 时刻的精确真值。本文特征完全不读取该字段，不用它作因果或精确误差证明。origin orientation 未序列化，分析按现有 Nav2 axis-aligned local costmap 假设。初始对齐诊断列于 maps_summary.json，不能替代精确时间同步 TF。',
      '',
      '0.6 s 左右地图发布间隔明显粗于约 20 Hz 控制循环；无地图变化不代表真实障碍静止，首次检测时刻也不等于首次可被任何传感器感知。GT 移动开始按 |y−initial_y|>0.005 m 的首个接收样本取值，误差包含 /clock 量化。报告中的 lead 已加入 0.1 s 接收余量，仍不是完整端到端延迟上界。',
      '',
      f"输入共 {len(summaries)} runs、{sum(s['raw_maps'] for s in summaries)} 张原地图；拒绝 {sum(s['rejected_maps'] for s in summaries)} 张、重复源时间戳 {sum(s['duplicate_stamps'] for s in summaries)} 张、gap {sum(s['gaps'] for s in summaries)} 对。诊断测试覆盖静态滚动、膨胀、unknown、重复与冲突时间戳、frame/resolution/位移错误、coverage、gap、三图持续方向和 key 前可用性。",
      '', '## 复现','', '`python3 -B -m unittest -v test_temporal`；`python3 -B analyze_maps.py`。增加新 run：`python3 -B analyze_maps.py --trial /absolute/trial --label new_name`；如 bag 已导出同 schema 地图流，在对应 `--trial` 后加 `--sensor-stream /absolute/stream.jsonl.gz`；它只覆盖紧前的 trial。脚本仅写本目录 outputs/maps* 和 outputs/figures/maps*。']
    (OUT/'maps_findings.md').write_text('\n'.join(lines)+'\n')

class StreamForTrial(argparse.Action):
    def __call__(self,parser,namespace,value,option_string=None):
        if not namespace.trial:parser.error('--sensor-stream must follow its --trial')
        mapping=dict(namespace.sensor_stream or {})
        index=len(namespace.trial)-1
        if index in mapping:parser.error('only one --sensor-stream is allowed per --trial')
        mapping[index]=value
        namespace.sensor_stream=mapping

def parse_cli(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',action='append',type=Path,default=[])
    parser.add_argument('--label',action='append',default=[])
    parser.add_argument('--sensor-stream',action=StreamForTrial,type=Path,default={})
    args=parser.parse_args(argv)
    if len(args.trial)!=len(args.label):parser.error('--trial and --label must have matching counts')
    for label in args.label:
        if not label or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in label):parser.error('label must contain only letters, digits, _ or -')
    return args

def main():
    args=parse_cli()
    cases=[(p/'trial',p.name,None) for p in sorted((BENCH/'dynamic/timing_sweep/runs').glob('first_*_mydwa_*'))]
    cases += [(BENCH/'dynamic/avoidability/runs'/n/'trial',n,None) for n in ('diagnostic_t2_linked','diagnostic_t4_linked')]
    cases += [(BENCH/'results/dynamic/formal_v0_1_20261020/D02/mydwa'/f'trial_{i:03d}',f'D02_mydwa_{i:03d}',None) for i in (1,2,3)]
    for i,(path,label) in enumerate(zip(args.trial,args.label)):
        cases.append((path,label,args.sensor_stream.get(i)))
    OUT.mkdir(exist_ok=True);FIG.mkdir(exist_ok=True)
    results=[analyze(*case) for case in cases]
    (OUT/'maps_details.json').write_text(json.dumps(results,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (OUT/'maps_summary.json').write_text(json.dumps([r['summary'] for r in results],ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    write_csv(OUT/'maps_summary.csv',[r['summary'] for r in results])
    write_csv(OUT/'maps_pairs.csv',[dict(label=r['summary']['label'],**pair) for r in results for pair in r['rows'] if pair['status']=='pair'])
    write_csv(OUT/'maps_key_availability.csv',[key for r in results for key in r['keys']])
    plots(results);report(results)
    for r in results:
        s=r['summary'];print(s['label'],'maps',s['unique_maps'],'pair',fmt(s['first_motion_pair_source_relative_s']),'direction',fmt(s['first_direction_source_relative_s']),'positive_y',fmt(s['first_positive_y_pair_relative_s']),'sustained_y',fmt(s['first_positive_y_sustained_relative_s']),'safe_y_lead',fmt(s['safe_positive_y_pair_lead_s']),'reject',s['rejected_maps'])

if __name__=='__main__':main()
