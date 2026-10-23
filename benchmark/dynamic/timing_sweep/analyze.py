#!/usr/bin/env python3
"""Offline D01 timing scan: action/contact metrics plus conservative behavior evidence."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
from rules import scene,selection,mapping_matches,completed_trial,verify_motion,response,classify,command_coverage,ROOT
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'benchmark'))
from dynamic.evaluation import calculate,box_polygon,ROBOT_BOUNDS

LABELS={
 'pass_behavior_unknown_command_evidence':'通过：指令观测覆盖不足，行为未知',
 'collision_behavior_unknown_command_evidence':'碰撞：指令观测覆盖不足，行为未知',
 'no_interaction_robot_first':'无交叉区交互通过：机器人先通过',
 'no_interaction_obstacle_first':'无交叉区交互通过：盒先通过',
 'observed_wait_then_pass':'观测到等待后通过（意图未证实）',
 'turn_with_lateral_displacement_pass':'转向并横移后通过（意图未证实）',
 'slowed_separated_pass':'减速后错时通过（机制未知）',
 'collision_with_turn':'碰撞：有持续转向指令',
 'collision_without_sustained_turn':'碰撞：未检出持续转向指令',
 'interaction_pass_unknown':'通过：交互机制未知',
 'interaction_unknown_incomplete_zone_evidence':'通过：交叉区证据不完整',
 'navigation_failure_unknown':'导航失败：机制未知'}

def load(p):return json.loads(p.read_text())
def jsonl(p):return [json.loads(l) for l in p.read_text().splitlines() if l]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,value):p.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def fmt(v,n=1):return 'NA' if v is None else f'{v:.{n}f}'
def table(headers,rows):return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,row))+' |' for row in rows])

def windows(observations,predicate,epoch):
    result=[];start=last=None
    for r in observations:
        if predicate(r):
            if start is None:start=r['t']
            last=r['t']
        elif start is not None:
            result.append([start-epoch,last-epoch]);start=last=None
    if start is not None:result.append([start-epoch,last-epoch])
    return result

def robot_x_bounds(row):
    points=box_polygon(*row['robot'],ROBOT_BOUNDS)
    return min(p[0] for p in points),max(p[0] for p in points)

def analyze_trial(planned):
    directory=HERE/'runs'/planned['run']
    own=load(directory/'manifest.json');summary=load(directory/'trial/summary.json')
    value=scene(planned['start_time_s'])
    assert summary['effective_scene']==value==load(directory/'effective_scene.json')
    prefix,plugin=selection(planned['controller'])
    assert own['runtime_mapping_verified'] and mapping_matches((directory/'plugin_maps.txt').read_text(),plugin)
    assert own['runtime_plugin_sha256']==own['current_plugin_sha256']==summary['selected_plugin_sha256']==sha(plugin)
    expected=load(ROOT/f'benchmark/results/dynamic/formal_v0_1_20261020/D01/{planned["controller"]}/trial_001/summary.json')['runtime_parameters']
    assert summary['runtime_parameters']==expected,'frozen runtime parameters changed'
    epoch=summary['navigation_start_sim_s'];end=summary['navigation_end_sim_s']
    gt=jsonl(directory/'trial/ground_truth.jsonl')
    motion=jsonl(directory/'trial/motion.jsonl')
    validation=verify_motion(gt,motion,value,epoch)
    assert motion,'no motion requests recorded'
    assert validation==load(directory/'motion_validation.json')
    metrics=calculate(directory/'trial')
    contacts=jsonl(directory/'trial/contacts.jsonl')
    contact=min(contacts,key=lambda row:row['stamp']) if contacts else None
    contact_time=contact['stamp'] if contact else None
    physical_clear=next((r['t'] for r in gt if robot_x_bounds(r)[0]>1.85+.3),end)
    before=min(end,physical_clear,contact_time-.15 if contact else end)
    native=jsonl(directory/'native_commands.jsonl') if (directory/'native_commands.jsonl').exists() else []
    commands=[(r['sim'],r['vx'],r['wz']) for r in native if r['topic']=='/cmd_vel_nav' and r['sim'] is not None]
    actual=[(r['t'],r['vx'],r['wz']) for r in gt]
    after=epoch+1.5
    coverage=command_coverage(commands,after,before)
    brake=response(commands,after,before,'brake');turn=response(commands,after,before,'turn')
    actual_brake=response(actual,after,before,'brake');actual_turn=response(actual,after,before,'turn')
    robot_windows=windows(gt,lambda r:robot_x_bounds(r)[0]<=1.85 and robot_x_bounds(r)[1]>=1.35,epoch)
    obstacle_windows=windows(gt,lambda r:abs(r['obstacle'][1])<=.403,epoch)
    waits=windows(gt,lambda r:after<=r['t']<before and robot_x_bounds(r)[1]<1.35+.03 and abs(r['vx'])<.02 and abs(r['wz'])<.05,epoch)
    waiting=max((b-a for a,b in waits),default=0.)
    excursion=max((abs(r['robot'][1]) for r in gt if after<=r['t']<before),default=0.)
    relative=lambda t:None if t is None else t-epoch
    features=dict(command_evidence_complete=coverage['complete'],success=metrics['success'],collision=metrics['collision'],brake_s=relative(brake),turn_s=relative(turn),
        actual_brake_s=relative(actual_brake),actual_turn_s=relative(actual_turn),waiting_before_zone_s=waiting,
        lateral_excursion_m=excursion,robot_windows=robot_windows,obstacle_windows=obstacle_windows,
        min_gap_m=metrics['min_gap_approx_m'],
        robot_window_right_censored=bool(robot_windows and robot_x_bounds(gt[-1])[0]<=1.85 and robot_x_bounds(gt[-1])[1]>=1.35),
        obstacle_window_right_censored=bool(obstacle_windows and abs(gt[-1]['obstacle'][1])<=.403))
    category=classify(features)
    near_contact=None
    if contact:
        _,near=min(enumerate(gt),key=lambda item:(abs(item[1]['t']-contact_time),-item[0]))
        near_contact=dict(source_stamp=contact_time,relative_s=contact_time-epoch,nearby_gt_receipt_relative_s=near['t']-epoch,
            robot=near['robot'],obstacle=near['obstacle'],actual_vx=near['vx'],actual_wz=near['wz'],pair=contact['pair'],
            note='Nearby receipt-clock GT, not exact synchronized contact pose; ~0.1s clock bins.')
    valid=bool(completed_trial(own['runner_returncode'],summary) and own['cleanup_returncode']==0 and validation['valid'] and planned.get('returncode')==0)
    row=dict(run=planned['run'],order=planned['order'],controller=planned['controller'],repeat=planned['repeat'],start_time_s=planned['start_time_s'],
        valid=valid,metrics=metrics,motion_validation=validation,features=features,category=category,category_label=LABELS[category],
        contact=near_contact,command_observation_coverage=coverage,behavior_window_relative_s=[1.5,before-epoch],runtime_plugin_path=str(plugin),runtime_plugin_sha256=sha(plugin),
        runtime_parameters_equal_original=True,raw_summary_sha256=sha(directory/'trial/summary.json'),
        sources_sha256={str(p.relative_to(directory)):sha(p) for p in [directory/'manifest.json',directory/'trial/summary.json',directory/'trial/summary_from_original_runner.json',directory/'trial/ground_truth.jsonl',directory/'trial/contacts.jsonl',directory/'trial/motion.jsonl',directory/'native_commands.jsonl',directory/'effective_scene.json',directory/'plugin_maps.txt'] if p.is_file()})
    return row,gt,commands

def aggregate(rows):
    groups=[]
    for start in (2,3,4,5,6):
        for controller in ('mydwa','dwb'):
            group=[r for r in rows if r['start_time_s']==start and r['controller']==controller]
            valid=[r for r in group if r['valid']]
            successes=[r for r in valid if r['metrics']['success']]
            gaps=[r['metrics']['min_gap_approx_m'] for r in valid if r['metrics']['min_gap_approx_m'] is not None]
            durations=[r['metrics']['successful_completion_sim_s'] for r in successes]
            groups.append(dict(start_time_s=start,controller=controller,completed=len(group),valid=len(valid),successes=len(successes),
                collisions=sum(r['metrics']['collision'] for r in valid),
                collision_rate=sum(r['metrics']['collision'] for r in valid)/len(valid) if valid else None,
                success_rate=len(successes)/len(valid) if valid else None,
                success_mean_sim_s=statistics.mean(durations) if durations else None,
                success_duration_range_s=[min(durations),max(durations)] if durations else None,
                min_gap_approx_m=min(gaps) if gaps else None,median_min_gap_approx_m=statistics.median(gaps) if gaps else None,
                actual_start_range_s=[min(r['motion_validation']['observed_first_gt_motion_s'] for r in valid),max(r['motion_validation']['observed_first_gt_motion_s'] for r in valid)] if valid else None,
                categories=dict(Counter(r['category'] for r in valid)),
                failures=[dict(run=r['run'],termination=r['metrics']['termination'],action=r['metrics']['action_result'],valid=r['valid']) for r in group if not r['valid'] or not r['metrics']['success']]))
    return groups

def trial_plot(row,gt,commands,destination):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    summary=load(HERE/'runs'/row['run']/'trial/summary.json');epoch=summary['navigation_start_sim_s']
    times=[r['t']-epoch for r in gt]
    fig,axes=plt.subplots(2,2,figsize=(11,7))
    ax=axes[0,0]
    ax.plot([r['robot'][0] for r in gt],[r['robot'][1] for r in gt],label='robot GT')
    ax.plot([r['obstacle'][0] for r in gt],[r['obstacle'][1] for r in gt],label='obstacle GT')
    if row['contact']:
        for key,bounds in [('robot',ROBOT_BOUNDS),('obstacle',[-.25,-.25,.25,.25])]:
            p=box_polygon(*row['contact'][key],bounds)
            ax.add_patch(Polygon(p,fill=False,color='crimson',lw=1.3))
        ax.scatter(*row['contact']['robot'][:2],color='crimson',marker='x',label='near first contact')
    ax.axvspan(1.35,1.85,alpha=.07,color='gray');ax.set(xlabel='world x m',ylabel='world y m',aspect='equal');ax.legend(fontsize=8)
    for axis,index,key,label in [(axes[0,1],1,'vx','linear m/s'),(axes[1,0],2,'wz','angular rad/s')]:
        axis.step([r[0]-epoch for r in commands],[r[index] for r in commands],where='post',label='/cmd_vel_nav native receipt')
        axis.plot(times,[r[key] for r in gt],label='GT actual',alpha=.8);axis.set(xlabel='simulation s after dispatch',ylabel=label);axis.legend(fontsize=8)
    axes[1,1].plot(times,[r['gap_approx_m'] for r in gt],label='approximate clearance')
    axes[1,1].set(xlabel='simulation s after dispatch',ylabel='approximate footprint gap m');axes[1,1].legend(fontsize=8)
    for axis in (axes[0,1],axes[1,0],axes[1,1]):
        axis.axvline(row['start_time_s'],color='gray',ls=':',label='configured motion start')
        if row['contact']:axis.axvline(row['contact']['relative_s'],color='crimson',ls='--')
        for key,color in [('brake_s','#168475'),('turn_s','#b48213')]:
            if row['features'][key] is not None:axis.axvline(row['features'][key],color=color,ls=':')
    for axis in axes.flat:axis.grid(alpha=.4)
    fig.suptitle(f'D01 / start {row["start_time_s"]}s / {row["controller"]} / repeat {row["repeat"]} / {row["category"]}',fontsize=10)
    fig.text(.01,.01,'Sources: raw GT/contact source stamp/native cmd_vel_nav. GT receipt clock ~0.1s; footprint gap and contact pose approximate.\nVertical lines: grey=start, green=brake criterion, ochre=turn criterion, red dashed=first physical contact.',fontsize=7)
    fig.tight_layout(rect=[0,.055,1,.96]);fig.savefig(destination/(row['run']+'.png'),dpi=120);plt.close(fig)

def plots(rows,groups,raw,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    destination=output/'figures';destination.mkdir(parents=True,exist_ok=True)
    fig,axes=plt.subplots(1,3,figsize=(12,4))
    for controller in ('mydwa','dwb'):
        group=[g for g in groups if g['controller']==controller and g['valid']]
        style=dict(marker='o' if controller=='mydwa' else 'x',markersize=8 if controller=='mydwa' else 5,markerfacecolor='none',linestyle='--' if controller=='mydwa' else ':')
        axes[0].plot([g['start_time_s'] for g in group],[g['collision_rate'] for g in group],label=controller,**style)
        axes[1].plot([g['start_time_s'] for g in group],[g['success_rate'] for g in group],label=controller,**style)
        axes[2].scatter([],[],marker='o' if controller=='mydwa' else 'x',color='C0' if controller=='mydwa' else 'C1',label=controller)
        for r in rows:
            if r['controller']==controller and r['valid']:
                axes[2].scatter(r['start_time_s']+(-.05 if controller=='mydwa' else .05),r['metrics']['min_gap_approx_m'],marker='o' if controller=='mydwa' else 'x',color='C0' if controller=='mydwa' else 'C1',alpha=.7)
    for ax,title in zip(axes,['physical collision rate','Nav2 task success rate','each run minimum approximate gap m']):
        ax.set(xlabel='configured crossing start after dispatch / sim s',title=title,xticks=[2,3,4,5,6]);ax.grid(alpha=.4)
    for ax in axes[:2]:ax.set_ylim(-.05,1.05);ax.legend()
    axes[2].legend()
    fig.suptitle('D01 timing scan / Release MyDWA vs system DWB / n=3 each point (descriptive only)',fontsize=11)
    fig.tight_layout();fig.savefig(output/'collision_rate.png',dpi=150);plt.close(fig)
    for r in rows:trial_plot(r,*raw[r['run']],destination)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag',default='first')
    parser.add_argument('--partial',action='store_true')
    parser.add_argument('--no-plots',action='store_true')
    args=parser.parse_args()
    if Path(args.tag).name!=args.tag or args.tag in ('.','..'):parser.error('invalid tag')
    sequence=load(HERE/f'sequence_{args.tag}.json')
    rows=[];raw={}
    for planned in sequence['runs']:
        if not (HERE/'runs'/planned['run']/'manifest.json').exists():continue
        if 'returncode' not in planned:continue
        if not (HERE/'runs'/planned['run']/'trial/summary.json').exists():
            raise RuntimeError('Missing summary: '+planned['run'])
        row,gt,commands=analyze_trial(planned);rows.append(row);raw[row['run']]=(gt,commands)
    if not args.partial:assert len(rows)==30 and all(r['valid'] for r in rows),'incomplete/invalid sweep; use --partial for retained evidence'
    groups=aggregate(rows)
    output=HERE/('outputs' if args.tag=='first' else f'outputs_{args.tag}');output.mkdir(exist_ok=True)
    write(output/'results.json',dict(tag=args.tag,complete=len(rows)==30,records=rows,groups=groups))
    fields=['run','order','start_time_s','controller','repeat','valid','action','termination','success','collision','navigation_sim_s','successful_completion_sim_s','min_gap_approx_m','observed_motion_start_s','brake_s','turn_s','actual_brake_s','actual_turn_s','waiting_before_zone_s','lateral_excursion_m','contact_s','contact_robot_x','contact_robot_y','contact_obstacle_x','contact_obstacle_y','category','plugin_sha256']
    with (output/'results.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fields);writer.writeheader()
        for r in rows:
            m=r['metrics'];features=r['features'];c=r['contact']
            writer.writerow(dict(run=r['run'],order=r['order'],start_time_s=r['start_time_s'],controller=r['controller'],repeat=r['repeat'],valid=r['valid'],action=m['action_result'],termination=m['termination'],success=m['success'],collision=m['collision'],navigation_sim_s=m['navigation_sim_s'],successful_completion_sim_s=m['successful_completion_sim_s'],min_gap_approx_m=m['min_gap_approx_m'],observed_motion_start_s=r['motion_validation']['observed_first_gt_motion_s'],**{k:features[k] for k in ('brake_s','turn_s','actual_brake_s','actual_turn_s','waiting_before_zone_s','lateral_excursion_m')},contact_s=None if c is None else c['relative_s'],contact_robot_x=None if c is None else c['robot'][0],contact_robot_y=None if c is None else c['robot'][1],contact_obstacle_x=None if c is None else c['obstacle'][0],contact_obstacle_y=None if c is None else c['obstacle'][1],category=r['category'],plugin_sha256=r['runtime_plugin_sha256']))
    with (output/'groups.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,list(groups[0]));writer.writeheader();writer.writerows(groups)
    if not args.no_plots:plots(rows,groups,raw,output)
    from report import render
    render(rows,groups,len(rows)==30,output.name,args.tag)
    print(table(['start','controller','valid','success','collision','behaviors'],[[g['start_time_s'],g['controller'],g['valid'],g['successes'],g['collisions'],g['categories']] for g in groups if g['completed']]))
if __name__=='__main__':main()