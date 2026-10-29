"""Fixed-window metrics and pre-registered phase-one screening."""
import pathlib,json,math,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=pathlib.Path('/home/user/ros2_experiments/score_ablation_20261029')
inp=json.loads((root/'input.json').read_text());initial=inp['initial'];walls=inp['walls']
def first_sustained(rows,predicate,duration=.2):
    begin=None
    for r in rows:
        if predicate(r):
            if begin is None:begin=r['t']+.05
            if r['t']+.05-begin>=duration-1e-8:return begin
        else:begin=None
    return None
def flips(values,threshold):
    signs=[1 if v>threshold else -1 for v in values if abs(v)>threshold]
    return sum(a!=b for a,b in zip(signs,signs[1:]))
def static_gap(p):
    def distance(b):
        a,c,d,e=b;dx=max(a-p[0],0,p[0]-d);dy=max(c-p[1],0,p[1]-e)
        return math.hypot(dx,dy) if dx or dy else -min(p[0]-a,d-p[0],p[1]-c,e-p[1])
    return min(distance(b) for b in walls)-.22
metrics=[];all_rows={};keys=[]
for folder in sorted((root/'results/phase1').iterdir()):
    params=json.loads((folder/'parameters.json').read_text());scheme=params['scheme'];start=params['start']
    summary=json.loads((folder/'summary.json').read_text())[0]
    rows=list(map(json.loads,(folder/f'start_{start:.2f}.jsonl').read_text().splitlines()))
    executed=[r for r in rows if 'next_pose' in r];all_rows[scheme]=executed
    early=[r for r in executed if r['t']+.05<8.8+1e-8]
    rear_shift=max([initial[1]-r['next_pose'][1] for r in early]+[0])
    lateral=max([abs(initial[1]-r['next_pose'][1]) for r in early]+[0])
    negative_onset=first_sustained(executed,lambda r:r['next_pose'][1]<=initial[1]-.05)
    clearances=[r['executed_clearance'] for r in executed]
    static_gaps=[static_gap(r['next_pose']) for r in executed]
    zone_gaps=[math.hypot(max(1.08-r['next_pose'][0],0,r['next_pose'][0]-2.12),
                         max(r['zone'][2]-r['next_pose'][1],0)) for r in executed]
    full_window=bool(executed) and abs(executed[-1]['t']+.05-14)<1e-7
    fallback_fail=sum(not r['fallback_ok'] for r in rows)
    safety_reason=summary['failure'] not in ['', 'timeout']
    violating=sum(1.08<=r['next_pose'][0]<=2.12 and r['next_pose'][1]>=r['zone'][2] for r in executed)
    stagnant=[]
    for i,r in enumerate(executed):
        if i>=20 and math.dist(r['next_pose'][:2],executed[i-20]['next_pose'][:2])<.02:stagnant.append(r['t']+.05)
    longest=0;begin=None;previous=None
    for t in stagnant:
        if previous is None or t-previous>.051:begin=t
        longest=max(longest,t-begin+1);previous=t
    result={'scheme':scheme,'start':start,'end':executed[-1]['t']+.05,'full_window':full_window,
       'weights':params['weights'],'first_motion':summary['first_motion'],
       'first_rearward_005_sustained':negative_onset,'max_rearward_shift_before_88':rear_shift,
       'max_abs_lateral_before_88':lateral,
       'first_selected_rear_reaching':next((r['t'] for r in rows if r['selected_rear_reaching'] is True),None),
       'rear_entry':None if summary['rear_entry']<0 else summary['rear_entry'],
       'pass_time':None if summary['pass_time']<0 else summary['pass_time'],
       'minimum_clearance':min(clearances),'minimum_static_trace_gap':min(static_gaps),
       'minimum_zone_trace_gap':min(zone_gaps),'zone_boundary_under_002_seconds':sum(g<.02 for g in zone_gaps)*.05,
       'contact_checks_nonpositive':sum(c<=0 for c in clearances),'exclusion_endpoint_violations':violating,
       'fallback_admission_losses':fallback_fail,'safety_failure':safety_reason,
       'safe_full_window':full_window and not fallback_fail and not safety_reason and not violating and min(clearances)>.05,
       'failure':summary['failure'],'stops':summary['stops'],
       'slow_crawl_seconds':sum(.01<=abs(r['measured'][0])<.05 for r in rows)*.05,
       'near_stopped_seconds':sum(abs(r['measured'][0])<.01 and abs(r['measured'][1])<.05 for r in rows)*.05,
       'rotation_only_seconds':sum(abs(r['measured'][0])<.01 and abs(r['measured'][1])>=.05 for r in rows)*.05,
       'command_turn_reversals_005':flips([r['selected'][1] for r in rows],.05),
       'measured_turn_reversals_005':flips([r['measured'][1] for r in rows],.05),
       'absolute_yaw_integral':sum(abs(r['measured'][1]) for r in rows)*.05,
       'stagnant_1s_window_count':len(stagnant),'longest_low_progress_episode_seconds':longest,
       'static_trace_gap_under_010_seconds':sum(g<.10 for g in static_gaps)*.05,
       'final_pose':summary['final_pose'],'net_forward_progress':summary['final_pose'][0]-initial[0],
       'cycles':len(rows),'rejections':summary['rejections']}
    metrics.append(result)
    for time in [4.5,6.55,8,8.6,10,13.5]:
        r=min(rows,key=lambda r:abs(r['t']-time))
        keys.append({'scheme':scheme,'t':r['t'],'pose':r['pose'],'measured':r['measured'],
          'winner':r['selected'],'score':r['score'],'weighted':r['weighted_score'],
          'rejections':r['rejected'],'rank':r['selected_rank'],'fallback_ok':r['fallback_ok']})
baseline=next(m for m in metrics if m['scheme']=='A')
shortlist=[]
for m in metrics:
    m['pass_gain_vs_A']=None if m['pass_time'] is None else baseline['pass_time']-m['pass_time']
    m['screen_positive']=bool(m['scheme']!='A' and m['safe_full_window'] and
      m['max_rearward_shift_before_88']>=.05 and m['pass_gain_vs_A'] is not None and m['pass_gain_vs_A']>=.2-1e-8)
    if m['screen_positive']:shortlist.append(m['scheme'])
    print(json.dumps(m,ensure_ascii=False))
same={}
for a,b in [('A','B'),('C','D'),('C','E')]:
    same[a+b]={'cycles':len(all_rows[a]),'command_mismatches':sum(x['selected']!=y['selected'] for x,y in zip(all_rows[a],all_rows[b])),
       'pose_mismatches':sum(x['next_pose']!=y['next_pose'] for x,y in zip(all_rows[a],all_rows[b]))}
result={'fixed_end_time':14,'metric_thresholds':{'rearward_shift':.05,'sustain_seconds':.2,'pass_gain':.2,
    'turn_sign_deadband':.05,'slow_crawl_v_range':[.01,.05],'static_hugging_approx_gap':.10},
    'metrics':metrics,'shortlist':shortlist,'phase2_required':bool(shortlist),'sequence_comparisons':same,
    'note':'No policy-level normalization, new planner, guide point or safety change. Thresholds classify outputs only.'}
(root/'analysis.json').write_text(json.dumps(result,indent=2))
(root/'key_cycles.json').write_text(json.dumps(keys,indent=2))
with (root/'comparison.csv').open('w',newline='') as f:
    fields=[k for k in metrics[0] if k not in ['weights','rejections','final_pose']]
    writer=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');writer.writeheader();writer.writerows(metrics)
fig,axs=plt.subplots(1,2,figsize=(12,5))
colors=plt.get_cmap('tab10')
for i,(scheme,rows) in enumerate(all_rows.items()):
    t=np.array([r['t']+.05 for r in rows]);xy=np.array([r['next_pose'][:2] for r in rows])
    axs[0].plot(xy[:,0],xy[:,1],label=scheme,color=colors(i));axs[1].plot(t,xy[:,1]-initial[1],label=scheme,color=colors(i))
axs[0].axvline(1.08,color='gray',ls=':');axs[0].axvline(2.12,color='gray',ls=':')
axs[0].set_xlabel('Scene x (m)');axs[0].set_ylabel('Scene y (m)');axs[0].set_title('Fixed-window executed paths')
axs[1].axvline(8.8,color='black',ls='--');axs[1].axhline(-.05,color='gray',ls=':')
axs[1].set_xlabel('Navigation-relative time (s)');axs[1].set_ylabel('Actual lateral shift (m)');axs[1].set_title('Rearward = negative; historical release = 8.8 s')
for ax in axs:ax.grid(alpha=.2);ax.legend(ncol=4)
fig.tight_layout();fig.savefig(root/'comparison.png',dpi=180)
