"""Classify actual displacement and risk/reference lifecycles without rerunning."""
import pathlib,json,math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=pathlib.Path('/home/user/ros2_experiments/locked_reference_20261029')
inp=json.loads((root/'input.json').read_text());initial=inp['initial']
def onset(rows,distance):
    begin=None
    for r in rows:
        if initial[1]-r['next_pose'][1]>=distance:
            if begin is None:begin=r['t']+.05
            if r['t']+.05-begin>=.2-1e-8:return begin
        else:begin=None
    return None
def reversals(rows,key,threshold=.05):
    signs=[1 if r[key][1]>0 else -1 for r in rows if abs(r[key][1])>threshold]
    return sum(a!=b for a,b in zip(signs,signs[1:]))
metrics=[];traces={};events=[]
for mode in ['A','B']:
    folder=root/'results/phase1'/f'{mode}_t4';summary=json.loads((folder/'summary.json').read_text())[0]
    allrows=list(map(json.loads,(folder/'start_4.00.jsonl').read_text().splitlines()))
    rows=[r for r in allrows if 'next_pose' in r];traces[mode]=rows
    early=[r for r in rows if r['t']+.05<8.8-1e-8]
    targets={tuple(r['reference']['target']) for r in rows if r['reference']['target'] is not None}
    proposals=[];previous=None;lock=None
    for r in rows:
        reference=r['reference'];now=(reference['proposal'],reference['votes'])
        if now!=previous and reference['phase']==0:
            proposals.append({'t':r['t'],'scan_t':r['scan_t'],'proposal':now[0],'votes':now[1],
                              'proposed_command':r['proposed_candidate'],'command':r['selected']})
        previous=now
        if reference['phase']==1 and lock is None:lock={'t':r['t'],'target':reference['target'],'stamp':reference['lock_stamp']}
        if reference['released'] or reference['aborted'] or r['risk']['released']:
            events.append({'mode':mode,'t':r['t'],'reference':reference,'risk':r['risk'],'ordinary_recovers_path':r['ordinary_recovers_path']})
    result={'mode':mode,'start':4,'end':rows[-1]['t']+.05,'cycles':len(rows),
      'first_motion':summary['first_motion'],'first_rearward_002_sustained':onset(rows,.02),
      'first_rearward_005_sustained':onset(rows,.05),
      'max_rearward_before_88':max([initial[1]-r['next_pose'][1] for r in early]+[0]),
      'max_rearward_full_window':max([initial[1]-r['next_pose'][1] for r in rows]+[0]),
      'first_selected_rear_reaching':next((r['t'] for r in rows if r['selected_rear_reaching'] is True),None),
      'rear_entry':None if summary['rear_entry']<0 else summary['rear_entry'],
      'pass_time':None if summary['pass_time']<0 else summary['pass_time'],
      'minimum_clearance':min(r['executed_clearance'] for r in rows),
      'safety_failure':summary['failure'] not in ['', 'timeout'],'fallback_losses':sum(not r['fallback_ok'] for r in allrows),
      'contact_checks_nonpositive':sum(r['executed_clearance']<=0 for r in rows),
      'intrusion_checks_failed':summary['failure']=='executed_contact_or_exclusion_violation',
      'stops':summary['stops'],'slow_crawl_seconds':sum(.01<=abs(r['measured'][0])<.05 for r in rows)*.05,
      'near_stopped_seconds':sum(abs(r['measured'][0])<.01 and abs(r['measured'][1])<.05 for r in rows)*.05,
      'command_reversals':reversals(rows,'selected'),'measured_reversals':reversals(rows,'measured'),
      'absolute_yaw_integral':sum(abs(r['measured'][1]) for r in rows)*.05,
      'lock':lock,'distinct_locked_points':list(targets),'proposals':proposals,
      'reference_release_times':[r['t'] for r in rows if r['reference']['released']],
      'risk_release_times':[r['t'] for r in rows if r['risk']['released']],
      'abort_times':[r['t'] for r in rows if r['reference']['aborted']],
      'all_fallback_checks':all(r['fallback_ok'] for r in allrows),'final_pose':summary['final_pose']}
    metrics.append(result);print(json.dumps(result,ensure_ascii=False))
a,b=metrics
positive=(not b['safety_failure'] and b['all_fallback_checks'] and b['max_rearward_before_88']>=.05 and
          b['first_rearward_005_sustained'] is not None and b['first_rearward_005_sustained']<8.8-.2+1e-8 and
          b['pass_time'] is not None and a['pass_time']-b['pass_time']>=.2-1e-8)
result={'metrics':metrics,'events':events,'positive_screen':positive,'phase2_required':positive,
        'pass_gain_vs_parking_A':a['pass_time']-b['pass_time'],
        'actual_entry_lead_vs_historical_88':8.8-b['rear_entry'],
        'note':'Only emitted actions count as execution; proposed candidates while held are not motion. Strict t<8.8 sample window.'}
(root/'analysis.json').write_text(json.dumps(result,indent=2))
fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True)
for mode,rows in traces.items():
    times=[r['t']+.05 for r in rows];x=[r['next_pose'][0] for r in rows];y=[r['next_pose'][1]-initial[1] for r in rows]
    axes[0].plot(times,x,label=mode);axes[1].plot(times,y,label=mode);axes[2].plot(times,[r['measured'][0] for r in rows],label=mode)
for ax in axes:ax.axvline(8.8,color='black',ls='--');ax.grid(alpha=.2);ax.legend()
axes[0].axhline(1.08,color='gray',ls=':');axes[0].axhline(2.12,color='gray',ls=':')
axes[1].axhline(-.05,color='gray',ls=':')
axes[0].set_ylabel('World x (m)');axes[1].set_ylabel('Rearward displacement (m)');axes[2].set_ylabel('Speed (m/s)')
axes[2].set_xlabel('Navigation-relative time (s)')
fig.suptitle('A: latched wait / B: four-frame locked reference, original weights')
fig.tight_layout();fig.savefig(root/'comparison.png',dpi=180)
