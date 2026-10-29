"""Read-only analysis of actual emitted trajectories; never launch a trial."""
import json,pathlib,math
from common import ROOT,metrics,rows
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
matrix=json.loads((ROOT/'params/matrix.json').read_text())
folders=sorted((ROOT/'results/phase1').iterdir())
results=[];traces={};events={};scores={}
for f in folders:
    if not (f/'summary.json').exists():continue
    m=metrics(f);p=json.loads((f/'parameters.json').read_text());m.update({'mode':p['mode'],'heading_multiplier':p['heading_multiplier'],'margin':p['extra_clearance']})
    results.append(m);data=rows(f);traces[f.name]=data
    # Preserve every source-time proposal change and all lifecycle events.
    events[f.name]=[{'t':r['t'],'scan_t':r['scan_t'],'reference':r['reference'],'risk':r['risk'],'zone':r['zone'],
       'command':r['selected'],'pose':r['pose']} for i,r in enumerate(data)
       if i==0 or r['scan_t']!=data[i-1]['scan_t'] or r['reference']['released'] or r['reference']['aborted']]
    keys=[5.,6.,7.,8.,8.4,8.6,8.8,9.]
    scores[f.name]=[{k:r[k] for k in ['t','pose','measured','selected','proposed_candidate','command_held','reference','guided_scoring','effective_heading_weight','score','weighted_score','rejected','zone']}
       for r in data if any(abs(r['t']-t)<1e-8 for t in keys)]
a=next(m for m in results if m['name']=='A_t4');b=next(m for m in results if m['name']=='H1_M10_t4')
for m in results:
    m['pass_gain_vs_A']=None if m['pass_time'] is None else a['pass_time']-m['pass_time']
    m['pass_gain_vs_B_original']=None if m['pass_time'] is None else b['pass_time']-m['pass_time']
    m['entry_lead_vs_88']=None if m['rear_entry'] is None else 8.8-m['rear_entry']
    m['eligible_for_confirmation']=(m['mode']=='B' and not m['safety_failure'] and not m['abnormal_chatter'] and
       m['fallback_losses']==0 and not m['contact'] and not m['intrusion'] and
       m['first_5cm_sustained'] is not None and m['first_5cm_sustained']<=8.6+1e-8 and
       m['max_rearward_before_88']>=.05 and m['rear_entry'] is not None and m['rear_entry']<8.8-1e-8 and
       m['pass_gain_vs_A'] is not None and m['pass_gain_vs_A']>=.2-1e-8 and
       m['pass_gain_vs_B_original']>=.2-1e-8 and m['stops']<=1 and m['command_reversals']<=4)
eligible=sorted([m for m in results if m['eligible_for_confirmation']],key=lambda m:(m['heading_multiplier'],-m['minimum_clearance'],m['pass_time']))
selected=eligible[0] if eligible and not (ROOT/'stopped.json').exists() else None
out={'matrix_complete':len(results)==13,'stopped':(ROOT/'stopped.json').exists(),'metrics':results,
     'selected_for_confirmation':None if not selected else {k:selected[k] for k in ['name','heading_multiplier','margin']},
     'observation':'ideal current scene, not real Scan or ROS/Gazebo loop','screening':matrix['confirmation_eligibility']}
for name,content in [('analysis.json',out),('reference_events.json',events),('key_scores.json',scores)]:
    (ROOT/name).write_text(json.dumps(content,indent=2))

def fmt(x,digits=2):return '—' if x is None else f'{x:.{digits}f}'
lines=['| H倍数 | 参考余量 m | 锁定 s | 开始运动 s | 8.8前最大后移 cm | 入区 s | 通过 s | 最小净距 cm | 再停/换向 |',
       '|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
for m in sorted(results,key=lambda m:(m['mode']!='A',m['margin'],m['heading_multiplier'])):
    lines.append(f'| {"A" if m["mode"]=="A" else int(m["heading_multiplier"])} | {"—" if m["mode"]=="A" else fmt(m["margin"],1)} | {fmt(m["first_lock"]["t"] if m["first_lock"] else None)} | {fmt(m["first_motion"])} | {100*m["max_rearward_before_88"]:.2f} | {fmt(m["rear_entry"])} | {fmt(m["pass_time"])} | {fmt(100*m["minimum_clearance"] if m["minimum_clearance"] is not None else None)} | {m["stops"]}/{m["command_reversals"]} |')
(ROOT/'comparison_table.md').write_text('\n'.join(lines)+'\n')
fig,axes=plt.subplots(3,3,figsize=(16,10),sharex=True)
colors={1:'#6c757d',10:'#0077b6',20:'#e76f51',40:'#8338ec'}
for j,margin in enumerate([.1,.2,.3]):
    for m in results:
        if m['mode']!='B' or abs(m['margin']-margin)>1e-8:continue
        data=[r for r in traces[m['name']] if 'next_pose' in r];times=[r['t']+.05 for r in data]
        for i,values in enumerate([[r['next_pose'][1]-.003770063344808791 for r in data],
                                  [r['next_pose'][0] for r in data], [r['measured'][0] for r in data]]):
            axes[i,j].plot(times,values,label=f'H x{m["heading_multiplier"]:g}',color=colors[m['heading_multiplier']])
    axes[0,j].set_title(f'Extra / forward offset = {margin:.1f} m')
    for ax in axes[:,j]:ax.axvline(8.8,color='black',ls='--',lw=1);ax.grid(alpha=.25);ax.legend(fontsize=8)
    axes[0,j].axhline(-.05,color='gray',ls=':');axes[1,j].axhline(1.08,color='gray',ls=':');axes[1,j].axhline(2.12,color='gray',ls=':')
    axes[2,j].set_xlabel('Navigation-relative time (s)')
axes[0,0].set_ylabel('Actual rearward displacement (m)');axes[1,0].set_ylabel('World x (m)');axes[2,0].set_ylabel('Measured speed (m/s)')
fig.suptitle('Original safety gates, H multiplier sweep; paired reference geometry margins')
fig.tight_layout();fig.savefig(ROOT/'comparison.png',dpi=160)
print('\n'.join(lines));print('Selected:',out['selected_for_confirmation'])
