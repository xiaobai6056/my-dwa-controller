import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from paths import data_root, previous_root
import pathlib,sys,json,math
ROOT=data_root('polyline_bypass_20261029')
from common import rows,metrics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
out=[];traces={};details={}
for phase in ['phase1','upper','phase2']:
 if not (ROOT/'results'/phase).exists():continue
 for d in sorted((ROOT/'results'/phase).iterdir()):
  if not (d/'summary.json').exists():continue
  m=metrics(d);p=json.loads((d/'parameters.json').read_text());m.update(p);m['phase']=phase
  data=rows(d);traces[d.name]=data
  m['no_progress_seconds']=sum(abs(r['next_pose'][0]-r['pose'][0])+abs(r['next_pose'][1]-r['pose'][1])<.0005 for r in data if 'next_pose' in r)*.05
  m['tail_pure_rotation_seconds']=sum(r['t']>=12 and r['selected'][0]==0 and abs(r['selected'][1])>.05 for r in data)*.05
  m['returned_to_path']=any(r['reference']['phase']==2 and abs(r['next_pose'][1]-.003770063344808791)<.05 and r['next_pose'][0]>2.12 for r in data if 'next_pose' in r)
  m['lock_line']=next((r['polyline'] for r in data if r['reference']['phase']==1 and r['polyline']),None)
  details[d.name]=[{k:r[k] for k in ['t','scan_t','polyline','reference','risk','pose','selected','proposed_candidate','command_held','score','weighted_score','rejected','zone','ordinary_recovers_path']} for i,r in enumerate(data) if i==0 or r['scan_t']!=data[i-1]['scan_t'] or r['reference']['released'] or r['reference']['aborted']]
  out.append(m)
a=next(m for m in out if m['name']=='A0');b=next(m for m in out if m['name']=='B0')
for m in out:
 m['qualified']=m['polyline'] and m['start']==4 and not m['safety_failure'] and not m['abnormal_chatter'] and m['first_5cm_sustained'] is not None and m['first_5cm_sustained']<=8.6+1e-8 and m['max_rearward_before_88']>=.05 and m['rear_entry'] is not None and m['rear_entry']<8.8-1e-8 and m['pass_time'] is not None and b['pass_time']-m['pass_time']>=.2-1e-8 and bool(m['reference_releases']) and m['returned_to_path'] and m['tail_pure_rotation_seconds']<.2
eligible=sorted([m for m in out if m['qualified']],key=lambda m:(round(m['pass_time']/.2),m['h'],-m['margin']))
selected=[{k:m[k] for k in ['name','h','margin']} for m in eligible[:2]]
result={'metrics':out,'selected':selected,'best_h':None if not eligible else eligible[0]['h'],'complete_matrix':sum(m['phase']=='phase1' for m in out)==10,'model':'ideal geometry/ID/direction and current-pose synthetic rays, not real Scan','qualified_count':len(eligible)}
(ROOT/'analysis.json').write_text(json.dumps(result,indent=2));(ROOT/'key_states.json').write_text(json.dumps(details,indent=2))
def fmt(v):return '—' if v is None else f'{v:.2f}'
lines=['| 组 | H×/余量m | 锁定/启动s | 8.8前后移cm | 持续5cm起点s | 入区/通过s | 释放s | 净距cm | 停车/换向 |', '|---|---|---|---:|---:|---|---|---:|---|']
for m in out:
 lines.append(f'| {m["name"]} | {m["h"]:g}/{m["margin"]} | {fmt(m["first_lock"]["t"] if m["first_lock"] else None)}/{fmt(m["first_motion"])} | {100*m["max_rearward_before_88"]:.2f} | {fmt(m["first_5cm_sustained"])} | {fmt(m["rear_entry"])}/{fmt(m["pass_time"])} | {fmt(m["reference_releases"][0] if m["reference_releases"] else None)} | {100*m["minimum_clearance"]:.2f} | {m["stops"]}/{m["command_reversals"]} |')
(ROOT/'table.md').write_text('\n'.join(lines)+'\n');print('\n'.join(lines));print('Selected:',selected)
fig,axes=plt.subplots(2,2,figsize=(13,8))
for j,margin in enumerate([.1,.2]):
 for m in out:
  if m['phase']!='phase1' or (m['polyline'] and m['margin']!=margin):continue
  data=[r for r in traces[m['name']] if 'next_pose' in r];t=[r['t']+.05 for r in data]
  axes[0,j].plot([r['next_pose'][0] for r in data],[r['next_pose'][1] for r in data],label=m['name'])
  axes[1,j].plot(t,[r['next_pose'][1]-.003770063344808791 for r in data],label=m['name'])
 axes[0,j].axvspan(1.08,2.12,alpha=.08,color='red');axes[0,j].set_title(f'Reference extra margin {margin} m');axes[0,j].set_xlabel('World x (m)');axes[0,j].set_ylabel('World y (m)')
 axes[1,j].axvline(8.8,color='black',ls='--');axes[1,j].axhline(-.05,color='gray',ls=':');axes[1,j].set_xlabel('Navigation-relative time (s)');axes[1,j].set_ylabel('Actual rear shift (m)')
 for ax in axes[:,j]:ax.legend();ax.grid(alpha=.25)
fig.tight_layout();fig.savefig(ROOT/'comparison.png',dpi=160)
