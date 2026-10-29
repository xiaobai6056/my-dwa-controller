import pathlib,sys,json
ROOT=pathlib.Path('/home/user/ros2_experiments/polyline_bypass_20261029');sys.path.insert(0,str(ROOT/'scripts'))
from common import rows,metrics
out={}
for name in ['A','B','C','D','E']:
 d=ROOT/'results/phase1'/name;data=rows(d);m=metrics(d)
 locks=[r for r in data if r['polyline'] and r['reference']['phase']==1]
 out[name]={'lock_line':locks[0]['polyline'] if locks else None,'slow_crawl_s':m['slow_crawl_seconds'],'yaw_integral':m['absolute_yaw_integral'],
 'last_pose':m['final_pose'],'pure_rotation_s':m['pure_rotation_seconds'],'key_cycles':[]}
 for t in [5,6,6.5,7,7.15,8,10,12,12.6,14,15.95]:
  matches=[r for r in data if abs(r['t']-t)<1e-8]
  if matches:
   r=matches[0];out[name]['key_cycles'].append({k:r[k] for k in ['t','pose','measured','selected','command_held','score','weighted_score','polyline','reference','rejected']})
f=json.loads((ROOT/'failure_check.json').read_text());out['E_failure']={k:v for k,v in f.items() if not k.endswith('_poses')}
out['E_failure']['fallback_max_x']=max(p[0] for p in f['fallback_poses']);out['E_failure']['queue_max_x']=max(p[0] for p in f['queue_poses'])
out['E_failure']['reserved_entry_x']=f['zone'][0]-f['reserve']
(ROOT/'diagnosis.json').write_text(json.dumps(out,indent=2));print(json.dumps(out['E_failure'],indent=2))
for n in ['A','B','C','D','E']: print(n,json.dumps({k:v for k,v in out[n].items() if k!='key_cycles'}))
