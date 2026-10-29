import json,subprocess
from common import ROOT,rows,metrics
names=['H1_M10_t4','H10_M10_t4','H40_M10_t4','H1_M20_t4','H10_M20_t4','H1_M30_t4','H40_M30_t4']
results={}
for name in names:
    folder=ROOT/'results/phase1'/name;data=rows(folder)
    cmd=[str(ROOT/'build/audit_reference'),str(folder/'input.json'),str(folder/'start_4.00.jsonl'),str(folder/'score_audit.json')]
    with (ROOT/f'logs/audit_{name}.log').open('w') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
    points=[]
    for t in [5,5.8,6.6,8,8.4,8.6,8.8,10,12,13,14,15,15.95]:
        r=min(data,key=lambda r:abs(r['t']-t))
        points.append({k:r[k] for k in ['t','pose','measured','selected','proposed_candidate','reference','risk','zone','rejected','score','weighted_score','ordinary_admitted','ordinary_recovers_path']})
    first=next((r for r in data if r['reference']['phase']==1),None)
    line=None
    if first:
        p=first['pose'];target=first['reference']['target'];entry=first['zone'][0]
        line={'lock_time':first['t'],'reference':target,'robot':p,'rear_boundary':first['zone'][2],
              'direct_target_y_at_entry':p[1]+(target[1]-p[1])*(entry-p[0])/(target[0]-p[0])}
    progress={}
    for begin,end in [(8.8,10),(10,12),(12,14),(14,16)]:
        a=min(data,key=lambda r:abs(r['t']-begin));b=min(data,key=lambda r:abs(r['t']+.05-end))
        part=[r for r in data if begin-1e-8<=r['t']<end-1e-8]
        progress[f'{begin:g}-{end:g}']={'dx':b['next_pose'][0]-a['pose'][0],
          'dy':b['next_pose'][1]-a['pose'][1],
          'mean_measured_v':sum(r['measured'][0] for r in part)/len(part),
          'zero_translation_proposals':sum(r['proposed_candidate'][0]==0 for r in part),
          'pure_rotation_cycles':sum(r['selected'][0]==0 and r['selected'][1]!=0 for r in part)}
    results[name]={'direct_heading':line,'progress':progress,'key_cycles':points}
(ROOT/'diagnosis.json').write_text(json.dumps(results,indent=2))
for name,r in results.items():
    print(name,json.dumps(r['direct_heading']),json.dumps(r['progress']))
    for p in r['key_cycles'][-4:]:print(json.dumps({'t':p['t'],'pose':p['pose'],'command':p['selected'],'phase':p['reference']['phase'],'ordinary_recovers_path':p['ordinary_recovers_path']}))
