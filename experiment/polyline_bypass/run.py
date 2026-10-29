import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from paths import data_root, previous_root
import pathlib,sys,json,subprocess
ROOT=data_root('polyline_bypass_20261029')
from common import metrics,rows
BASE=previous_root()
def run(name,mode,poly=False,h=1.,margin=.1,start=4.,phase='phase1'):
 d=ROOT/'results'/phase/name;d.mkdir(parents=True,exist_ok=False)
 inp=json.loads((ROOT/'input.json').read_text());inp['reference_parameters']={'polyline':poly,'heading_multiplier':h,'extra_clearance':margin,'forward_offset':margin}
 (d/'input.json').write_text(json.dumps(inp,indent=2));(d/'parameters.json').write_text(json.dumps({'polyline':poly,'mode':mode,'h':h,'margin':margin,'start':start,'end':16},indent=2))
 cmd=[str(ROOT/'build/rolling'),str(d/'input.json'),str(d),mode,str(start)]
 with (ROOT/'commands.jsonl').open('a') as f:f.write(json.dumps(cmd)+'\n')
 with (d/'stdout.log').open('w') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
 m=metrics(d);(d/'metrics.json').write_text(json.dumps(m,indent=2));print(json.dumps(m),flush=True)
 if m['safety_failure'] or m['fallback_losses'] or m['contact'] or m['intrusion'] or m['abnormal_chatter']:
  (ROOT/'stopped.json').write_text(json.dumps(m,indent=2));raise SystemExit('STOP unsafe/chatter; no tuning/retry')
 return d
phase=sys.argv[1]
if phase=='baseline':
 checks={}
 for name,mode,previous in [('A0','A','A_t4'),('B0','B','H1_M10_t4')]:
  d=run(name,mode);a=rows(BASE/'results/phase1'/previous);b=rows(d)
  mismatch=[i for i,(x,y) in enumerate(zip(a,b)) if x!={k:y[k] for k in x}]
  assert len(a)==len(b)==240 and not mismatch,mismatch;checks[name]={'equal_old_fields_cycles':240}
 (ROOT/'baseline_reproduction.json').write_text(json.dumps(checks,indent=2))
elif phase=='matrix':
 assert (ROOT/'baseline_reproduction.json').exists()
 for i,(h,m) in enumerate((h,m) for h in [1.,2.,4.,8.] for m in [.1,.2]):run(chr(65+i),'B',True,h,m)
elif phase=='confirm':
 selected=json.loads((ROOT/'analysis.json').read_text())['selected'];assert selected
 for start in [6.,8.]:
  run(f'A0_t{start:g}','A',start=start,phase='phase2');run(f'B0_t{start:g}','B',start=start,phase='phase2')
  for s in selected:run(f'{s["name"]}_t{start:g}','B',True,s['h'],s['margin'],start,'phase2')
elif phase=='upper':
 analysis=json.loads((ROOT/'analysis.json').read_text());assert analysis['selected'] and analysis['best_h']==8
 m=analysis['selected'][0]['margin']
 for h in [12.,16.]:run(f'H{h:g}_extra','B',True,h,m,phase='upper')
