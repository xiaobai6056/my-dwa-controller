"""Fixed matrix; no search expansion and no reruns. Each directory is exclusive."""
import pathlib,json,subprocess,sys,hashlib
from common import ROOT,metrics,rows,previous_root
BASE=previous_root()
def run(name,mode,start=4.,heading=1.,margin=.1,phase='phase1'):
    folder=ROOT/'results'/phase/name;folder.mkdir(parents=True,exist_ok=False)
    original=json.loads((ROOT/'input.json').read_text())
    input=dict(original);input['reference_parameters']={'heading_multiplier':heading,'extra_clearance':margin,'forward_offset':margin}
    # Scene, static geometry, path, initial state, motion/sampler configuration stay unchanged.
    assert {k:v for k,v in input.items() if k!='reference_parameters'}==original
    (folder/'input.json').write_text(json.dumps(input,indent=2))
    params={'mode':mode,'start':start,'heading_multiplier':heading,'heading_weight':.05*heading,
      'extra_clearance':margin,'forward_offset':margin,'end':16.,'weights_other':{'P':.2,'G':.6,'V':.1,'O':.1},
      'observation':'ideal full scene envelope/association and current-pose synthetic rays, not real Scan',
      'geometry':'only reference construction/certificate margins change; hard safety gates unchanged'}
    (folder/'parameters.json').write_text(json.dumps(params,indent=2))
    cmd=[str(ROOT/'build/rolling'),str(folder/'input.json'),str(folder),mode,str(start)]
    with (ROOT/'commands.jsonl').open('a') as f:f.write(json.dumps(cmd)+'\n')
    with (folder/'stdout.log').open('w') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
    m=metrics(folder);(folder/'metrics.json').write_text(json.dumps(m,indent=2))
    print(json.dumps(m),flush=True)
    if m['safety_failure'] or m['contact'] or m['intrusion'] or m['fallback_losses'] or m['abnormal_chatter']:
        (ROOT/'stopped.json').write_text(json.dumps({'variant':name,'metrics':m,'reason':'predeclared safety/chatter stop; do not tune or retry'},indent=2))
        raise SystemExit('STOP: safety or abnormal chatter; preserve partial matrix')
    return folder,m
if __name__=='__main__':
    phase=sys.argv[1]
    if phase=='baseline':
        checks={}
        for name,mode in [('A_t4','A'),('H1_M10_t4','B')]:
            folder,m=run(name,mode)
            previous=rows(BASE/f'results/phase1/{mode}_t4');actual=rows(folder)
            assert len(previous)==len(actual)==240
            mismatch=[]
            for i,(a,b) in enumerate(zip(previous,actual)):
                # Added metadata is intentionally excluded; every old action/state/score field must match.
                if a!={k:b[k] for k in a}:mismatch.append(i)
            checks[mode]={'cycles':len(actual),'different_rows':mismatch,'all_inherited_fields_equal':not mismatch}
            assert not mismatch,checks
        (ROOT/'baseline_reproduction.json').write_text(json.dumps(checks,indent=2))
    elif phase=='matrix':
        assert json.loads((ROOT/'baseline_reproduction.json').read_text())['B']['all_inherited_fields_equal']
        assert not (ROOT/'stopped.json').exists()
        for margin in [.1,.2,.3]:
            for heading in [1.,10.,20.,40.]:
                if margin==.1 and heading==1:continue
                run(f'H{heading:g}_M{int(margin*100):02d}_t4','B',heading=heading,margin=margin)
    elif phase=='confirm':
        screen=json.loads((ROOT/'analysis.json').read_text());selected=screen['selected_for_confirmation']
        assert selected and not (ROOT/'stopped.json').exists()
        for start in [6.,8.]:
            run(f'A_t{start:g}','A',start,phase='phase2')
            run(f'{selected["name"].removesuffix("_t4")}_t{start:g}','B',start,selected['heading_multiplier'],selected['margin'],phase='phase2')
    else:raise SystemExit('baseline | matrix | confirm')
