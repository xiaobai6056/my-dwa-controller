import json
from common import ROOT
out={}
for name in ['H1_M10_t4','H10_M10_t4','H40_M10_t4','H10_M20_t4','H40_M30_t4']:
    data=json.loads((ROOT/f'results/phase1/{name}/score_audit.json').read_text());selected=[]
    for t in [5.,8.4,12.,14.,15.95]:
        matches=[a for a in data if abs(a['t']-t)<1e-6]
        if not matches:continue
        a=matches[0];choices=a['choices'];summary={'t':a['t'],'pose':a['pose'],'measured':a['measured'],'desired':a['desired_yaw'],'choices':{}}
        for kind in ['selected_proposal','fastest_admitted','best_ordinary_admitted']:
            c=choices[kind];summary['choices'][kind]=None if c is None else {k:c[k] for k in ['command','admitted','guided_terms','ordinary_terms_same_action','nominal_end']}
        selected.append(summary)
        print(name,round(a['t'],2),'desired',a['desired_yaw'])
        for kind in ['selected_proposal','fastest_admitted','best_ordinary_admitted']:
            c=choices[kind]
            if c is not None:print(kind,json.dumps(c['command']),'J',c['guided_terms']['total'],'H',c['guided_terms']['heading'],'G',c['guided_terms']['goal'],'P',c['guided_terms']['path'],'V',c['guided_terms']['speed'],'O',c['guided_terms']['obstacle'],'end',c['nominal_end'])
    out[name]=selected
(ROOT/'score_evidence.json').write_text(json.dumps(out,indent=2))
