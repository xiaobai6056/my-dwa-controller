import json
from pathlib import Path
H=Path(__file__).resolve().parent
z=json.loads((H/'outputs/offline_results.json').read_text())
for s in (2,3,4,5):
 for c in ('mydwa','dwb'):
  g=[r for r in z['runs'] if r['controller']==c and r['start_time_s']==s]
  print(s,c,[(r['repeat'],{p:(w['latest_sample_safe_s'],w['next_sample_unsafe_s']) for p,w in r['windows'].items()}) for r in g])
for r in json.loads((H/'outputs/decision_summary.json').read_text()):
 print('\n',r['run'],'contact',r['physical_contact_s'],'latest',r['latest_observed_conditional_safe_cycle']['t'],'brake',None if r['first_successful_plugin_output_le_0_21'] is None else r['first_successful_plugin_output_le_0_21']['t'])
 for z in [r['latest_observed_conditional_safe_cycle'],*r['examples']]:
  print('cycle',round(z['t'],3),'x',round(z['x'],3),'v',z['command_v'],'w',z['command_w'],'valid',z['valid'],'margin',round(z['conditional_gap_lower_m'],3),'chosen',None if not z['best'] else z['best']['scores'],'slow',None if not z['slowest_best'] else (z['slowest_best']['v'],z['slowest_best']['scores']))