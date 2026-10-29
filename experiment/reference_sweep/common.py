import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from paths import data_root, previous_root
import pathlib,json,math
ROOT=data_root('reference_sweep_20261029')
INITIAL=json.loads((ROOT/'input.json').read_text())['initial']
def rows(folder):
    return [json.loads(x) for x in next(folder.glob('start_*.jsonl')).read_text().splitlines()]
def onset(data,d):
    begin=None
    for r in data:
        if 'next_pose' not in r:continue
        if INITIAL[1]-r['next_pose'][1]>=d:
            if begin is None:begin=r['t']+.05
            if r['t']+.05-begin>=.2-1e-8:return begin
        else:begin=None
    return None
def reversals(data,key):
    signs=[1 if r[key][1]>0 else -1 for r in data if abs(r[key][1])>.05]
    return sum(a!=b for a,b in zip(signs,signs[1:]))
def metrics(folder):
    s=json.loads((folder/'summary.json').read_text())[0];allrows=rows(folder)
    data=[r for r in allrows if 'next_pose' in r];early=[r for r in data if r['t']+.05<8.8-1e-8]
    locks=[r for r in data if r['reference']['phase']==1]
    starts=[r for r in locks if r is locks[0] or r['reference']['lock_stamp']!=locks[locks.index(r)-1]['reference']['lock_stamp']]
    out={'name':folder.name,'start':s['start'],'failure':s['failure'],'cycles':len(data),
      'end':data[-1]['t']+.05 if data else None,'first_motion':s['first_motion'] if s['first_motion']>=0 else None,
      'first_2cm_sustained':onset(data,.02),'first_5cm_sustained':onset(data,.05),
      'max_rearward_before_88':max([INITIAL[1]-r['next_pose'][1] for r in early]+[0]),
      'max_rearward_full':max([INITIAL[1]-r['next_pose'][1] for r in data]+[0]),
      'selected_rear_reaching':next((r['t'] for r in data if r['selected_rear_reaching'] is True),None),
      'rear_entry':s['rear_entry'] if s['rear_entry']>=0 else None,'pass_time':s['pass_time'] if s['pass_time']>=0 else None,
      'minimum_clearance':min((r['executed_clearance'] for r in data),default=None),
      'stops':s['stops'],'slow_crawl_seconds':sum(.01<=abs(r['measured'][0])<.05 for r in data)*.05,
      'command_reversals':reversals(data,'selected'),'measured_reversals':reversals(data,'measured'),
      'pure_rotation_seconds':sum(r['selected'][0]==0 and abs(r['selected'][1])>.05 for r in data)*.05,
      'reference_locked_at_end':bool(data and data[-1]['reference']['phase']==1),
      'absolute_yaw_integral':sum(abs(r['measured'][1]) for r in data)*.05,
      'first_lock':None if not locks else {'t':locks[0]['t'],'target':locks[0]['reference']['target']},
      'lock_count':len(starts),'lock_points':[{'t':r['t'],'target':r['reference']['target']} for r in starts],
      'permission_aborts':sum(r['reference']['aborted'] for r in allrows),
      'active_aborts':sum(a['reference']['phase']==1 and b['reference']['aborted'] for a,b in zip(allrows,allrows[1:])),
      'reference_releases':[r['t'] for r in data if r['reference']['released']],
      'risk_releases':[r['t'] for r in data if r['risk']['released']],
      'fallback_losses':sum(not r['fallback_ok'] for r in allrows),
      'rejections':s['rejections'],'final_pose':s['final_pose'],
      'safety_failure':s['failure'] not in ['', 'timeout'],
      'contact':any(r['executed_clearance']<=0 for r in data),
      'intrusion':s['failure']=='executed_contact_or_exclusion_violation'}
    out['abnormal_chatter']=out['stops']>=3 or out['active_aborts']>=3 or out['command_reversals']>8
    return out
