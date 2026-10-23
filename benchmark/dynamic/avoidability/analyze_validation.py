#!/usr/bin/env python3
import argparse,json,csv
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from kinematics import rollout,gaps
HERE=Path(__file__).resolve().parent

def jl(p):return [json.loads(s) for s in p.read_text().splitlines() if s]
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--tag');args=parser.parse_args()
    out=HERE/('outputs_'+args.tag if args.tag else 'outputs');out.mkdir(exist_ok=True)
    rows=[];fig,axs=plt.subplots(3,2,figsize=(12,10))
    runs=['validation_t2_early_fixed','validation_t4_early','validation_t4_late']
    for i,name in enumerate(runs):
        name=(args.tag+'_'+['t2_early','t4_early','t4_late'][i]) if args.tag else name
        directory=HERE/'runs'/name
        if not (directory/'manifest.json').exists():continue
        m=json.loads((directory/'manifest.json').read_text());epoch=m['epoch'];brake=m['brake_event'];tbr=brake['relative_s'];gt=jl(directory/'ground_truth.jsonl');cmd=jl(directory/'commands.jsonl');odom=jl(directory/'odom.jsonl');contact=jl(directory/'contacts.jsonl')
        nearest=lambda t:min(enumerate(gt),key=lambda z:(abs(z[1]['t']-epoch-t),-z[0]))[1]
        g0=nearest(tbr);future=[z for z in gt if z['t']-epoch>=tbr+.1 and z['t']-epoch<m['arguments']['start']+9.3]
        stopped=next((z for z in future if abs(z['vx'])<.01 and all(abs(n['vx'])<.01 for n in future if z['t']<=n['t']<=z['t']+.2) and any(n['t']>=z['t']+.19 for n in future)),None)
        decisionstate=g0['robot']+[g0['vx'],g0['wz']]
        model=rollout(decisionstate,tbr,m['arguments']['start'],'brake',1,.15,.03,trace=True,wheel_accel=1)
        shape=np.array([z['robot'] for z in gt]);times=np.array([z['t']-epoch for z in gt]);clear=gaps(shape[:,0],shape[:,1],shape[:,2],times,m['arguments']['start'])
        row=dict(run=name,crossing_start_s=m['arguments']['start'],brake_config_s=m['arguments']['brake'],brake_observed_receipt_s=tbr,brake_x_m=brake['robot'][0],brake_v_m_s=brake['vx'],termination=m['termination'],physical_contact=bool(contact),first_contact_s=None if not contact else min(z['stamp'] for z in contact)-epoch,stop_receipt_s=None if stopped is None else stopped['t']-epoch,stop_x_m=None if stopped is None else stopped['robot'][0],stop_distance_m=None if stopped is None else stopped['robot'][0]-brake['robot'][0],model_safe=model['safe'],model_gap_lower_m=model['min_projection_gap_m'],measured_enclosure_min_gap_m=float(min(clear)),motion_valid=m['motion_validation']['valid'],cmd_publishers=m['cmd_vel_publishers'])
        # Source-stamped odometry gives denser response; command event itself remains +/- clock-bin uncertainty.
        os=[z for z in odom if epoch+tbr-.25<=z['stamp']<=epoch+tbr+.65]
        row['odom_response']=os
        rows.append(row)
        ax=axs[i,0];ax.plot(times,[z['vx'] for z in gt],label='GT executed v');ax.plot([z['relative_s'] for z in cmd],[z['vx'] for z in cmd],label='command v',alpha=.6);ax.axvline(tbr,color='red',ls=':');ax.set_title(name);ax.set_ylabel('m/s');ax.legend();ax.grid(alpha=.2)
        ax=axs[i,1];ax.plot(times,shape[:,0],label='executed robot x');tr=model['trace'];ax.plot(tr['t'],tr['x'],'--',label='conditional brake prediction');ax.axhspan(1.35,1.85,color='orange',alpha=.2,label='obstacle x slab');ax.set_ylabel('center x (m)');ax.legend();ax.grid(alpha=.2)
    for ax in axs[-1,:]:ax.set_xlabel('relative simulation time (s)')
    fig.tight_layout();fig.savefig(out/'validation.png',dpi=150);plt.close(fig)
    (out/'validation_results.json').write_text(json.dumps(rows,indent=2)+'\n')
    flat=[{k:v for k,v in z.items() if k!='odom_response'} for z in rows]
    if flat:
        with (out/'validation_results.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
    print(json.dumps(flat,indent=2))
if __name__=='__main__':main()