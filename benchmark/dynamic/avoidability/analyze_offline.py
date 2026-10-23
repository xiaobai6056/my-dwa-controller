#!/usr/bin/env python3
"""Replay all24 existing collisions against longitudinal counterfactuals; no simulation control."""
import json,csv,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from kinematics import rollout,geometry,BOUNDS,SDF
HERE=Path(__file__).resolve().parent
SWEEP=HERE.parent/'timing_sweep'
def jl(p):return [json.loads(s) for s in p.read_text().splitlines() if s]
def write(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def main():
    out=HERE/'outputs';out.mkdir(exist_ok=True)
    write(HERE/'evidence/actual_model.json',dict(path=str(SDF),sha256=hashlib.sha256(SDF.read_bytes()).hexdigest(),bounds=BOUNDS,parts=geometry(),wheel_acceleration_m_s2=1.0,note='Actual default robot_sdf from installed tb3_simulation_launch; independent of stock turtlebot3 model. Test front bound corrected from assumed .077 to extracted .073.'))
    rows=[];summaries=[]
    for r in json.loads((SWEEP/'outputs/results.json').read_text())['records']:
        if r['start_time_s']>5:continue
        trial=SWEEP/'runs'/r['run']/'trial';summary=json.loads((trial/'summary.json').read_text());epoch=summary['navigation_start_sim_s'];gt=jl(trial/'ground_truth.jsonl')
        contact=min(jl(trial/'contacts.jsonl'),key=lambda z:z['stamp'])['stamp']-epoch
        unique={round(z['t']-epoch,6):z for z in gt};times=sorted(unique)
        local=[]
        for t in times:
            if t<1.5 or t>=contact-.1:continue
            z=unique[t];state=z['robot']+[z['vx'],z['wz']]
            for label,policy,a,lag,margin in [('hold','hold',2.5,0,0),('ideal_brake','brake',2.5,0,0),('physical_conditional','brake',1,.15,.03),('uncertainty_sensitivity','brake',1,.15,.12)]:
                result=rollout(state,t,r['start_time_s'],policy,a,lag,margin,wheel_accel=1 if a==1 else None)
                row=dict(run=r['run'],controller=r['controller'],start_time_s=r['start_time_s'],repeat=r['repeat'],decision_s=t,policy=label,x=z['robot'][0],v=z['vx'],**result)
                rows.append(row);local.append(row)
        entry=dict(run=r['run'],controller=r['controller'],start_time_s=r['start_time_s'],repeat=r['repeat'],physical_contact_s=contact,windows={})
        for policy in ('ideal_brake','physical_conditional','uncertainty_sensitivity'):
            sequence=[z for z in local if z['policy']==policy]
            # Initial safe prefix only; later isolated safe pockets are kept in raw rows.
            last=None;first_bad=None
            for z in sequence:
                if not z['safe']:first_bad=z['decision_s'];break
                last=z
            entry['windows'][policy]=dict(latest_sample_safe_s=None if last is None else last['decision_s'],next_sample_unsafe_s=first_bad,braking_distance_m=None if last is None else last['braking_distance_m'],remaining_projection_margin_m=None if last is None else last['min_projection_gap_m'],x=None if last is None else last['x'])
        summaries.append(entry)
    write(out/'offline_results.json',dict(geometry_bounds=BOUNDS,decisions=rows,runs=summaries,model='yaw-only full collision AABB enclosure; prescribed obstacle continues full crossing; dt .01s; obstacle-only risk, no wall checks; SAT projection gap lower bound. Physical_conditional wheel-coordinated a1, latency.15, margin.03 is conditional nominal result. Uncertainty_sensitivity margin.12 covers assumed .026m translation + .025m yaw support + .06m obstacle schedule envelope, rounded up; not timestamp-certified bound. Simulator execution validated separately.'))
    with (out/'offline_decisions.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    fig,axs=plt.subplots(2,4,figsize=(15,7),sharex=True,sharey=True)
    for j,c in enumerate(('mydwa','dwb')):
        for i,start in enumerate((2,3,4,5)):
            ax=axs[j,i]
            for repeat in (1,2,3):
                for label,color,style in [('hold','gray',':'),('ideal_brake','tab:blue','-'),('physical_conditional','tab:orange','--'),('uncertainty_sensitivity','tab:green','-.')]:
                    rs=[z for z in rows if z['controller']==c and z['start_time_s']==start and z['repeat']==repeat and z['policy']==label]
                    ax.plot([z['decision_s'] for z in rs],[z['min_projection_gap_m'] for z in rs],color=color,ls=style,alpha=.6,label=label if repeat==1 else None)
            ax.axhline(0,color='red',lw=.8);ax.set_title(f'{c} crossing {start}s');ax.set_ylim(-.3,.65);ax.set_xlim(1.5,7.5);ax.set_xlabel('decision time from dispatch (s)')
            if i==0:ax.set_ylabel('minimum SAT gap lower bound (m)')
    axs[0,0].legend(fontsize=7);fig.tight_layout();fig.savefig(out/'braking_risk.png',dpi=160);plt.close(fig)
    print(json.dumps(summaries,indent=2))
if __name__=='__main__':main()