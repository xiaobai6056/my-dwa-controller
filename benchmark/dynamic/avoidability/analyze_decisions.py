#!/usr/bin/env python3
"""Read actual locked Costmap and per-candidate feasibility/scores from original Release calls."""
import argparse,csv,hashlib,json,math
from pathlib import Path
import os

import numpy as np
import matplotlib

# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from kinematics import rollout
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'timing_sweep'))
from rules import response
HERE=Path(__file__).resolve().parent
RELEASE=(ARCHIVE_WS / 'install_compile_comparison_20261020/release/lib/libmy_dwa_controller.so')
def jl(p):return [json.loads(s) for s in p.read_text().splitlines() if s]
def dump(p,z):p.write_text(json.dumps(z,indent=2,allow_nan=False)+'\n')
def stat(z):
    a=np.array(z);return dict(n=len(a),mean=float(np.mean(a)),p95=float(np.percentile(a,95)),max=float(max(a)))
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--tag');args=parser.parse_args()
    allrows=[];runs=[];out=HERE/('outputs_'+args.tag if args.tag else 'outputs');out.mkdir(exist_ok=True)
    for name,start in [('diagnostic_t2_linked',2),('diagnostic_t4_linked',4)]:
        name=(args.tag+'_diagnostic_t'+str(start)) if args.tag else name
        d=HERE/'runs'/name;m=json.loads((d/'manifest.json').read_text());summary=json.loads((d/'trial/summary.json').read_text());epoch=summary['navigation_start_sim_s'];gt=jl(d/'trial/ground_truth.jsonl');contact=min(jl(d/'trial/contacts.jsonl'),key=lambda z:z['stamp'])['stamp']-epoch
        assert m['runtime_mapping_verified'] and m['runtime_plugin_sha256']==hashlib.sha256(RELEASE.read_bytes()).hexdigest()
        assert str(RELEASE) in (d/'plugin_maps.txt').read_text()
        before=json.loads((HERE.parent/'timing_sweep/runs/first_05_t4_mydwa_001/trial/summary.json').read_text())
        assert before['runtime_parameters']==summary['runtime_parameters']
        records=jl(next(d.glob('decisions_*.jsonl')));verified=0;rows=[];errors=[]
        for i,c in enumerate(records):
            diag=c['diagnostic'];cs=c['candidates'];stamp=c['input_stamp'];t=stamp-epoch
            if not diag or not cs:continue
            try:
                assert len(cs)==diag['processed']
                assert sum(z['free'] for z in cs)==diag['valid']
                assert sum(z['scores'] is not None for z in cs)==diag['scored']
                for z in cs:
                    if z['scores']:
                        costs=z['scores'];expected=sum(a*b for a,b in zip(costs[:5],[.1,.1,.6,.2,.05]));assert abs(expected-costs[5])<2e-10
                if not c['threw']:
                    best=cs[c['best_index']];assert abs(best['v']-c['command'][0])<1e-10 and abs(best['w']-c['command'][1])<1e-10
                    assert abs(best['scores'][5]-min(z['scores'][5] for z in cs if z['scores']))<1e-10
                    grid=c['costmap'];costs=[]
                    for x,y,_ in c['best_poses']:
                        mx=int((x-grid['origin'][0])/grid['resolution']);my=int((y-grid['origin'][1])/grid['resolution']);assert 0<=mx<grid['width'] and 0<=my<grid['height'];costs.append(grid['data'][my*grid['width']+mx])
                    assert max(costs)==best['max'] and abs(np.mean(costs)-best['mean'])<1e-8
                verified+=1
            except AssertionError:errors.append(dict(index=i,t=t,error='candidate/score/map consistency mismatch'))
            near=min(enumerate(gt),key=lambda z:(abs(z[1]['t']-stamp),-z[0]))[1];state=near['robot']+[near['vx'],near['wz']]
            brake=rollout(state,t,start,'brake',1,.15,.03,wheel_accel=1)
            hold=rollout(state,t,start,'hold',2.5,0,0)
            valid=[z for z in cs if z['scores'] is not None];slow=None;best=None
            if valid:
                low=min(z['v'] for z in valid);slow=min((z for z in valid if z['v']<=low+1e-5),key=lambda z:z['scores'][5]);best=cs[c['best_index']] if c['best_index']>=0 else None
            row=dict(run=name,index=i,t=t,gt_receipt_relative_s=near['t']-epoch,x=near['robot'][0],actual_v=near['vx'],input_v=c['input'][0],command_v=None if c['threw'] else c['command'][0],command_w=None if c['threw'] else c['command'][1],threw=c['threw'],sampled=diag['sampled'],valid=diag['valid'],scored=diag['scored'],vmin=diag['vmin'],best_index=c['best_index'],conditional_brake_safe=brake['safe'],conditional_gap_lower_m=brake['min_projection_gap_m'],hold_predicted_overlap_s=hold['first_overlap_s'],hold_ttc_s=None if hold['first_overlap_s'] is None else hold['first_overlap_s']-t,compute_ms=c['compute_with_observation_ms'],serialize_ms=c['serialize_ms'],score_ms=diag['score_ms'],best=best,slowest_best=slow)
            rows.append(row);allrows.append(row)
        assert not errors,errors
        safe=[z for z in rows if z['t']>=1.5 and z['t']<contact and z['conditional_brake_safe']]
        # Representative cycle closest to conservative last safe sample, plus earlier clean case.
        latest=max(safe,key=lambda z:z['t']) if safe else None
        chosen=[min(rows,key=lambda z:abs(z['t']-target)) for target in (4.0,4.6,5.,5.5)]
        significant=next((z for z in rows if z['t']>1.5 and z['command_v'] is not None and z['command_v']<=.21),None)
        entry=dict(run=name,crossing_start_s=start,epoch=epoch,physical_contact_s=contact,cycles=len(records),consistency_verified_cycles=verified,consistency_errors=errors,plugin_path=str(RELEASE),plugin_sha256=m['runtime_plugin_sha256'],parameters_match_frozen=True,compute_ms=stat([z['compute_ms'] for z in rows]),serialize_only_ms=stat([z['serialize_ms'] for z in rows]),score_ms=stat([z['score_ms'] for z in rows]),missed_loop_log_count=(d/'launch.log').read_text().count('Control loop missed its desired rate'),successful_output_rate_sim_hz=(lambda s:(len(s)-1)/(max(s)-min(s)))([z['stamp'] for z in records if not z['threw']]),latest_observed_conditional_safe_cycle=latest,first_successful_plugin_output_le_0_21=significant,examples=chosen)
        native=jl(d/'native_commands.jsonl')
        entry['native_sustained_slow_receipt_s']={}
        for topic in ('/cmd_vel_nav','/cmd_vel'):
            observations=[(z['sim'],z['vx'],z['wz']) for z in native if z['topic']==topic and z['sim'] is not None]
            onset=response(observations,epoch+1.5,epoch+contact,'brake')
            entry['native_sustained_slow_receipt_s'][topic]=None if onset is None else onset-epoch
        onset=response([(z['t'],z['vx'],z['wz']) for z in gt],epoch+1.5,epoch+contact,'brake')
        entry['gt_sustained_slow_receipt_s']=None if onset is None else onset-epoch
        runs.append(entry)
        fig,axs=plt.subplots(3,1,figsize=(10,9),sharex=True)
        axs[0].plot([z['t'] for z in rows],[z['command_v'] for z in rows],label='chosen command v');axs[0].plot([z['t'] for z in rows],[z['vmin'] for z in rows],label='minimum sampled v');axs[0].plot([z['t'] for z in rows],[z['actual_v'] for z in rows],label='nearby GT v',alpha=.7);axs[0].legend();axs[0].set_ylabel('m/s')
        axs[1].plot([z['t'] for z in rows],[z['valid'] for z in rows],label='map-feasible candidates');axs[1].set_ylabel('count');axs[1].legend()
        axs[2].plot([z['t'] for z in rows],[z['best']['scores'][5] if z['best'] and z['best']['scores'] else np.nan for z in rows],label='chosen total');axs[2].plot([z['t'] for z in rows],[z['slowest_best']['scores'][5] if z['slowest_best'] else np.nan for z in rows],label='best among slowest candidates');axs[2].legend();axs[2].set_ylabel('weighted cost');axs[2].set_xlabel('time since Nav2 dispatch (s)')
        for ax in axs:
            ax.axvline(contact,color='red',ls=':');ax.grid(alpha=.2)
            if latest:ax.axvline(latest['t'],color='orange',ls='--')
        fig.suptitle(name+'; orange = last sampled conditional safe brake, red = contact');fig.tight_layout();fig.savefig(out/(name+'_decisions.png'),dpi=160);plt.close(fig)
        # Show actual raw costmap at the last conditional safe cycle with both constant-speed forecast endpoints.
        if latest:
            c=records[latest['index']];g=c['costmap'];fig,ax=plt.subplots(figsize=(8,6));arr=np.array(g['data']).reshape(g['height'],g['width']);ox,oy=g['origin'];res=g['resolution'];im=ax.imshow(arr,origin='lower',extent=(ox,ox+g['width']*res,oy,oy+g['height']*res),vmin=0,vmax=255,cmap='magma');path=np.array(c['best_poses']);ax.plot(path[:,0],path[:,1],'c.-',label='selected constant v/w trajectory centers');ax.scatter([c['pose'][0]],[c['pose'][1]],c='lime',label='input pose');ax.legend(fontsize=8);ax.set_title(f'Actual locked costmap; t={latest["t"]:.2f}s (odom frame)');fig.colorbar(im,ax=ax,label='raw cost0..255');fig.tight_layout();fig.savefig(out/(name+'_costmap.png'),dpi=150);plt.close(fig)
    dump(out/'decision_summary.json',runs);dump(out/'decision_cycles.json',allrows)
    flat=[{k:v for k,v in z.items() if k not in ('best','slowest_best')}|{'best_total':None if not z['best'] or not z['best']['scores'] else z['best']['scores'][5],'slowest_total':None if not z['slowest_best'] else z['slowest_best']['scores'][5]} for z in allrows]
    with (out/'decision_cycles.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
    print(json.dumps(runs,indent=2))
if __name__=='__main__':main()