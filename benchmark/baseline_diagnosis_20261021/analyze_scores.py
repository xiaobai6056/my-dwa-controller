"""Export original decisions and offline tail statistics; do not define a new score."""
import argparse
import csv
import gzip
import json
from pathlib import Path
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from scoring import rollout, sample_statistics, verify_observation, weighted_terms, selection_order, admission_errors

HERE=Path(__file__).resolve().parent
TERMS=['O','S','G','P','H']


def read(path):return json.loads(path.read_text())


def dump(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def describe(values):
    a=np.asarray(values,dtype=float)
    return dict(n=len(a),mean=float(a.mean()),p50=float(np.median(a)),
        p95=float(np.percentile(a,95)),max=float(a.max())) if len(a) else dict(n=0)


def sources():
    old=HERE.parent/'dynamic/avoidability/runs'
    candidates=[(name,old/name) for name in ['diagnostic_t2_linked','diagnostic_t4_linked']]
    for p in sorted((HERE/'runs').iterdir()):
        if p.is_dir() and list(p.glob('decisions_*.jsonl')) and (p/'trial/summary.json').exists():
            candidates.append((p.name,p))
    included=[];excluded=[]
    for label,p in candidates:
        legacy=p.parent==old and label in ('diagnostic_t2_linked','diagnostic_t4_linked')
        reasons=admission_errors(read(p/'trial/summary.json'),read(p/'manifest.json') if (p/'manifest.json').exists() else None,legacy=legacy)
        if not legacy and not (p/'bag/metadata.yaml').exists():reasons.append('missing_bag_metadata')
        if reasons:excluded.append(dict(run=label,path=str(p),reasons=reasons))
        else:included.append((label,p))
    return included,excluded


def analyze(label, directory, k, allwriter):
    summary=read(directory/'trial/summary.json');epoch=summary['navigation_start_sim_s']
    params=summary['runtime_parameters']['controller_server']
    weights=[params['FollowPath.'+name+'_weight'] for name in ['obstacle','speed','goal','path','heading']]
    sim_time=params['FollowPath.sim_time'];dt=params['FollowPath.simulation_dt']
    contacts=directory/'trial/contacts.jsonl'
    contact=min((json.loads(s)['stamp'] for s in contacts.read_text().splitlines()),default=None) if contacts.exists() else None
    cycles=[];keyrows=[];errors=[];checked=0;rejected_checked=0;unreconstructable=0;raw_count=0
    for file in sorted(directory.glob('decisions_*.jsonl')):
        with file.open() as stream:
            for number,line in enumerate(stream):
                c=json.loads(line);diag=c.get('diagnostic') or {};obs=c['candidates']
                raw_count+=1
                t=c['input_stamp']-epoch;phase=diag.get('goal_phase','unknown')
                initial=(c.get('rollout_initial_pose') or c.get('best_poses') or [None])[0]
                if not obs or initial is None:
                    unreconstructable+=1
                    cycles.append(dict(run=label,cycle=number,t=t,input_stamp=c['input_stamp'],
                        original_command=c['command'] if not c['threw'] else None,threw=c['threw'],
                        valid=sum(z['scores'] is not None for z in obs),evaluated=len(obs),sampled=diag.get('sampled'),
                        goal_phase=phase,ordinary_score=False,pre_contact=contact is None or c['input_stamp']<contact,
                        compute_ms=c['compute_with_observation_ms'],serialize_ms=c['serialize_ms'],score_ms=diag.get('score_ms'),
                        selected=None,key_candidates={},obstacle_term_decisive=None,
                        reconstruction_limit='Old probe omitted exact initial pose when no best trajectory existed.'))
                    continue
                predicted=rollout(initial,[[z['v'],z['w']] for z in obs],sim_time,dt)
                stats=sample_statistics(predicted,c['costmap'],k)
                available=[i for i,z in enumerate(obs) if z['scores'] is not None]
                if not available:available=[]
                best=c['best_index'] if not c['threw'] and c['best_index']>=0 else None
                scored_order=selection_order(obs,phase)
                ordinary=(phase=='normal' or (phase=='unknown' and available and all(
                    abs(sum(weighted_terms(obs[i]['scores'][:5],weights))-obs[i]['scores'][5])<1e-7 for i in available)))
                roles={}
                if best is not None:roles['selected']=best
                if available:
                    roles['fastest_best']=min(available,key=lambda i:(-obs[i]['v'],obs[i]['scores'][5],abs(obs[i]['w'])))
                    roles['slowest_best']=min(available,key=lambda i:(obs[i]['v'],obs[i]['scores'][5],abs(obs[i]['w'])))
                    roles['least_obstacle']=min(available,key=lambda i:(obs[i]['mean'],obs[i]['scores'][5]))
                    other=[i for i in scored_order if i!=best]
                    if other and (ordinary or phase in ('slowing','rotating')):roles['runner_up']=other[0]
                rows=[]
                for i,(z,s) in enumerate(zip(obs,stats)):
                    mismatch=verify_observation(z,predicted[i],s)
                    # Rejected raw returns intentionally have mean/max==0; no fake checks.
                    if mismatch:errors.append(dict(file=file.name,cycle=number,candidate=i,t=t,errors=mismatch,
                        boundary_distance_m=s['minimum_grid_boundary_distance_m']))
                    if z['scores'] is not None:checked+=1
                    else:rejected_checked+=1
                    raw=z['scores'][:5] if z['scores'] else [None]*5
                    contributions=weighted_terms(raw,weights) if z['scores'] else [None]*5
                    row=dict(run=label,cycle=number,t=t,input_stamp=c['input_stamp'],candidate=i,
                        roles=';'.join(name for name,index in roles.items() if index==i),v=z['v'],w=z['w'],
                        original_free=z['free'],original_rejection=z['rejection'],goal_phase=phase,
                        ordinary_score=bool(ordinary),original_mean=z['mean'],original_max=z['max'],
                        original_total=z['scores'][5] if z['scores'] else None,
                        reconstruction_verified=not mismatch,**s)
                    for j,name in enumerate(TERMS):row['raw_'+name]=raw[j];row['weighted_'+name]=contributions[j]
                    rows.append(row);allwriter(row)
                    if row['roles']:keyrows.append(row)
                    if ordinary and z['scores'] and abs(sum(contributions)-z['scores'][5])>1e-7:
                        errors.append(dict(file=file.name,cycle=number,candidate=i,t=t,errors=['weighted_total']))
                cycle=dict(run=label,cycle=number,t=t,input_stamp=c['input_stamp'],
                    original_command=c['command'] if not c['threw'] else None,threw=c['threw'],
                    valid=len(available),evaluated=len(obs),sampled=diag.get('sampled'),goal_phase=phase,
                    ordinary_score=bool(ordinary),pre_contact=contact is None or c['input_stamp']<contact,
                    compute_ms=c['compute_with_observation_ms'],serialize_ms=c['serialize_ms'],
                    score_ms=diag.get('score_ms'),selected=None if best is None else rows[best],
                    key_candidates={role:rows[i] for role,i in roles.items()},obstacle_term_decisive=None)
                if best is not None:
                    if ordinary:
                        if obs[best]['scores'][5]>min(obs[i]['scores'][5] for i in available)+1e-7:
                            errors.append(dict(cycle=number,t=t,errors=['selected_not_minimum']))
                        other_without=min(available,key=lambda i:obs[i]['scores'][5]-weights[0]*obs[i]['scores'][0])
                        no_o_chosen=obs[best]['scores'][5]-weights[0]*obs[best]['scores'][0]
                        no_o_other=obs[other_without]['scores'][5]-weights[0]*obs[other_without]['scores'][0]
                        cycle['obstacle_term_decisive']=no_o_other<no_o_chosen-1e-10
                        cycle['obstacle_term_decisive_definition']='Deleting only existing O contribution changes the strict score preference; descriptive attribution, never executed online.'
                    if phase in ('slowing','rotating') and scored_order and best!=scored_order[0]:
                        errors.append(dict(cycle=number,t=t,errors=['terminal_selection_order']))
                    slower=roles.get('slowest_best')
                    if slower is not None and ordinary:
                        cycle['slowest_minus_selected']={name:rows[slower]['weighted_'+name]-rows[best]['weighted_'+name] for name in TERMS}
                        cycle['slowest_minus_selected']['total']=rows[slower]['original_total']-rows[best]['original_total']
                cycles.append(cycle)
    selected=[z['selected'] for z in cycles if z['selected'] and z['selected']['reconstruction_verified']]
    ordinary_pre=[z for z in cycles if z['ordinary_score'] and z['pre_contact'] and z['selected']]
    old_context=HERE.parent/'dynamic/avoidability/outputs/decision_summary.json'
    historical=next((z for z in read(old_context) if z['run']==label),None)
    target=historical['latest_observed_conditional_safe_cycle']['t'] if historical and historical.get('latest_observed_conditional_safe_cycle') else ((contact-epoch-1.) if contact else None)
    representative=min((z for z in ordinary_pre),key=lambda z:abs(z['t']-target)) if ordinary_pre and target is not None else None
    if representative is None and ordinary_pre:
        representative=max(ordinary_pre,key=lambda z:z['selected']['max']-z['selected']['mean'])
    result=dict(run=label,source=str(directory),historical=HERE not in directory.parents,
        source_valid=summary.get('valid'),status=summary.get('status'),action_result=summary.get('action_result'),
        epoch=epoch,contact_source_relative_s=None if contact is None else contact-epoch,
        weights=weights,k=k,scored_candidates_checked=checked,rejected_candidates_checked=rejected_checked,
        raw_cycles_read=raw_count,analyzed_cycles=len(cycles),unreconstructable_cycles=unreconstructable,consistency_errors=errors,
        obstacle_term_decisive_cycles=sum(z['obstacle_term_decisive'] is True for z in ordinary_pre),
        ordinary_pre_contact_cycles=len(ordinary_pre),selected_zero_mean_cycles=sum(z['mean']==0 for z in selected),
        selected_peak_minus_mean=describe([z['max']-z['mean'] for z in selected]),
        selected_topk_sample_minus_mean=describe([z['topk_sample_mean']-z['mean'] for z in selected]),
        selected_topk_unique_minus_mean=describe([z['topk_unique_mean']-z['mean'] for z in selected]),
        selected_different_cells=describe([z['different_cells'] for z in selected]),
        compute_ms=describe([z['compute_ms'] for z in cycles]),
        serialization_construction_ms=describe([z['serialize_ms'] for z in cycles]),
        control_loop_missed_log_count=(directory/'launch.log').read_text().count('Control loop missed its desired rate') if (directory/'launch.log').exists() else None,
        representative=representative,
        limits=['Top-K describes tails only: no replacement score, ranking or control was implemented.',
            'Original rejected statistics default to zero; offline rejected statistics never entered selection.',
            'ABI map captures have no actual Costmap timestamp; use separately stamped map stream for temporal claims.',
            'Observer computation excludes final write/flush time; serialization construction is separate, not complete overhead.'])
    fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True)
    t=[z['t'] for z in cycles]
    axes[0].plot(t,[z['selected']['v'] if z['selected'] else np.nan for z in cycles],label='original selected v')
    axes[0].plot(t,[z['key_candidates'].get('slowest_best',{}).get('v',np.nan) for z in cycles],label='lowest feasible v',alpha=.7)
    axes[0].set_ylabel('m/s');axes[0].legend()
    for name in ['mean','max','topk_sample_mean','topk_unique_mean']:
        axes[1].plot(t,[z['selected'][name] if z['selected'] else np.nan for z in cycles],label=name)
    axes[1].set_ylabel('raw map cost');axes[1].legend(fontsize=8)
    for name in TERMS:
        axes[2].plot(t,[z['selected']['weighted_'+name] if z['selected'] else np.nan for z in cycles],label=name)
    axes[2].set_ylabel('weighted original terms');axes[2].set_xlabel('seconds since navigation dispatch');axes[2].legend()
    for ax in axes:
        ax.grid(alpha=.2)
        if contact is not None:ax.axvline(contact-epoch,color='r',ls=':',label='physical contact')
        if target is not None:ax.axvline(target,color='orange',ls='--')
    fig.suptitle(label+' — descriptive Top-K only; original decisions preserved');fig.tight_layout()
    figures=HERE/'outputs/figures';figures.mkdir(exist_ok=True)
    fig.savefig(figures/(label+'_scores.png'),dpi=150);plt.close(fig)
    return result,cycles,keyrows


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--k',type=int,default=5);a=p.parse_args()
    out=HERE/'outputs';out.mkdir(exist_ok=True)
    summaries=[];allcycles=[];keys=[];writer=None
    with gzip.open(out/'all_candidates.csv.gz','wt',newline='') as stream:
        def emit(row):
            nonlocal writer
            if writer is None:writer=csv.DictWriter(stream,fieldnames=list(row));writer.writeheader()
            writer.writerow(row)
        input_sources,excluded=sources()
        dump(out/'score_excluded_runs.json',excluded)
        for label,directory in input_sources:
            result,cycles,keyrows=analyze(label,directory,a.k,emit)
            summaries.append(result);allcycles.extend(cycles);keys.extend(keyrows)
            print(label,'scored',result['scored_candidates_checked'],'errors',len(result['consistency_errors']),flush=True)
    dump(out/'score_summary.json',summaries)
    with gzip.open(out/'score_cycles.jsonl.gz','wt') as f:
        for c in allcycles:f.write(json.dumps(c,allow_nan=False)+'\n')
    if keys:
        with (out/'key_candidates.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(keys[0]));w.writeheader();w.writerows(keys)
    dump(out/'score_verification.json',dict(runs=len(summaries),
        scored_candidates_checked=sum(z['scored_candidates_checked'] for z in summaries),
        rejected_candidates_checked=sum(z['rejected_candidates_checked'] for z in summaries),
        mismatches=sum(len(z['consistency_errors']) for z in summaries),
        k=a.k,topk_definition='mean largest min(K,N) costs; separately pose-weighted and unique-cell-weighted'))


if __name__=='__main__':main()
