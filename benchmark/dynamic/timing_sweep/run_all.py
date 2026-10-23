#!/usr/bin/env python3
"""Thirty restart-isolated actual D01 trials, all attempts retained."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from rules import ROOT,selection,scene,verify_motion
HERE=Path(__file__).resolve().parent

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):return json.loads(p.read_text())
def jsonl(p):return [json.loads(l) for l in p.read_text().splitlines() if l]
def save(p,value):p.write_text(json.dumps(value,indent=2)+'\n')
def check_sources():
    source=load(ROOT/'benchmark/dynamic/diagnostics/release_comparison/evidence/before.json')['source']
    assert all(sha(ROOT/name)==digest for name,digest in source.items()),'Release source no longer matches current source'
    frozen=load(HERE/'evidence/before.json')['files']
    for controller in ('mydwa','dwb'):
        _,plugin=selection(controller)
        assert sha(plugin)==frozen[str(plugin)],'selected library changed'

def environment():
    return dict(utc=datetime.now(timezone.utc).isoformat(),loadavg=Path('/proc/loadavg').read_text().strip(),affinity=sorted(os.sched_getaffinity(0)))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag',default='first')
    args=parser.parse_args()
    if Path(args.tag).name!=args.tag or args.tag in ('.','..'):parser.error('tag must be one directory component')
    dest=HERE/f'sequence_{args.tag}.json'
    if dest.exists():raise RuntimeError('refusing existing suite tag')
    check_sources()
    if os.environ.get('LD_PRELOAD'):raise RuntimeError('unexpected external preload')
    timings={1:[2,3,4,5,6],2:[6,5,4,3,2],3:[4,2,6,3,5]}
    schedule=[]
    for repeat,points in timings.items():
        for index,start in enumerate(points):
            order=['mydwa','dwb'] if (repeat+index)%2 else ['dwb','mydwa']
            for controller in order:schedule.append(dict(start_time_s=start,controller=controller,repeat=repeat))
    manifest=dict(classification='fresh_release_timing_scan_not_v01',scheduled=schedule,started=environment(),runs=[])
    save(dest,manifest)
    evidence=HERE/'evidence'
    binaries={controller:dict(path=str(selection(controller)[1]),sha256=sha(selection(controller)[1])) for controller in ('mydwa','dwb')}
    save(HERE/'evidence/selected_binaries.json',binaries)
    for index,planned in enumerate(schedule,1):
        check_sources()
        run_id=f'{args.tag}_{index:02}_t{planned["start_time_s"]}_{planned["controller"]}_{planned["repeat"]:03}'
        row=dict(**planned,order=index,run=run_id,before=environment())
        manifest['runs'].append(row);save(dest,manifest)
        print(f'[{index}/30] {run_id}',flush=True)
        command=[sys.executable,str(HERE/'run_one.py'),'--scenario','D01','--controller',planned['controller'],'--headless','--start-time',str(planned['start_time_s']),'--run-id',run_id]
        with (evidence/f'{run_id}_supervisor.log').open('w') as f:
            child=subprocess.run(command,stdout=f,stderr=subprocess.STDOUT)
        row.update(returncode=child.returncode,after=environment())
        d=HERE/'runs'/run_id
        if (d/'manifest.json').exists():
            own=load(d/'manifest.json');row['cleanup_returncode']=own.get('cleanup_returncode',1)
        if (d/'trial/summary.json').exists():
            summary=load(d/'trial/summary.json')
            if summary.get('navigation_start_sim_s') is not None:
                validation=verify_motion(jsonl(d/'trial/ground_truth.jsonl'),jsonl(d/'trial/motion.jsonl'),scene(planned['start_time_s']),summary['navigation_start_sim_s'])
                save(d/'motion_validation.json',validation)
                row['motion_valid']=validation['valid']
                row['observed_start_s']=validation['observed_first_gt_motion_s']
            row.update(action=summary.get('action_result'),valid=summary.get('valid'),stop_reason=summary.get('stop_reason'))
            print(f'  action={row.get("action")}, stop={row.get("stop_reason")}, actual motion start={row.get("observed_start_s")}, motion_valid={row.get("motion_valid")}',flush=True)
        save(dest,manifest)
        if row.get('cleanup_returncode',1):raise RuntimeError('cleanup unconfirmed; refusing contaminated next trial')
        if index<len(schedule):time.sleep(2)
    manifest['finished']=environment();save(dest,manifest)
    print(dest,flush=True)
    return int(any(r.get('returncode') or not r.get('motion_valid') for r in manifest['runs']))
if __name__=='__main__':raise SystemExit(main())