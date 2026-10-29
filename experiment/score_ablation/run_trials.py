"""Run only specified weights; A prefix must match before the remaining sweep."""
import pathlib,json,subprocess,sys,hashlib
root=pathlib.Path('/home/user/ros2_experiments/score_ablation_20261029')
base=root.parent/'rolling_bypass_20261029'
phase=sys.argv[1]
if phase=='phase1':trials=[(s,4.) for s in 'ABCDEFG']
elif phase=='phase2':
    schemes=sys.argv[2:];assert schemes and all(s in 'BCDEFG' for s in schemes)
    trials=[(s,t) for t in [6.,8.] for s in ['A',*schemes]]
else:raise ValueError(phase)
for scheme,start in trials:
    out=root/'results'/phase/f'{scheme}_t{start:g}';out.mkdir(parents=True,exist_ok=False)
    command=[str(root/'build/rolling'),str(root/'input.json'),str(out),scheme,str(start)]
    params=json.loads((root/'params'/f'{scheme}.json').read_text());params['start']=start
    params['common_input_sha256']=hashlib.sha256((root/'input.json').read_bytes()).hexdigest()
    (out/'parameters.json').write_text(json.dumps(params,indent=2))
    with (root/'commands.jsonl').open('a') as f:f.write(json.dumps(command)+'\n')
    with (out/'stdout.log').open('w') as f:subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,check=True)
    result=json.loads((out/'summary.json').read_text())[0]
    if scheme=='A':
        old=list(map(json.loads,(base/f'results/start_{start:.2f}.jsonl').read_text().splitlines()))
        new=list(map(json.loads,(out/f'start_{start:.2f}.jsonl').read_text().splitlines()))
        mismatches=[(i,k) for i,(a,b) in enumerate(zip(old,new)) for k,v in a.items() if b.get(k)!=v]
        assert len(new)>=len(old) and not mismatches,mismatches[:5]
        previous=next(s for s in json.loads((base/'results/summary.json').read_text()) if s['start']==start)
        assert result['rear_entry']==previous['rear_entry'] and result['pass_time']==previous['pass_time']
        (out/'baseline_parity.json').write_text(json.dumps({'original_prefix_cycles':len(old),'new_cycles':len(new),
          'mismatches':mismatches,'entry':result['rear_entry'],'pass':result['pass_time']},indent=2))
    print(phase,scheme,start,json.dumps(result),flush=True)
