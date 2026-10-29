"""One fixed-parameter parking-baseline / reference-guidance comparison."""
import pathlib,json,subprocess,sys
root=pathlib.Path('/home/user/ros2_experiments/locked_reference_20261029')
phase=sys.argv[1]
starts=[4.] if phase=='phase1' else [6.,8.]
if phase!='phase1':
    screening=json.loads((root/'analysis.json').read_text());assert screening['positive_screen'], 'No authorized positive first-stage evidence'
for start in starts:
    for mode in ['A','B']:
        out=root/'results'/phase/f'{mode}_t{start:g}';out.mkdir(parents=True,exist_ok=False)
        command=[str(root/'build/rolling'),str(root/'input.json'),str(out),mode,str(start)]
        params=json.loads((root/'params/design.json').read_text());params.update({'mode':mode,'start':start})
        (out/'parameters.json').write_text(json.dumps(params,indent=2))
        with (root/'commands.jsonl').open('a') as f:f.write(json.dumps(command)+'\n')
        with (out/'stdout.log').open('w') as f:subprocess.run(command,stdout=f,stderr=subprocess.STDOUT,check=True)
        summary=json.loads((out/'summary.json').read_text())[0];print(mode,start,json.dumps(summary),flush=True)
