#!/usr/bin/env python3
"""Six preregistered restart-isolated D01 runs; immutable raw output directories."""
import argparse
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time
HERE=Path(__file__).resolve().parent
DIAG=HERE.parent

def environment():
    cpufreq={}
    for path in Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq'):
        try:cpufreq[str(path)]=path.read_text().strip()
        except OSError:pass
    return dict(utc=datetime.now(timezone.utc).isoformat(),loadavg=Path('/proc/loadavg').read_text().strip(),cpu_stat=Path('/proc/stat').read_text().splitlines()[0],cpu_frequency_khz=cpufreq,affinity=sorted(os.sched_getaffinity(0)))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag',default='first')
    args=parser.parse_args()
    if Path(args.tag).name!=args.tag or args.tag in ('.','..'):parser.error('tag must be one directory component')
    sequence=HERE/f'sequence_{args.tag}.json'
    if sequence.exists():raise RuntimeError('suite tag already exists')
    schedule=[('unoptimized',1),('release',1),('release',2),('unoptimized',2),('unoptimized',3),('release',3)]
    manifest=dict(order=schedule,clock='paired descriptive runs, steady timing',started=environment(),runs=[])
    sequence.write_text(json.dumps(manifest,indent=2)+'\n')
    for index,(variant,repeat) in enumerate(schedule,1):
        run_id=f'{args.tag}_{index:02}_{variant}_{repeat:03}'
        row=dict(run=run_id,variant=variant,repeat=repeat,order=index,before=environment())
        manifest['runs'].append(row)
        sequence.write_text(json.dumps(manifest,indent=2)+'\n')
        print(f'[{index}/6] {run_id}',flush=True)
        cmd=[sys.executable,str(HERE/'run_one.py'),'--scenario','D01','--controller','mydwa','--headless','--variant',variant,'--run-id',run_id,'--preload',str(DIAG/'perf_build/libnavperf.so')]
        with (HERE/f'evidence/{run_id}_supervisor.log').open('w') as f:
            child=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
        row.update(returncode=child.returncode,after=environment())
        directory=HERE/'runs'/run_id
        if (directory/'manifest.json').exists():
            run_manifest=json.loads((directory/'manifest.json').read_text())
            row['cleanup_returncode']=run_manifest.get('cleanup_returncode',1)
            if child.returncode==0:
                with (directory/'perf_read.log').open('w') as f:
                    read=subprocess.run([sys.executable,str(DIAG/'read_perf.py'),str(directory)],stdout=f,stderr=subprocess.STDOUT)
                row['parser_returncode']=read.returncode
                if read.returncode==0:
                    report=json.loads((directory/'perf_summary.json').read_text())
                    print(f'  wall mean={report["wall_ms"]["mean"]:.3f} ms, call={report["call_hz_steady"]:.2f} Hz, warnings={report["warnings"]}, stop={report["stop_reason"]}',flush=True)
        sequence.write_text(json.dumps(manifest,indent=2)+'\n')
        if row.get('cleanup_returncode',1):raise RuntimeError('cleanup not confirmed; refusing next trial')
        # Same quiet interval for every condition; no concurrent build/test work.
        if index<len(schedule):time.sleep(3)
    manifest['finished']=environment()
    sequence.write_text(json.dumps(manifest,indent=2)+'\n')
    print(sequence,flush=True)
    return int(any(r.get('returncode') or r.get('parser_returncode',1) for r in manifest['runs']))
if __name__=='__main__':raise SystemExit(main())