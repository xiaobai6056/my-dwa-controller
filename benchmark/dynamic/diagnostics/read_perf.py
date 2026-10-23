#!/usr/bin/env python3
"""Read committed mmap samples and compare equivalent outer plugin functions."""
import argparse
import csv
import json
import math
from pathlib import Path
import struct

from analyze import distribution

HEADER = struct.Struct('<8s7Q')
RECORD = struct.Struct('<QQQd7dIIQQQQdd')
STAGES = ('prefix_to_report', 'state_wait', 'path', 'map_wait', 'predict', 'obstacle', 'score')


def read(path):
    blob = path.read_bytes()
    magic, count, dropped, size, capacity, *_ = HEADER.unpack_from(blob)
    if magic != b'NAVPERF2' or size != RECORD.size:
        raise ValueError('unsupported performance observation layout')
    rows = []
    for index in range(min(count, capacity)):
        values = RECORD.unpack_from(blob, HEADER.size+index*size)
        start, end, cpu, ros, *tail = values
        stages = tail[:7]
        controller, threw, processed, valid, scored, _, vx, wz = tail[7:]
        if end < start or not end:
            continue
        rows.append(dict(start_ns=start, end_ns=end, wall_ms=(end-start)*1e-6,
            thread_cpu_ms=cpu*1e-6, ros_stamp=ros, controller=controller, threw=threw,
            processed=processed, valid=valid, scored=scored, vx=vx, wz=wz,
            **{key:(value if math.isfinite(value) else None) for key,value in zip(STAGES,stages)}))
    return rows, dict(reserved=count, committed=len(rows), dropped=dropped)


def analyze(directory):
    rows, integrity = [], []
    for path in sorted(directory.glob('perf_*.bin')):
        data, header = read(path)
        rows += data
        integrity.append(dict(file=path.name, **header))
    if not rows:
        raise RuntimeError('No intercepted computeVelocityCommands samples; do not claim profiling success')
    rows.sort(key=lambda row: row['start_ns'])
    with (directory / 'perf_samples.csv').open('w', newline='') as output:
        writer = csv.DictWriter(output, rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    result = dict(integrity=integrity, clock='steady_clock wall and CLOCK_THREAD_CPUTIME_ID',
        wall_ms=distribution([r['wall_ms'] for r in rows]),
        thread_cpu_ms=distribution([r['thread_cpu_ms'] for r in rows]),
        call_hz_steady=((len(rows)-1)*1e9/(rows[-1]['start_ns']-rows[0]['start_ns']) if len(rows)>1 else None),
        exceptions=sum(r['threw'] for r in rows),
        returning_wall_ms=distribution([r['wall_ms'] for r in rows if not r['threw']]),
        throwing_wall_ms=distribution([r['wall_ms'] for r in rows if r['threw']]),
        stage_ms={stage:distribution([r[stage] for r in rows if r[stage] is not None]) for stage in STAGES},
        note='Diagnostic mmap interposition; original plugin forwarded once, no parameter changes. '
             'CPU excludes descheduling; wall includes it. Stage values copied from existing MyDWA observer only.')
    native = [json.loads(line) for line in (directory / 'native_commands.jsonl').read_text().splitlines()]
    summary = json.loads((directory / 'trial' / 'summary.json').read_text())
    begin, end = summary['navigation_start_sim_s'], summary['navigation_end_sim_s']
    result['output_hz'] = {}
    for topic in ('/cmd_vel_nav', '/cmd_vel'):
        samples = [r for r in native if r['topic']==topic and r['sim'] is not None and begin <= r['sim'] <= end]
        result['output_hz'][topic] = dict(n=len(samples),
            steady_receipt_hz=((len(samples)-1)*1e9/(samples[-1]['monotonic_ns']-samples[0]['monotonic_ns']) if len(samples)>1 else None),
            sim_receipt_hz=((len(samples)-1)/(samples[-1]['sim']-samples[0]['sim']) if len(samples)>1 and samples[-1]['sim']>samples[0]['sim'] else None))
    result['warnings'] = (directory / 'launch.log').read_text().count('Control loop missed')
    result['action_result'] = summary['action_result']
    result['stop_reason'] = summary['stop_reason']
    result['plugin_sha256'] = json.loads((directory / 'manifest.json').read_text())['current_plugin_sha256']
    (directory / 'perf_summary.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    analyze(parser.parse_args().directory)