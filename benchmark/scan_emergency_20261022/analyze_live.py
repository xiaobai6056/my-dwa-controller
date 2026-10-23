"""Reuse old collectors, adding motion validity and fixed-region release evidence."""
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    'prior_analysis', HERE.parent / 'scan_trend_20261022/analyze.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)


def analyze(directory):
    """Keep raw estimates separate from accepted motion and actual robot behavior."""
    result = prior.analyze(directory)
    if not result.get('outcome', {}).get('valid'):
        return result
    epoch = result['epoch']
    risk = prior.log_records(directory / 'launch.log', '[PATH_CONFLICT]')
    tracks = prior.log_records(directory / 'launch.log', '[SCAN_TRACK]')
    result['first_local_clear'] = next((dict(t=r['t']-epoch, line=r['line'])
                                       for r in risk if r.get('area_clear')), None)
    result['release'] = [dict(t=r['t']-epoch, line=r['line'])
                         for r in risk if r.get('release')]
    result['accepted_moving_samples'] = sum(t.get('motion') == 'moving' for t in tracks)
    result['unknown_samples'] = sum(t.get('motion') == 'unknown' for t in tracks)
    result['monitor_ms'] = {
        'median': float(np.median([r['ms'] for r in risk])),
        'max': max(r['ms'] for r in risk)}
    if directory.name.endswith('S05'):
        result['static_accepted_moving_samples'] = sum(
            t.get('motion') == 'moving' and t['observed'] for t in tracks)
        return result
    import csv
    actor = list(csv.DictReader((directory / 'actor_aligned.csv').open()))
    actor = [{k: (float(v) if k != 'motion' and k != 'state' else v)
              for k, v in r.items()} for r in actor]
    valid = [r for r in actor if r['motion'] == 'moving' and
             result['motion_start'] <= r['elapsed'] <= result['motion_start'] + 8.8]
    result['accepted_actor_motion'] = prior.stats(valid)
    result['accepted_conflict_samples'] = sum(r['conflict'] for r in valid)
    result['accepted_motion_samples'] = [
        dict(elapsed=r['elapsed'], vx=r['world_vx'], vy=r['world_vy'],
             confidence=r['confidence'], support=r['support']) for r in valid]
    gt = prior.rows(directory / 'trial/ground_truth.jsonl')
    begin = next((r['t'] for r in risk if r['state'] != 'normal'), math.inf)
    stopped = [g for g in gt if g['t'] > begin and abs(g['vx']) < .01]
    result['first_stop'] = next((dict(t=g['t']-epoch, robot=g['robot'], vx=g['vx'])
                                 for g in stopped), None)
    release = result['release'][0]['t']+epoch if result['release'] else math.inf
    result['first_move_after_release'] = next((dict(t=g['t']-epoch, robot=g['robot'], vx=g['vx'])
                                              for g in gt
                                              if g['t'] > release and g['vx'] > .05), None)
    result['minimum_collector_gap_approx'] = min(g['gap_approx_m'] for g in gt)
    prior.write_csv(directory / 'ground_truth_aligned.csv',
                    [dict(g, elapsed=g['t']-epoch) for g in gt])
    return result


if __name__ == '__main__':
    results = [analyze(p.parent) for p in sorted((HERE / 'runs').glob('*/manifest.json'))]
    (HERE / 'analysis.json').write_text(json.dumps(results, indent=2) + '\n')
    for row in results:
        print(json.dumps(row))
