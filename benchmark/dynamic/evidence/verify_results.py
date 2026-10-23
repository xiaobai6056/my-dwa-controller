#!/usr/bin/env python3
"""Audit all trial results against raw geometry, sensor grids and action evidence."""
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import struct

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from evaluation import calculate, read_jsonl  # noqa: E402
from scenario import load, position  # noqa: E402


def audit(directory):
    manifest = json.loads((directory / 'manifest.json').read_text())
    published = {r['run']: r for r in json.loads(
        (directory / 'results.json').read_text())}
    checks = []
    for trial in manifest['trials']:
        path = directory / trial['directory']
        summary = json.loads((path / 'summary.json').read_text())
        result = calculate(path)
        saved = published[trial['directory']]
        metric_equal = all(saved[k] == v for k, v in result.items())
        gt = read_jsonl(path / 'ground_truth.jsonl')
        epoch = summary['navigation_start_sim_s']
        scene = load(trial['scene'])
        # Unstamped ModelStates can lead latest observed clock by one tick.
        errors = [math.dist(r['obstacle'][:2], position(
            scene, r['t'] - epoch)) for r in gt]
        movement = max(r['obstacle'][1] for r in gt) - \
            min(r['obstacle'][1] for r in gt)
        events = read_jsonl(path / 'nav_events.jsonl')
        behavior = {name: len({e['goal_id'] for e in events
                              if e['action'] == name and e['status'] in (1, 2, 4)})
                    for name in ('spin', 'backup', 'wait', 'drive_on_heading')}
        perception = read_jsonl(path / 'perception.jsonl')
        sensor_agrees = True
        mismatches = []
        sensor_counts = {'scan': 0, 'costmap': 0}
        with gzip.open(path / 'sensors.jsonl.gz', 'rt') as stream:
            for line in stream:
                row = json.loads(line)
                record = perception[sensor_counts['scan'] +
                                    sensor_counts['costmap']]
                kind = row['type']
                sensor_counts[kind] += 1
                if kind == 'scan':
                    x, y, angle = row['laser_world']
                    ox, oy, oa = row['obstacle_world']
                    w, h, _ = scene['size']
                    minimum = row.get(
                        'range_min', struct.unpack(
                            'f', struct.pack(
                                'f', .12))[0])
                    maximum = row.get('range_max', 20.)
                    hits = 0
                    for i, radius in enumerate(row['ranges']):
                        if radius is None or not minimum <= radius <= maximum:
                            continue
                        a = angle + row['angle_min'] + \
                            i * row['angle_increment']
                        px, py = x + radius * \
                            math.cos(a) - ox, y + radius * math.sin(a) - oy
                        u = math.cos(oa) * px + math.sin(oa) * py
                        v = -math.sin(oa) * px + math.cos(oa) * py
                        hits += abs(u) <= w / 2 + .08 and abs(v) <= h / 2 + .08
                    sensor_agrees &= hits == record['hit_rays']
                    if hits != record['hit_rays']:
                        mismatches.append(
                            dict(
                                type=kind, t=row['t'],
                                raw=hits,
                                recorded=record['hit_rays']))
                else:
                    x, y, angle = row['obstacle_grid_pose']
                    w, h, _ = scene['size']
                    lethal = 0
                    for j in range(row['height']):
                        for i in range(row['width']):
                            px = row['origin'][0] + \
                                (i + .5) * row['resolution'] - x
                            py = row['origin'][1] + \
                                (j + .5) * row['resolution'] - y
                            u = math.cos(angle) * px + math.sin(angle) * py
                            v = -math.sin(angle) * px + math.cos(angle) * py
                            if abs(u) <= w / 2 and abs(v) <= h / 2:
                                lethal += row['data'][j *
                                                      row['width'] + i] >= 99
                    sensor_agrees &= lethal == record['lethal_cells']
                    if lethal != record['lethal_cells']:
                        mismatches.append(
                            dict(
                                type=kind, t=row['t'],
                                raw=lethal,
                                recorded=record
                                ['lethal_cells']))
        source_hashes_valid = all(
            hashlib.sha256((path / c['saved']).read_bytes()).hexdigest() ==
            c['sha256'] for c in summary['config_files'])
        # Controller failure causes remain observations from the original
        # launch log.
        log = (directory /
               'logs' /
               (trial['directory'].replace('/', '_') +
                '_launch.log')).read_text()
        causes = {
            phrase: log.count(phrase) for phrase in (
                'Failed to make progress',
                'No valid trajectories',
                'Failed to get a path',
                'Control loop missed',
                'Behavior will be aborted')}
        contacts = read_jsonl(path / 'contacts.jsonl')
        checks.append(
            dict(
                run=trial['directory'],
                metric_equal=metric_equal,
                plugin=summary['actual_controller_plugin'],
                accepted=summary['action_accepted'],
                action_error=summary.get('action_error'),
                valid=summary['valid'],
                source_hashes_valid=source_hashes_valid,
                state_subscribers=summary['topic_endpoints']
                ['/benchmark/model_states']['subscribers'],
                motion_y_span_m=movement,
                max_motion_clock_association_error_m=max(errors),
                motion_service=summary['motion_service'],
                physical_start_error=summary
                ['physical_start_error'],
                physical_contact_first=contacts[0]
                if contacts else None,
                physical_contact_pairs=sorted(
                    {tuple(c['pair']) for c in contacts}),
                perception_raw_recomputed=sensor_agrees,
                sensor_counts=sensor_counts,
                sensor_mismatches=mismatches,
                behavior_goals=behavior, log_phrase_counts=causes,
                end_robot_world=gt[-1]['robot']))
    before = json.loads((directory / 'protected_before.json').read_text())
    after = json.loads((directory / 'protected_after.json').read_text())
    aggregate = json.loads((directory / 'aggregate.json').read_text())
    audit_result = dict(
        trials=len(checks),
        accepted=sum(
            r['accepted'] for r in checks),
        all_metrics_match=all(
            r['metric_equal'] for r in checks),
        all_perception_raw_matches=all(
            r['perception_raw_recomputed'] for r in checks),
        all_source_hashes_valid=all(
            r['source_hashes_valid'] for r in checks),
        all_valid=all(
            r['valid'] for r in checks),
        no_action_client_errors=all(
            r['action_error'] is None for r in checks),
        all_cleanup_confirmed=all(
            t['cleanup_returncode'] == 0 for t in manifest['trials']),
        protected_file_count=len(before),
        protected_before_after_equal=before == after,
        fairness=aggregate['fairness'],
        checks=checks)
    (directory / 'validation.json').write_text(json.dumps(audit_result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in audit_result.items()
          if k not in ('checks', 'fairness')}, indent=2))


if __name__ == '__main__':
    audit(Path(sys.argv[1]).resolve())
