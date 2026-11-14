#!/usr/bin/env python3
"""Score live fusion logs with the frozen R2 visibility and truth evaluator."""

import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from math import acos, degrees
import os
from pathlib import Path
import sys

import numpy as np

VELOCITIES = (
    ('direct', 'direct', 'direct_valid', 'direct_stamps'),
    ('ols_0.5', 'velocity', 'velocity_valid', 'sample_stamps'),
    ('ols_0.8', 'diagnostic', 'diagnostic_valid', 'diagnostic_stamps'),
)


def output_csv(path, rows):
    """Write per-frame evidence, including failed or unresolved observations."""
    if rows:
        fields = list(dict.fromkeys(key for row in rows for key in row))
        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fields)
            writer.writeheader()
            writer.writerows(rows)


def output_json(path, data):
    """Write a deterministic summary."""
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def scan_outputs(run, read, epoch, start, end):
    """Index finalized scans and keep timeout and reset publications separately."""
    rows = list(read(run / 'pedestrians.jsonl'))
    indexed = {}
    for row in rows:
        stamp = row['scan_stamp_ns']
        if (row['epoch'] != epoch or not start <= stamp <= end
                or row['status'] in ('scan_timeout', 'clock_reset')):
            continue
        if stamp in indexed:
            raise ValueError(f'{run}: duplicate fusion scan output at {stamp}')
        indexed[stamp] = row
    return indexed, rows


def score_actors(reference, scans, published, prior, second, start):
    """Apply R2 eligibility, IoU identities and source-window velocity truth."""
    frames = {}
    identity = {}
    for stamp, scan in sorted(scans.items()):
        output = published.get(stamp, {})
        image = output.get('image_stamp_ns')
        camera, owners = reference.camera_matches(image if image is not None else stamp)
        if image is None:
            owners = {}
        for state in output.get('states', []):
            actor = owners.get(state.get('sort_id', state['track_id']))
            if actor and state['last_observation_stamp'] == stamp:
                identity[stamp, state['track_id']] = actor
        lidar = prior.evaluate_geometry(
            'lidar', reference.robot.at(stamp), reference.poses(stamp),
            scan, None, reference.mount)
        frames[stamp] = (camera, lidar, owners, output)

    actors = []
    for stamp, (camera, lidar, owners, output) in frames.items():
        states = {}
        for state in output.get('states', []):
            track = state['track_id']
            if state.get('velocity_held'):
                actor = identity.get((state['last_observation_stamp'], track))
                if actor:
                    states.setdefault(actor, state)
            elif identity.get((stamp, track)):
                states[identity[stamp, track]] = state
        for actor in reference.people:
            state = states.get(actor, {})
            track = state.get('track_id')
            eligible = camera[actor]['visibility'] == lidar[actor]['visibility'] == 'full'
            actor_pose = reference.truth[actor].at(stamp)
            position_valid = bool(eligible and state.get('position_valid'))
            truth_xy = actor_pose[0][:2] if actor_pose is not None else None
            item = {
                'stamp_ns': stamp, 'scene_s': (stamp - start) / second,
                'image_stamp_ns': output.get('image_stamp_ns'), 'actor': actor,
                'track_id': track, 'sort_id': state.get('sort_id', track),
                'eligible': eligible, 'position_valid': position_valid,
                'velocity_held': bool(eligible and state.get('velocity_held')),
                'velocity_stamp_ns': state.get('velocity_stamp_ns'),
                'velocity_hold_age_ms': state['observation_age_ns'] / 1e6
                if eligible and state.get('velocity_held') else None,
                'camera_visibility': camera[actor]['visibility'],
                'lidar_visibility': lidar[actor]['visibility'],
                'status': output.get('status', 'missing_fusion_output'),
                'actor_motion': reference.truth[actor].motion(stamp),
                'robot_motion': reference.robot.motion(stamp),
                'truth_x': float(truth_xy[0]) if truth_xy is not None else None,
                'truth_y': float(truth_xy[1]) if truth_xy is not None else None,
            }
            if position_valid:
                xy = np.asarray(state['position'])
                item.update(x=float(xy[0]), y=float(xy[1]),
                            position_error_m=float(np.linalg.norm(xy - truth_xy)),
                            radius_m=state.get('occupancy_radius_m'))
            for method, key, flag, samples in VELOCITIES:
                item[method + '_reported'] = bool(eligible and state.get(flag))
                item[method + '_scored'] = False
                if not item[method + '_reported'] or state.get('velocity_held'):
                    continue
                estimate = {'sample_stamps_ns': state[samples], 'value': state[key]}
                score = reference.score_velocity(estimate, actor, track, identity, {})
                if score['valid']:
                    truth_speed = float(np.linalg.norm(score['truth']))
                    item.update({method + '_scored': True, method + '_error_mps': score['error'],
                                 method + '_speed_mps': score['speed'],
                                 method + '_stationary': score['stationary'],
                                 method + '_truth_speed_mps': truth_speed})
                    if not score['stationary'] and score['speed'] > 0:
                        cosine = np.dot(score['value'], score['truth']) / (
                            score['speed'] * truth_speed)
                        item[method + '_direction_error_deg'] = degrees(
                            acos(float(np.clip(cosine, -1., 1.))))
            actors.append(item)
    return actors, frames


def score_groups(reference, scans, frames):
    """Audit each associated or withheld A group using the R2 beam reference."""
    groups = []
    previous = {}
    transitions = Counter()
    for stamp, (_, _, owners, output) in frames.items():
        current = Counter(group['track_id'] for group in output.get('groups', [])
                          if group['track_id'] is not None)
        for track in current.keys() & previous.keys():
            if current[track] != previous[track]:
                transitions['split' if current[track] > previous[track] else 'merge'] += 1
        previous = current
        beam_info = {}
        scan = scans[stamp]
        for group in output.get('groups', []):
            for index in group['beams']:
                beam_info[index] = {
                    'beam': index,
                    'stamp_ns': stamp + round(index * scan['time_increment'] * 1e9),
                    'ids': [group['track_id']] if group['track_id'] else [],
                    'reason': 'overlap' if len(group['candidates']) > 1
                    and group['track_id'] is None else group['reason'],
                }
        evidence = {item['beam']: item for item in reference.beam_reference(
            {'stamp_ns': stamp, 'points': beam_info.values()}, owners)}
        for group in output.get('groups', []):
            labels = [evidence[index]['reference']['owners'] for index in group['beams']]
            confirmed = {label[0] for label in labels if len(label) == 1}
            assigned_actor = owners.get(group['track_id'])
            actual = next(iter(confirmed)) if len(confirmed) == 1 else None
            if group['track_id'] is None:
                outcome = 'unassigned'
            elif len(confirmed) > 1:
                outcome = 'mixed'
            elif not actual or not assigned_actor or any(len(label) != 1 for label in labels):
                outcome = 'unverified'
            else:
                outcome = 'correct' if actual == assigned_actor else 'wrong'
            groups.append({'stamp_ns': stamp, **group, 'assigned_actor': assigned_actor,
                           'truth_actor': actual, 'outcome': outcome,
                           'ray_owners': sorted(confirmed),
                           'ray_verified': actual is not None and all(
                               len(label) == 1 for label in labels),
                           'beam_labels': [evidence[index]['classification']
                                           for index in group['beams']]})
    return groups, dict(transitions)


def crossing_intervals(reference, stamps, start, second):
    """Bracket the MULTI pedestrian 1 and 2 image-center order reversal."""
    previous = None
    intervals = []
    for stamp in sorted(stamps):
        view, _ = reference.camera_matches(stamp)
        left, right = (view[actor].get('bbox') for actor in ('pedestrian_1', 'pedestrian_2'))
        if left is None or right is None:
            previous = None
            continue
        difference = (left[0] + left[2] - right[0] - right[2]) / 2
        if previous is not None and difference * previous[1] < 0:
            intervals.append([(previous[0] - start) / second, (stamp - start) / second])
        previous = stamp, difference
    return intervals


def frequencies(rows, stamps, second):
    """Measure finalized scan and ROS-clock publication rate."""
    source = sorted(stamps)
    publish = sorted(row['publish_stamp_ns'] for row in rows)
    return {
        'source_hz': (len(source) - 1) * second / (source[-1] - source[0])
        if len(source) > 1 and source[-1] > source[0] else None,
        'output_hz': (len(publish) - 1) * second / (publish[-1] - publish[0])
        if len(publish) > 1 and publish[-1] > publish[0] else None,
    }


def scene_metrics(actors, metrics):
    """Aggregate the unchanged eligible target-frame denominator."""
    eligible = [row for row in actors if row['eligible']]
    valid = [row for row in eligible if row['position_valid']]
    velocity = {}
    common = [row for row in eligible if all(row[m + '_scored'] for m, *_ in VELOCITIES)]
    for method, *_ in VELOCITIES:
        scored = [row for row in eligible if row[method + '_scored']]
        velocity[method] = {
            'reported': sum(row[method + '_reported'] for row in eligible),
            'scored': len(scored),
            'valid_rate': len(scored) / len(eligible) if eligible else None,
            'rmse_mps': metrics.rmse(row[method + '_error_mps'] for row in scored),
            'stationary_rms_mps': metrics.rmse(row[method + '_speed_mps'] for row in scored
                                               if row[method + '_stationary']),
            'direction_error_deg': metrics.distribution(
                row[method + '_direction_error_deg'] for row in scored
                if method + '_direction_error_deg' in row),
            'common_n': len(common),
            'common_rmse_mps': metrics.rmse(row[method + '_error_mps'] for row in common),
        }
    return {
        'eligible': len(eligible), 'positions': len(valid),
        'position_coverage': len(valid) / len(eligible) if eligible else None,
        'position_rmse_m': metrics.rmse(row['position_error_m'] for row in valid),
        'velocity': velocity,
        'held_ols': sum(row['velocity_held'] for row in eligible),
        'held_ols_age_ms': metrics.distribution(
            row['velocity_hold_age_ms'] for row in eligible if row['velocity_held']),
    }


def motion_history_audit(published, groups):
    """Audit accepted measurements and OLS windows using verified ray owners."""
    by_observation = defaultdict(list)
    for group in groups:
        if group['track_id'] is not None:
            by_observation[group['stamp_ns'], group['track_id']].append(group)
    samples = {}
    established = {}
    results = Counter(verified_measurements=0, unverified_measurements=0,
                      mixed_measurements=0, verified_history_links=0, wrong_history_links=0,
                      verified_fresh_ols=0, unverified_fresh_ols=0, held_ols_publications=0,
                      mixed_ols=0, foreign_sample_ols=0, duplicate_positions=0,
                      matched_after_miss=0)
    last_seen = {}
    previous = None
    for stamp, output in sorted(published.items()):
        current_owners = Counter()
        for state in output['states']:
            track = state['track_id']
            if state['last_observation_stamp'] == stamp:
                if track in last_seen and last_seen[track] != previous:
                    results['matched_after_miss'] += 1
                last_seen[track] = stamp
                support = by_observation[stamp, state.get('sort_id', track)]
                owners = {actor for group in support for actor in group['ray_owners']}
                if len(owners) == 1 and all(group['ray_verified'] for group in support):
                    actor, = owners
                    samples[stamp, track] = actor
                    results['verified_measurements'] += 1
                    if state['position_valid']:
                        current_owners[actor] += 1
                    if track in established:
                        results['verified_history_links'] += 1
                        results['wrong_history_links'] += int(established[track] != actor)
                    else:
                        established[track] = actor
                elif len(owners) > 1:
                    results['mixed_measurements'] += 1
                else:
                    results['unverified_measurements'] += 1
            if state.get('velocity_held'):
                results['held_ols_publications'] += 1
            elif state['velocity_valid'] and state['sample_stamps']:
                known = [samples.get((t, track)) for t in state['sample_stamps']]
                metric = 'verified_fresh_ols' if all(known) else 'unverified_fresh_ols'
                results[metric] += 1
                owners = {actor for actor in known if actor}
                results['mixed_ols'] += int(len(owners) > 1)
                results['foreign_sample_ols'] += int(bool(owners - {established.get(track)}))
        results['duplicate_positions'] += sum(count - 1 for count in current_owners.values())
        previous = stamp
    return dict(results), samples


def crossing_recovery(actors, intervals, second, metrics, ray_owners):
    """Time from truth image order reversal to observed speed recovery."""
    results = []
    for start, end in intervals:
        for actor in ('pedestrian_1', 'pedestrian_2'):
            actor_rows = [r for r in actors if r['actor'] == actor]
            rows = [r for r in actor_rows if r['scene_s'] >= end]
            before = next((r for r in reversed(actor_rows)
                           if r['scene_s'] < start and r['position_valid']), None)
            after = next((r for r in rows if r['position_valid']), None)
            first_direct = next((r for r in rows if r['direct_scored']), None)
            first_ols = next((r for r in rows if r['ols_0.5_scored']), None)
            consecutive = next((b for a, b in zip(rows, rows[1:])
                                if a['track_id'] == b['track_id'] and a['ols_0.5_scored']
                                and b['ols_0.5_scored']
                                and b['stamp_ns'] - a['stamp_ns'] <= .3 * second), None)
            verified = [r for r in actor_rows if r['position_valid']
                        and ray_owners.get((r['stamp_ns'], r['track_id'])) == actor]
            ray_before = next((r for r in reversed(verified) if r['scene_s'] < start), None)
            ray_after = next((r for r in verified if r['scene_s'] >= end), None)
            results.append({
                'actor': actor, 'crossing_end_s': end,
                'pre_cross_track': before['track_id'] if before else None,
                'post_cross_track': after['track_id'] if after else None,
                'observation_gap_s': after['scene_s'] - before['scene_s']
                if before and after else None,
                'ray_pre_cross_track': ray_before['track_id'] if ray_before else None,
                'ray_post_cross_track': ray_after['track_id'] if ray_after else None,
                'first_direct_s': first_direct['scene_s'] - end if first_direct else None,
                'first_ols_s': first_ols['scene_s'] - end if first_ols else None,
                'two_consecutive_ols_s': consecutive['scene_s'] - end if consecutive else None,
            })
    jumps = []
    previous = {}
    for row in (r for r in actors if r['position_valid']):
        key = row['actor']
        old = previous.get(key)
        if old and row['stamp_ns'] - old['stamp_ns'] <= .3 * second:
            jumps.append(float(np.hypot(
                (row['x'] - old['x']) - (row['truth_x'] - old['truth_x']),
                (row['y'] - old['y']) - (row['truth_y'] - old['truth_y']))))
        previous[key] = row
    return results, metrics.distribution(jumps)


def score_run(run, out, replay, projection, metrics, geometry, mesh):
    """Evaluate one recorded run and export all state and association evidence."""
    epoch, start, end = projection.complete_scene(run)
    reference = replay.Evaluation(run, epoch, geometry, mesh)
    published, all_rows = scan_outputs(run, replay.jsonl, epoch, start, end)
    scans = {stamp: scan for stamp, scan in reference.scans.items() if start <= stamp <= end}
    # Retain the independent scan recorder as denominator even if fusion dropped a scan.
    actors, frames = score_actors(reference, scans, published, replay.prior, metrics.SECOND, start)
    groups, transitions = score_groups(reference, scans, frames)
    sort_stamps = sorted(reference.sort)
    ideal = {stamp: projection.nearest_time(sort_stamps, stamp) for stamp in scans}
    pair_limit = round(metrics.SECOND / 30)
    timing = {
        'scans': len(scans), 'published': len(published),
        'recorder_missing_for_output': len(published.keys() - scans.keys()),
        'paired': sum(bool(row['image_stamp_ns']) for row in published.values()),
        'archived_nearest_pairs': sum(t is not None and abs(t - s) <= pair_limit
                                      for s, t in ideal.items()),
        'scan_status': dict(Counter(row['status'] for row in published.values())),
        'processing_ms': metrics.distribution(row['processing_ms'] for row in published.values()),
        'callback_to_publish_ms': metrics.distribution(
            row['callback_to_publish_ms'] for row in published.values()),
        'source_to_publish_ros_ms': metrics.distribution(
            (row['publish_stamp_ns'] - row['scan_stamp_ns']) / 1e6 for row in published.values()),
        'paired_abs_residual_ms': metrics.distribution(
            abs(row['image_stamp_ns'] - row['scan_stamp_ns']) / 1e6
            for row in published.values() if row['image_stamp_ns'] is not None),
        **frequencies(published.values(), scans, metrics.SECOND),
    }
    world_hash = hashlib.sha256((run / 'v01.world').read_bytes()).hexdigest()
    scene = run.name.split('-')[0]
    reference_worlds = Path(projection.__file__).parent.glob(
        f'captures/{scene}-*/v01.world')
    summary = {
        'run': run.name, 'scene': scene, 'epoch': epoch,
        'world_sha256': world_hash,
        'world_matches_r2': any(hashlib.sha256(path.read_bytes()).hexdigest() == world_hash
                                for path in reference_worlds),
        **scene_metrics(actors, metrics),
        'continuity': replay.continuity(actors), 'timing': timing,
        'track_group_transitions': transitions,
        'crossing_intervals_s': crossing_intervals(reference, scans, start, metrics.SECOND)
        if scene == 'MULTI' else [],
        'group_outcomes': {
            reason: dict(Counter(row['outcome'] for row in groups if row['reason'] == reason))
            for reason in sorted({row['reason'] for row in groups})
        },
    }
    history, ray_owners = motion_history_audit(published, groups)
    recovery, jumps = crossing_recovery(
        actors, summary['crossing_intervals_s'], metrics.SECOND, metrics, ray_owners)
    summary.update(motion_history=history,
                   crossing_recovery=recovery, position_jump_residual_m=jumps)
    path = out / run.name
    path.mkdir(parents=True, exist_ok=True)
    output_json(path / 'summary.json', summary)
    output_csv(path / 'actor_frames.csv', actors)
    output_csv(path / 'groups.csv', groups)
    output_csv(path / 'track_states.csv', [
        {'status': row['status'], 'epoch': row['epoch'], 'scan_stamp_ns': row['scan_stamp_ns'],
         'image_stamp_ns': row['image_stamp_ns'], 'publish_stamp_ns': row['publish_stamp_ns'],
         **state}
        for row in all_rows for state in row['states']
    ])
    return summary, actors


def plot_scenes(out, scored):
    """Render a representative online trajectory and speed timeline per scene."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    scenes = ('STATIC', 'EGO', 'PERSON', 'MULTI')
    figure, axes = plt.subplots(len(scenes), 2, figsize=(12, 12), layout='constrained')
    for scene, (xy, velocity) in zip(scenes, axes):
        options = [(summary, rows) for summary, rows in scored if summary['scene'] == scene]
        if not options:
            xy.set_axis_off()
            velocity.set_axis_off()
            continue
        summary, rows = options[-1]
        by_actor = defaultdict(list)
        for row in rows:
            by_actor[row['actor']].append(row)
        for number, (actor, actor_rows) in enumerate(sorted(by_actor.items())):
            color = f'C{number}'
            label = actor.replace('human_female_1', 'person')
            xy.plot([row['truth_x'] for row in actor_rows], [row['truth_y'] for row in actor_rows],
                    '--', color=color, marker='+', markevery=[0], label=label + ' truth')
            valid = [row for row in actor_rows if row['position_valid']]
            xy.scatter([row['x'] for row in valid], [row['y'] for row in valid],
                       s=12, color=color, label=label + ' measured')
            if valid:
                last = valid[-1]
                if last['radius_m']:
                    xy.add_patch(plt.Circle((last['x'], last['y']), last['radius_m'],
                                            color=color, alpha=.13))
            times = [row['scene_s'] for row in actor_rows]
            velocity.plot(times, [row.get('ols_0.5_speed_mps', np.nan) for row in actor_rows],
                          '.-', color=color, markersize=4, label=label + ' OLS 0.5 s')
            velocity.scatter(times, [row.get('direct_speed_mps', np.nan) for row in actor_rows],
                             s=10, marker='x', color=color, alpha=.5)
            truth_speeds = [row.get('ols_0.5_truth_speed_mps', np.nan) for row in actor_rows]
            velocity.plot(times, truth_speeds, '--', color=color, alpha=.65)
        xy.set(title=summary['run'], xlabel='odom x [m]', ylabel='odom y [m]', aspect='equal')
        velocity.set(title='OLS speed, matched truth (dashed), direct difference (×)',
                     xlabel='Scene time [s]', ylabel='Speed [m/s]',
                     xlim=(min(row['scene_s'] for row in rows) - .1,
                           max(row['scene_s'] for row in rows) + .1))
        xy.legend(loc='upper left', bbox_to_anchor=(1.02, 1), fontsize=7)
        velocity.legend(loc='best', fontsize=7)
        for ax in (xy, velocity):
            ax.grid(alpha=.23)
    figure.savefig(out / 'scene_tracks.png', dpi=150)
    plt.close(figure)


def plot_crossings(out, scored):
    """Show position outages and decisions for overlapping groups in MULTI."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    multi = [(summary, actors) for summary, actors in scored if summary['scene'] == 'MULTI']
    if not multi:
        return
    figure, axes = plt.subplots(len(multi), 2, squeeze=False,
                                figsize=(12, 3.2 * len(multi)), layout='constrained')
    for (summary, actors), (coverage, association) in zip(multi, axes):
        actor_ids = sorted({row['actor'] for row in actors})
        time_by_stamp = {row['stamp_ns']: row['scene_s'] for row in actors}
        for index, actor in enumerate(actor_ids):
            eligible = [r for r in actors if r['actor'] == actor and r['eligible']]
            coverage.scatter([r['scene_s'] for r in eligible], [index] * len(eligible),
                             color='.7', marker='x', s=10)
            observed = [r for r in eligible if r['position_valid']]
            coverage.scatter([r['scene_s'] for r in observed], [index] * len(observed),
                             color=f'C{index}', s=12)
        coverage.set_yticks(range(len(actor_ids)), actor_ids)
        coverage.set_title(summary['run'] + ' position valid (●) or missing (×)')
        counts = defaultdict(Counter)
        with (out / summary['run'] / 'groups.csv').open(encoding='utf-8') as stream:
            for row in csv.DictReader(stream):
                counts[row['reason']][time_by_stamp[int(row['stamp_ns'])]] += 1
        for index, reason in enumerate(('overlap_ipm', 'overlap_unresolved', 'ipm_incomplete')):
            events = counts[reason]
            sizes = [12 + 10 * count for count in events.values()]
            association.scatter(list(events), [index] * len(events), s=sizes, color=f'C{index}')
            association.annotate(str(sum(events.values())), xy=(.99, index),
                                 xycoords=('axes fraction', 'data'), ha='right')
        association.set_yticks(range(3), ['Assigned by IPM', 'Ambiguous', 'Incomplete IPM'])
        association.set_title('Overlap group decisions (marker size = group count)')
        for ax in (coverage, association):
            for left, right in summary['crossing_intervals_s']:
                ax.axvspan(left, right, color='orange', alpha=.22)
            limits = (min(time_by_stamp.values()) - .1, max(time_by_stamp.values()) + .1)
            ax.set(xlabel='Scene time [s]', xlim=limits)
            ax.grid(axis='x', alpha=.23)
    figure.suptitle('Orange bands mark the truth image-center order reversal')
    figure.savefig(out / 'multi_crossing.png', dpi=150)
    plt.close(figure)


def main():
    """Evaluate scene directories using the existing experiment environment."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--r2', type=Path, default=os.environ.get('R2_DEMO'),
                        help='R2 evaluator directory (or set R2_DEMO)')
    args = parser.parse_args()
    if args.r2 is None or not (args.r2 / 'replay.py').is_file():
        parser.error('set --r2 or R2_DEMO to a directory containing replay.py')
    sys.path.insert(0, str(args.r2))
    import occupancy
    import projection
    import r2_metrics
    import replay

    args.output.mkdir(parents=True, exist_ok=True)
    with np.load(args.r2 / 'collision_triangles.npz') as data:
        mesh = r2_metrics.RayMesh(data['triangles'])
    geometry = occupancy.load_geometry(args.r2 / 'collision_geometry.json')
    results = [score_run(run, args.output, replay, projection, r2_metrics, geometry, mesh)
               for run in args.runs]
    output_json(args.output / 'runs.json', [row for row, _ in results])
    scenarios = {}
    for scenario in sorted({summary['scene'] for summary, _ in results}):
        rows = [
            row for summary, actors in results if summary['scene'] == scenario for row in actors
        ]
        scenarios[scenario] = scene_metrics(rows, r2_metrics)
    output_json(args.output / 'scenarios.json', scenarios)
    plot_scenes(args.output, results)
    plot_crossings(args.output, results)
    print(args.output / 'runs.json')


if __name__ == '__main__':
    main()
