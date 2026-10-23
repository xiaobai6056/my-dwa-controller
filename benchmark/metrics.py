#!/usr/bin/env python3
"""Recomputable navigation metrics from saved CSV/JSON; no ROS dependency."""
import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path


def angle_error(actual, target):
    return abs(math.atan2(math.sin(actual - target), math.cos(actual - target)))


def polyline_distance(point, path):
    if not path:
        return None
    best = math.inf
    for a, b in zip(path, path[1:] or path):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length2 = dx*dx + dy*dy
        fraction = (max(0., min(1., ((point[0]-a[0])*dx + (point[1]-a[1])*dy) / length2))
                    if length2 else 0.)
        distance = math.hypot(point[0]-a[0]-fraction*dx, point[1]-a[1]-fraction*dy)
        best = min(best, distance)
    return best


def intervals(samples, max_gap=.5):
    previous = None
    for sample in samples:
        if sample is None:
            previous = None
            continue
        if previous is not None:
            dt = sample[0] - previous[0]
            if 0. < dt <= max_gap:
                yield previous, sample, dt
            if dt <= 0.:
                continue
        previous = sample


def pose_metrics(samples, reference, max_gap=.5):
    length, integral, duration = 0., 0., 0.
    errors = [polyline_distance(s[1:3], reference) for s in samples if s is not None]
    errors = [e for e in errors if e is not None]
    for a, b, dt in intervals(samples, max_gap):
        length += math.hypot(b[1]-a[1], b[2]-a[2])
        duration += dt
        if reference:
            ea, eb = polyline_distance(a[1:3], reference), polyline_distance(b[1:3], reference)
            integral += .5 * (ea*ea + eb*eb) * dt
    return {'trajectory_length_m': length if duration else None, 'integrated_time_s': duration,
            'lateral_rmse_m': math.sqrt(integral / duration) if reference and duration else None,
            'lateral_max_m': max(errors) if errors else None,
            'reference_missing_reason': None if reference else 'no map-frame reference path'}


def velocity_metrics(samples, max_gap=.5):
    duration = integral = square = variation = acceleration_square = 0.
    maximum = 0.
    edges = list(intervals(samples, max_gap))
    for a, b, dt in edges:
        delta = b[1] - a[1]
        duration += dt
        integral += .5 * (a[1] + b[1]) * dt
        square += .5 * (a[1]**2 + b[1]**2) * dt
        variation += abs(delta)
        acceleration_square += delta*delta / dt
        maximum = max(maximum, abs(delta) / dt)
    mean = integral / duration if duration else None
    values = [s[1] for s in samples if s is not None]
    return {'mean': mean, 'std': math.sqrt(max(0., square / duration - mean**2))
            if duration else None, 'max_abs': max(map(abs, values)) if values else None,
            'total_variation': variation if duration else None, 'integrated_time_s': duration,
            'mean_abs_acceleration': variation / duration if duration else None,
            'rms_acceleration': math.sqrt(acceleration_square / duration) if duration else None,
            'max_abs_acceleration': maximum if duration else None,
            'discarded_intervals': max(0, len(samples) - 1 - len(edges))}


def direction_switches(samples, deadband=.05, min_duration=.15, max_gap=.5):
    if not any(intervals(samples, max_gap)):
        return None
    stable = candidate = 0
    began = previous_time = None
    switches = 0
    for sample in samples:
        if sample is None:
            stable = candidate = 0
            previous_time = None
            continue
        stamp, value = sample
        if previous_time is not None:
            if stamp <= previous_time:
                continue
            if stamp - previous_time > max_gap:
                stable = candidate = 0
        previous_time = stamp
        sign = (1 if value > 0. else -1) if abs(value) >= deadband else 0
        if sign != candidate:
            candidate, began = sign, stamp
        if candidate and stamp - began >= min_duration - 1e-9 and candidate != stable:
            switches += int(stable != 0)
            stable = candidate
    return switches


def number(row, name):
    try:
        result = float(row[name])
        return result if math.isfinite(result) else None
    except (KeyError, ValueError, TypeError):
        return None


def samples_from_rows(rows, stamp_column, columns, minimum_stamp=None):
    result = []
    for row in rows:
        values = [number(row, name) for name in [stamp_column] + columns]
        valid = all(v is not None for v in values)
        valid = valid and (minimum_stamp is None or values[0] >= minimum_stamp)
        result.append(tuple(values) if valid else None)
    return result


def timing_metrics(summary, events, feedback):
    start = number(summary, 'navigation_start_sim_s')
    end = number(summary, 'navigation_end_sim_s')
    goals = {}
    for event in events:
        stamp = number(event, 'goal_stamp_s')
        if (event.get('action') == 'compute_path_to_pose' and
                event.get('status') in (1, 2, 3, 4, 5, 6) and stamp is not None and stamp > 0.
                and (start is None or stamp >= start) and (end is None or stamp <= end)):
            goals.setdefault(event.get('goal_id', stamp), stamp)
    anchor = min(goals.values()) if goals else None
    span = end - anchor if end is not None and anchor is not None and end >= anchor else None
    feedback_durations = [number(f, 'navigation_time_s') for f in feedback]
    feedback_durations = [value for value in feedback_durations if value is not None]
    last_feedback = feedback_durations[-1] if feedback_durations else None
    return {'first_planner_goal_stamp_s': anchor,
            'first_planner_to_observed_stop_sim_s': span,
            'legacy_phase_clock_span_s': summary.get('navigation_sim_s'),
            'nav2_last_feedback_duration_s': last_feedback,
            'anchor_missing_reason': None if anchor is not None else
            'no accepted planner goal with a native stamp inside the recorded trial',
            'definition': 'First ComputePathToPose goal native stamp to last collector /clock '
            'at result detection or watchdog stop; excludes dispatch latency. Not the exact '
            'NavigateToPose action duration. Clock receipt quantization is about 0.1 s.'}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def calculate(directory, max_gap=.5, deadband=.05, min_duration=.15):
    directory = Path(directory)
    summary = json.loads((directory / 'summary.json').read_text())
    with (directory / 'trajectory.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    plans = []
    plan_file = directory / 'plans.jsonl'
    if plan_file.exists():
        plans = [json.loads(line) for line in plan_file.read_text().splitlines() if line]
    reference_file = directory / 'reference_path.json'
    reference_record = json.loads(reference_file.read_text()) if reference_file.exists() else next(
        (p for p in plans if p.get('frame_id') == 'map' and len(p.get('points', [])) > 1), None)
    reference = (reference_record['points'] if reference_record and
                 reference_record.get('frame_id') == 'map' else None)
    events_file = directory / 'nav_events.jsonl'
    events = [json.loads(line) for line in events_file.read_text().splitlines()
              if line] if events_file.exists() else []
    feedback_file = directory / 'feedback.jsonl'
    feedback = [json.loads(line) for line in feedback_file.read_text().splitlines()
                if line] if feedback_file.exists() else []
    timing = timing_metrics(summary, events, feedback)
    anchor = timing['first_planner_goal_stamp_s']
    pose_samples = samples_from_rows(rows, 'tf_stamp_s', ['x', 'y'], anchor)
    motion = pose_metrics(pose_samples, reference, max_gap)
    measured = {}
    for name, stamp, linear, angular in (
            ('actual', 'odom_stamp_s', 'actual_vx', 'actual_wz'),
            ('controller_command', 'controller_cmd_receipt_sim_s',
             'controller_vx', 'controller_wz'),
            ('robot_command', 'robot_cmd_receipt_sim_s', 'robot_cmd_vx', 'robot_cmd_wz')):
        vx = samples_from_rows(rows, stamp, [linear], anchor)
        wz = samples_from_rows(rows, stamp, [angular], anchor)
        measured[name] = {'linear': velocity_metrics(vx, max_gap),
                          'angular': velocity_metrics(wz, max_gap),
                          'angular_direction_switches': direction_switches(
                              wz, deadband, min_duration, max_gap)}
    executing = [e for e in events if e.get('status') == 2]
    aborted = [e for e in events if e.get('status') == 6]
    behavior_goals = {(e['action'], e['goal_id']) for e in executing
                      if e['action'] in ('spin', 'backup', 'drive_on_heading', 'wait')}
    observed_behaviors = {(e['action'], e['goal_id']) for e in events
                          if e.get('status', 0) > 0 and
                          e['action'] in ('spin', 'backup', 'drive_on_heading', 'wait')}
    spins = sum(action == 'spin' for action, _ in behavior_goals)
    bt_file = directory / 'bt_events.jsonl'
    bt_events = [json.loads(line) for line in bt_file.read_text().splitlines()
                 if line] if bt_file.exists() else []
    bt_failures = Counter(e['node'] for e in bt_events if e.get('current') == 'FAILURE')
    terminal = None
    goal = summary.get('arguments', {}).get('goal')
    if goal:
        nearby = [r for r in rows if number(r, 'x') is not None and
                  math.hypot(float(r['x'])-goal[0], float(r['y'])-goal[1]) <= .30 and
                  number(r, 'actual_vx') is not None and abs(float(r['actual_vx'])) <= .03 and
                  number(r, 'actual_wz') is not None and abs(float(r['actual_wz'])) >= .05]
        terminal = {'near_goal_rotation_samples': len(nearby), 'spin_actions': spins,
                    'attribution': ('controller' if nearby and not observed_behaviors and
                                    summary.get('event_monitor_ready') else 'unconfirmed')}
    duration = summary.get('navigation_sim_s')
    output = {'schema_version': 2, 'metric_revision': 'source_window_v2',
              'trial_id': summary['trial_id'],
              'status': summary['status'], 'valid': summary.get('valid', False),
              'success': (summary.get('action_result') == 'SUCCEEDED' and
                          summary.get('valid', False)),
              'failure_reason': None if summary.get('action_result') == 'SUCCEEDED' else {
                  k: summary.get(k) for k in
                  ('stop_reason', 'error', 'action_result', 'action_error')},
              'navigation_sim_s': duration if duration is not None and duration >= 0. else None,
              'timing': timing,
              'motion': motion, 'velocity': measured, 'final_error': summary.get('final_error'),
              'reference': {'definition': 'saved fixed reference' if reference_file.exists()
                            else 'first valid map-frame global plan',
                            'point_count': len(reference) if reference else 0,
                            'plan_messages': len(plans),
                            'distinct_plans': len({p.get('geometry_sha256') for p in plans}),
                            'replanning_publications': max(0, len(plans)-1)},
              'navigation_system': {'behavior_goals_executed': len(behavior_goals),
                                    'behavior_goals_observed': len(observed_behaviors),
                                    'spin_goals_executed': spins,
                                    'observed_aborted_actions': aborted,
                                    'abort_semantics': 'Old FollowPath goals may be ABORTED '
                                    'during normal replanning preemption. Cross-check BT '
                                    'FAILURE events and recovery feedback before attribution.',
                                    'bt_node_failure_counts': dict(bt_failures),
                                    'max_feedback_recoveries': summary.get(
                                        'max_feedback_recoveries'),
                                    'event_monitor_ready': summary.get(
                                        'event_monitor_ready', False)},
              'terminal_rotation': terminal,
              'localization': {'end_map_world_disagreement_m':
                               summary.get('end_map_world_disagreement_m'),
                               'warning_over_0_5m': summary.get(
                                   'end_map_world_disagreement_m', 0.) > .5,
                               'note': 'World/map alignment follows the scene definition; '
                               'endpoint queries are asynchronous. No clearance inference.'},
              'safety': {'minimum_clearance_m': None, 'collision_count': None,
                         'missing_reason': 'No validated full robot collision footprint/contact '
                         'sensor; scan ranges and costmap radius are not ground-truth clearance.'},
              'parameters': {'max_gap_s': max_gap, 'wz_deadband_rad_s': deadband,
                             'minimum_sign_duration_s': min_duration,
                             'minimum_source_stamp_s': anchor,
                             'window': 'Exclude source samples before first planner acceptance '
                             'when available; retain samples collected through stop detection '
                             'even if their source stamps lead the last /clock. No anchor: '
                             'legacy recording window, explicitly flagged in timing.',
                             'integration': 'trapezoidal, source sim time; gaps not bridged'},
              'raw_rows': len(rows), 'input_sha256': {
                  p.name: digest(p) for p in
                  (directory / 'summary.json', directory / 'trajectory.csv',
                   plan_file, reference_file, events_file, bt_file, feedback_file) if p.exists()},
              'metrics_source_sha256': digest(Path(__file__))}
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial_directory', type=Path)
    parser.add_argument('--max-gap', type=float, default=.5)
    parser.add_argument('--wz-deadband', type=float, default=.05)
    parser.add_argument('--minimum-sign-duration', type=float, default=.15)
    args = parser.parse_args()
    output = calculate(args.trial_directory, args.max_gap, args.wz_deadband,
                       args.minimum_sign_duration)
    path = args.trial_directory / 'metrics.json'
    path.write_text(json.dumps(output, indent=2, allow_nan=False) + '\n')
    print(path)


if __name__ == '__main__':
    main()
