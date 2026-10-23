"""Analyze all saved scan trials without replaying or publishing any ROS data."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import re
import sqlite3
import sys

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
spec = importlib.util.spec_from_file_location(
    'alignment', ROOT / 'benchmark/baseline_diagnosis_20261021/bag_export.py')
align = importlib.util.module_from_spec(spec)
spec.loader.exec_module(align)


def rows(path):
    """Read an original collector's JSONL stream."""
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def log_records(path, marker):
    """Keep line numbers for direct audit of every diagnostic record."""
    result = []
    for number, line in enumerate(path.read_text().splitlines(), 1):
        if marker not in line:
            continue
        row = {'line': number}
        for key, value in re.findall(r'(\w+)=([^\s]+)', line.split(marker, 1)[1]):
            try:
                row[key] = float(value)
            except ValueError:
                row[key] = value
        result.append(row)
    return result


def export_bag(directory):
    """Read exact scan/TF source timestamps; no latest-transform fallback."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    bag = directory / 'bag'
    info = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    edges, static, scans, odom = {}, set(), [], []
    for db in info['relative_file_paths']:
        con = sqlite3.connect(f'file:{bag / db}?mode=ro', uri=True)
        topics = {i: (name, get_message(typ)) for i, name, typ in
                  con.execute('select id,name,type from topics')}
        for topic_id, receipt, data in con.execute(
                'select topic_id,timestamp,data from messages order by timestamp,id'):
            name, cls = topics[topic_id]
            if name not in ('/tf', '/tf_static', '/scan', '/odom'):
                continue
            msg = deserialize_message(data, cls)
            if name in ('/tf', '/tf_static'):
                for tr in msg.transforms:
                    edge = (tr.header.frame_id, tr.child_frame_id)
                    v = tr.transform
                    pose = [v.translation.x, v.translation.y, align.yaw(v.rotation)]
                    edges.setdefault(edge, {})[align.seconds(tr.header.stamp)] = pose
                    if name == '/tf_static':
                        static.add(edge)
            elif name == '/scan':
                scans.append(dict(stamp=align.seconds(msg.header.stamp), receipt=receipt * 1e-9,
                                  frame=msg.header.frame_id, scan_time=msg.scan_time,
                                  time_increment=msg.time_increment, rays=len(msg.ranges),
                                  valid=sum(
                                      math.isfinite(v) and msg.range_min <= v <= msg.range_max
                                      for v in msg.ranges)))
            else:
                odom.append(dict(stamp=align.seconds(msg.header.stamp),
                                 v=msg.twist.twist.linear.x, w=msg.twist.twist.angular.z))
        con.close()
    return {e: sorted(v.items()) for e, v in edges.items()}, static, scans, odom


def write_csv(path, records):
    """Save fully inspectable samples, including poorly estimated intervals."""
    if not records:
        return
    keys = list(dict.fromkeys(k for r in records for k in r))
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, keys)
        writer.writeheader()
        writer.writerows(records)


def stats(records):
    """Report velocity magnitude AND transverse error; a correct sign is insufficient."""
    if not records:
        return {'n': 0}
    vx, vy = np.array([[r['world_vx'], r['world_vy']] for r in records]).T
    error = np.hypot(vx, vy - .25)
    return dict(n=len(records), stamp_interval=[records[0]['stamp'], records[-1]['stamp']],
                median_vx=float(np.median(vx)), median_vy=float(np.median(vy)),
                vy_range=[float(min(vy)), float(max(vy))],
                velocity_rmse=float(np.sqrt(np.mean(error ** 2))),
                velocity_error_p90=float(np.quantile(error, .9)),
                correct_direction_fraction=float(np.mean(vy > 0)),
                median_confidence=float(np.median([r['confidence'] for r in records])),
                median_position_error=float(np.median([r['position_error'] for r in records])),
                max_stamp_gap=float(max(np.diff([r['stamp'] for r in records]), default=0)))


def analyze(directory):
    """Compare online tracker output with held-out scenario motion and observed GT."""
    manifest = json.loads((directory / 'manifest.json').read_text())
    summary = json.loads((directory / 'trial/summary.json').read_text())
    result = dict(run=directory.name, plugin_sha256=manifest['plugin_sha256'],
                  mapping=manifest.get('runtime_mapping_verified'),
                  outcome=manifest.get('outcome'), cleanup=manifest.get('cleanup_returncode'))
    if not summary.get('valid'):
        return result
    epoch = summary['navigation_start_sim_s']
    tracks = log_records(directory / 'launch.log', '[SCAN_TRACK]')
    risk = log_records(directory / 'launch.log', '[PATH_CONFLICT]')
    status = log_records(directory / 'launch.log', '[SCAN_STATUS]')
    write_csv(directory / 'tracks_all.csv', tracks)
    write_csv(directory / 'risk_all.csv', risk)
    edges, static, scans, odom = export_bag(directory)
    write_csv(directory / 'scan_metadata.csv', scans)
    result['epoch'] = epoch
    result['scan_metadata'] = dict(
        frames=sorted({s['frame'] for s in scans}), count=len(scans),
        median_period=float(np.median(np.diff([s['stamp'] for s in scans]))),
        time_increments=sorted({s['time_increment'] for s in scans}),
        scan_times=sorted({s['scan_time'] for s in scans}))
    transitions = []
    for r in risk:
        if not transitions or r['state'] != transitions[-1]['state']:
            transitions.append(dict(t=r['t'] - epoch, state=r['state'], line=r['line'],
                                    cap=r['cap'], predicted=r['predicted'],
                                    inner=r['inner'], core=r['core'], region=r['region']))
    result['transitions'] = transitions
    result['risk_counts'] = {s: sum(r['state'] == s for r in risk)
                             for s in sorted({r['state'] for r in risk})}
    result['max_tf_failures'] = max((s['tf_failures'] for s in status), default=0)
    result['unresolved_statuses'] = sum(s['unresolved'] for s in status)
    result['predicted_conflicts_without_costmap_inner'] = sum(
        r['predicted'] and str(r['core']).startswith('0/') for r in risk)
    result['first_non_normal'] = next((r for r in transitions if r['state'] != 'normal'), None)
    if not directory.name.endswith(('D01_t2', 'D01_t4')):
        confident = [t for t in tracks if t['observed'] and t['confidence'] >= .65]
        result['static_confident_tracks'] = len(confident)
        moving = [t for t in confident if math.hypot(t['vx'], t['vy']) >= .08]
        result['static_moving_samples'] = len(moving)
        result['static_moving_ids'] = sorted({t['id'] for t in moving})
        result['static_scan_conflict_samples'] = sum(t['conflict'] for t in tracks)
        result['static_max_confident_speed'] = max(
            (math.hypot(t['vx'], t['vy']) for t in confident), default=0)
        return result
    gt = rows(directory / 'trial/ground_truth.jsonl')
    scene = json.loads((directory / 'effective_scene.json').read_text())
    start = scene['keyframes'][1][0]
    result['motion_start'] = start
    result['first_observed_gt_motion'] = next(
        (g['t'] - epoch for g in gt if g['obstacle'][1] > -1.095), None)
    result['first_contact'] = next(
        (g['stamp'] - epoch for g in rows(directory / 'trial/contacts.jsonl')), None)
    candidates = []
    for t in tracks:
        if not t['observed']:
            continue
        g = min(gt, key=lambda g: abs(g['t'] - t['now']))
        transform = align.lookup_pose(edges, static, 'odom', 'base_footprint', t['now'])
        if transform is None or abs(g['t'] - t['now']) > .2:
            continue
        world_odom = align.compose(g['robot'], align.inverse(transform))
        world = align.compose(world_odom, [t['x'], t['y'], 0])
        c, s = math.cos(world_odom[2]), math.sin(world_odom[2])
        row = dict(t, elapsed=t['stamp'] - epoch, world_x=world[0], world_y=world[1],
                   world_vx=c * t['vx'] - s * t['vy'], world_vy=s * t['vx'] + c * t['vy'],
                   position_error=math.hypot(
                       world[0] - g['obstacle'][0], world[1] - g['obstacle'][1]),
                   gt_x=g['obstacle'][0], gt_y=g['obstacle'][1])
        candidates.append(row)
    ids = sorted({t['id'] for t in candidates})
    chosen = max(ids, key=lambda i: sum(
        t['id'] == i and t['position_error'] < .45 and start <= t['elapsed'] <= start + 8.8
        for t in candidates)) if ids else None
    actor = [t for t in candidates if t['id'] == chosen]
    write_csv(directory / 'actor_aligned.csv', actor)
    result['actor_id'] = chosen
    # Include ALL samples from the matched ID; disclose transients separately.
    valid_motion = [t for t in actor if start <= t['elapsed'] <= start + 8.8]
    mature = [t for t in valid_motion if t['elapsed'] >= start + .8 and t['confidence'] >= .65]
    result['all_moving_interval'] = stats(valid_motion)
    result['mature_confident_interval'] = stats(mature)
    result['latency_now_minus_stamp'] = [
        min((t['age'] for t in actor), default=0),
        max((t['age'] for t in actor), default=0)]
    result['velocity_truth'] = [0., .25]
    return result


def main():
    """Retain every attempted run, including failed preflight launches."""
    results = [analyze(p.parent) for p in sorted((HERE / 'runs').glob('*/manifest.json'))]
    (HERE / 'analysis.json').write_text(json.dumps(results, indent=2) + '\n')
    for r in results:
        print(json.dumps(r))


if __name__ == '__main__':
    sys.exit(main())
