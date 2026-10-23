"""Read prior raw scans/TF/maps and separate surface motion from risk-state gates."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path('/home/user/ros2_ws/src/my_dwa_controller')
OLD = ROOT / 'benchmark/scan_trend_20261022'
OUT = ROOT / 'benchmark/scan_emergency_20261022'
spec = importlib.util.spec_from_file_location('prior', OLD / 'analyze.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)


def messages(directory):
    """Read original bag records with source timestamps left intact."""
    for p in sorted((directory / 'bag').glob('*.db3')):
        con = sqlite3.connect(f'file:{p}?mode=ro', uri=True)
        types = {i: (n, get_message(t)) for i, n, t in
                 con.execute('select id,name,type from topics')}
        for i, receipt, raw in con.execute(
                'select topic_id,timestamp,data from messages order by timestamp,id'):
            name, cls = types[i]
            if name in ('/scan', '/plan', '/local_costmap/costmap'):
                yield name, receipt * 1e-9, deserialize_message(raw, cls)
        con.close()


def clusters(scan, pose):
    """Exactly reproduce the current cluster gap and visible bounding-box support."""
    groups, group = [], []
    c, s = math.cos(pose[2]), math.sin(pose[2])
    for i, value in enumerate(scan.ranges):
        if not math.isfinite(value) or not scan.range_min <= value <= scan.range_max:
            if group:
                groups.append(group)
                group = []
            continue
        a = scan.angle_min + i * scan.angle_increment
        x, y = value * math.cos(a), value * math.sin(a)
        point = [pose[0] + c*x - s*y, pose[1] + s*x + c*y]
        if group and math.dist(point, group[-1]) > .12:
            groups.append(group)
            group = []
        group.append(point)
    if group:
        groups.append(group)
    if len(groups) > 1 and len(scan.ranges) * scan.angle_increment > 6.2:
        if math.dist(groups[0][0], groups[-1][-1]) <= .12:
            groups[0] += groups.pop()
    result = []
    for g in groups:
        if len(g) < 3:
            continue
        p = np.array(g)
        lo, hi = p.min(axis=0), p.max(axis=0)
        if .06 <= np.linalg.norm(hi-lo) <= .9:
            result.append(dict(x=float((lo[0]+hi[0])/2), y=float((lo[1]+hi[1])/2),
                               xmin=float(lo[0]), xmax=float(hi[0]),
                               ymin=float(lo[1]), ymax=float(hi[1]), points=len(g)))
    return result


def patch_evidence(
        scan, pose, grid, center=(1.59, .02), half_length=.39, width=.285, approach=False):
    """Audit a small original crossing patch, independently of cluster identity."""
    covered = ray_clear = map_clear = valid = no_return = 0
    for x in np.arange(center[0]-half_length, center[0]+half_length+.001, .05):
        for y in np.arange(-1.34 if approach else center[1]-width, center[1]+width+.001, .05):
            covered += 1
            dx, dy = x-pose[0], y-pose[1]
            angle = (math.atan2(dy, dx)-pose[2]-scan.angle_min) % (2*math.pi)
            index = int(round(angle/scan.angle_increment)) % len(scan.ranges)
            distance = math.hypot(dx, dy)
            r = scan.ranges[index]
            # Positive infinity is audited separately as an explicit no return.
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                valid += 1
                ray_clear += distance + .04 < r
            elif r == math.inf:
                no_return += 1
                ray_clear += distance + .04 < scan.range_max
            mx = int(math.floor((x-grid.info.origin.position.x)/grid.info.resolution))
            my = int(math.floor((y-grid.info.origin.position.y)/grid.info.resolution))
            if 0 <= mx < grid.info.width and 0 <= my < grid.info.height:
                value = grid.data[my*grid.info.width+mx]
                map_clear += 0 <= value < 99 or (approach and abs(y-center[1]) > width)
    return dict(cells=covered, ray_clear=ray_clear, map_clear=map_clear,
                finite_rays=valid, infinite_rays=no_return,
                both_clear=ray_clear == covered and map_clear == covered)


def main():
    """Save diagnosis without touching any previous experiment output."""
    directory = OLD / 'runs/round3_D01_t2'
    epoch = json.loads((directory/'trial/summary.json').read_text())['navigation_start_sim_s']
    edges, static, _, _ = prior.export_bag(directory)
    actor = list(csv.DictReader((directory/'actor_aligned.csv').open()))
    risk = prior.log_records(directory/'launch.log', '[PATH_CONFLICT]')
    features, observations, swept = [], [], []
    grid = None
    for name, receipt, msg in messages(directory):
        if name == '/local_costmap/costmap':
            grid = msg
        if name != '/scan':
            continue
        t = prior.align.seconds(msg.header.stamp)
        pose = prior.align.lookup_pose(edges, static, 'odom', msg.header.frame_id, t)
        if pose is None or t < epoch:
            continue
        groups = clusters(msg, pose)
        previous = min(actor, key=lambda a: abs(float(a['stamp'])-t))
        if abs(float(previous['stamp'])-t) < .02:
            # Track positions in the log are extrapolated to query now.
            x = float(previous['x']) - float(previous['vx'])*float(previous['age'])
            y = float(previous['y']) - float(previous['vy'])*float(previous['age'])
            best = min(groups, key=lambda a: math.hypot(a['x']-x, a['y']-y), default=None)
            if best and math.hypot(best['x']-x, best['y']-y) < .04:
                features.append(dict(stamp=t, elapsed=t-epoch, **best,
                                     old_vx=previous['vx'], old_vy=previous['vy'],
                                     confidence=previous['confidence']))
        if grid:
            observations.append(dict(stamp=t, elapsed=t-epoch,
                                     **patch_evidence(msg, pose, grid)))
            swept.append(dict(stamp=t, elapsed=t-epoch,
                              **patch_evidence(msg, pose, grid, approach=True)))
    prior.write_csv(OUT/'prior_surface_support.csv', features)
    prior.write_csv(OUT/'prior_patch_visibility.csv', observations)
    prior.write_csv(OUT/'prior_sweep_visibility.csv', swept)
    summary = dict(
        first_swept_clear_after_protection=next(
            (p for p in swept if p['elapsed'] >= 2.6 and p['both_clear']), None),
        swept_clear_after_12=sum(p['both_clear'] for p in swept if p['elapsed'] > 12),
        prior_source_matches=True, epoch=epoch,
        surface_intervals={str(a): [p for p in features if a <= p['elapsed'] <= b]
                           for a, b in ((2.8, 3.6), (4.8, 6), (7.4, 8.4))},
        first_full_patch_clear=next((p for p in observations if p['both_clear']), None),
        clear_patch_after_12=sum(p['both_clear'] for p in observations if p['elapsed'] > 12),
        patch_records_after_12=sum(p['elapsed'] > 12 for p in observations),
        risk_after_12=dict(
            records=sum(r['t']-epoch > 12 for r in risk),
            unresolved=sum(r['unresolved'] for r in risk if r['t']-epoch > 12),
            full_corridor_known=sum(r['known'] for r in risk if r['t']-epoch > 12)),
        first_protection=next(r for r in risk if r['state'] != 'normal'))
    (OUT/'diagnosis.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'surface_intervals'}, indent=2))


if __name__ == '__main__':
    main()
