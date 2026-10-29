"""Export original packets to a native, read-only Tracker/Monitor/Governor replay."""
import bisect
import hashlib
import importlib.util
import json
import math
from pathlib import Path


ARCHIVE_REPO = Path(__file__).resolve().parents[2]

ROOT = ARCHIVE_REPO
HERE = ROOT / 'benchmark/scan_emergency_20261022'
spec = importlib.util.spec_from_file_location('diagnose', HERE / 'diagnose.py')
diagnose = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnose)
prior = diagnose.prior
# Recorded AMCL TF is spaced up to .201s; the old plotting helper's .200s
# guard dropped valid interpolation brackets. Native TF permits these brackets.
interpolate = prior.align.interpolate_pose
prior.align.interpolate_pose = lambda samples, stamp, max_gap=.25: interpolate(
    samples, stamp, max_gap=max_gap)


def main():
    """Preserve source time TF and real path/map samples, without ROS publishers."""
    directory = ROOT / 'benchmark/scan_trend_20261022/runs/round3_D01_t2'
    epoch = json.loads((directory/'trial/summary.json').read_text())['navigation_start_sim_s']
    edges, static, _, odom = prior.export_bag(directory)
    scans, plans, maps = [], [], []
    for name, receipt, msg in diagnose.messages(directory):
        if name == '/scan':
            scans.append(msg)
        elif name == '/plan':
            plans.append((receipt, msg))
        else:
            maps.append((prior.align.seconds(msg.header.stamp), msg))
    plans.sort(key=lambda p: p[0])
    maps.sort(key=lambda p: p[0])
    statuses = prior.log_records(directory/'launch.log', '[SCAN_STATUS]')
    now_by_stamp = {round(p['stamp'], 6): p['now'] for p in statuses}
    count = 0
    output = HERE / 'prior_replay.input'
    with output.open('w') as f:
        f.write(f'{epoch}\n')
        for scan in scans:
            stamp = prior.align.seconds(scan.header.stamp)
            if round(stamp, 6) not in now_by_stamp:
                continue
            now = now_by_stamp[round(stamp, 6)]
            si = bisect.bisect_right([p[0] for p in plans], now)-1
            mi = bisect.bisect_right([p[0] for p in maps], now)-1
            if si < 0 or mi < 0:
                continue
            sensor = prior.align.lookup_pose(edges, static, 'odom', scan.header.frame_id, stamp)
            robot = prior.align.lookup_pose(edges, static, 'odom', 'base_footprint', now)
            plan = plans[si][1]
            transform = prior.align.lookup_pose(edges, static, 'odom', plan.header.frame_id, now)
            if sensor is None or robot is None or transform is None:
                continue
            grid = maps[mi][1]
            assert grid.header.frame_id == 'odom'
            v = min(odom, key=lambda p: abs(p['stamp']-now))
            header = [now, stamp, *sensor, *robot, v['v'], v['w'], len(scan.ranges),
                      scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max]
            f.write(' '.join(map(str, header))+'\n')
            f.write(' '.join(str(r if math.isfinite(r) else -1) for r in scan.ranges)+'\n')
            f.write(str(len(plan.poses))+'\n')
            for p in plan.poses:
                point = prior.align.compose(transform, [p.pose.position.x, p.pose.position.y,
                                                        prior.align.yaw(p.pose.orientation)])
                f.write(' '.join(map(str, point))+'\n')
            f.write(' '.join(map(str, [grid.info.width, grid.info.height, grid.info.resolution,
                                       grid.info.origin.position.x,
                                       grid.info.origin.position.y]))+'\n')
            costs = [255 if p < 0 else 254 if p == 100 else 253 if p == 99 else
                     0 if p == 0 else 1 + round((p-1)*251/97) for p in grid.data]
            f.write(' '.join(map(str, costs))+'\n')
            count += 1
    (HERE/'replay_input_manifest.json').write_text(json.dumps(dict(
        frames=count, input_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
        source_bag=str(directory/'bag'), epoch=epoch,
        caveat=('Recorded robot motion is held fixed; '
                'replay cannot establish new closed-loop behavior.')
    ), indent=2)+'\n')
    print(count, output)


if __name__ == '__main__':
    main()
