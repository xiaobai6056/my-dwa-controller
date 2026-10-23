"""Audit isolated raw-Scan free-space pulses against nearby recorded truth."""
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
spec = importlib.util.spec_from_file_location(
    'old_diagnostic', ROOT / 'benchmark/scan_emergency_20261022/diagnose.py')
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)


def intersect(origin, direction, center, half=.25):
    """Ray intersection with the scenario's axis-aligned box in planar truth."""
    lower, upper = -math.inf, math.inf
    for o, v, c in zip(origin, direction, center):
        if abs(v) < 1e-10:
            if abs(o-c) > half:
                return None
            continue
        a, b = (c-half-o)/v, (c+half-o)/v
        lower, upper = max(lower, min(a, b)), min(upper, max(a, b))
    return (lower, upper) if upper >= max(0, lower) else None


def main():
    """Use exact-time sensor TF; label GT receipt-time uncertainty explicitly."""
    directory = HERE / 'runs/v2_t2_b_20261022_D01_t2'
    epoch = json.loads((directory/'trial/summary.json').read_text())['navigation_start_sim_s']
    edges, static, _, _ = old.prior.export_bag(directory)
    gt = old.prior.rows(directory/'trial/ground_truth.jsonl')
    rows = []
    for name, _, scan in old.messages(directory):
        if name != '/scan':
            continue
        stamp = old.prior.align.seconds(scan.header.stamp)
        if not 3.5 <= stamp-epoch <= 4.5:
            continue
        g = min(gt, key=lambda g: abs(g['t']-stamp))
        sensor = old.prior.align.lookup_pose(edges, static, 'odom', scan.header.frame_id, stamp)
        robot = old.prior.align.lookup_pose(edges, static, 'odom', 'base_footprint', stamp)
        if sensor is None or robot is None:
            continue
        world_odom = old.prior.align.compose(g['robot'], old.prior.align.inverse(robot))
        laser = old.prior.align.compose(world_odom, sensor)
        hits = through = expected = foreground = 0
        for i, r in enumerate(scan.ranges):
            angle = laser[2]+scan.angle_min+i*scan.angle_increment
            vector = [math.cos(angle), math.sin(angle)]
            cross = intersect(laser[:2], vector, g['obstacle'][:2], .20)
            valid = math.isfinite(r) and scan.range_min <= r <= scan.range_max
            if valid:
                endpoint = [laser[j]+r*vector[j] for j in (0, 1)]
                hits += all(abs(endpoint[j]-g['obstacle'][j]) < .30 for j in (0, 1))
            if cross is not None and cross[0] > scan.range_min:
                expected += 1
                if valid and r > cross[1]+.10:
                    through += 1
                if valid and r < cross[0]-.15:
                    foreground += 1
        rows.append(dict(stamp=stamp, source_elapsed=stamp-epoch,
                         gt_receipt=g['t'], gt_delta=g['t']-stamp,
                         robot=g['robot'], obstacle=g['obstacle'],
                         raw_endpoints_near_box=hits, rays_intersect_inner_box=expected,
                         background_through_box=through, foreground=foreground))
    old.prior.write_csv(HERE/'visibility_pulse.csv', rows)
    (HERE/'visibility_pulse.json').write_text(json.dumps(rows, indent=2)+'\n')
    for row in rows:
        print(row)


if __name__ == '__main__':
    main()
