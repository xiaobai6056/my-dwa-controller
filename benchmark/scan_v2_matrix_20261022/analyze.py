"""Read-only matrix diagnosis using frozen collectors and previous bag/TF helpers."""
import bisect
import csv
import importlib.util
import json
import math
from pathlib import Path
import re

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
spec = importlib.util.spec_from_file_location(
    'prior_scan_tools', ROOT / 'benchmark/scan_trend_20261022/analyze.py')
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)


def records(path, marker):
    """Retain source line and logging system timestamp, separately from ROS time."""
    result = prior.log_records(path, marker)
    lines = path.read_text().splitlines()
    for row in result:
        match = re.search(r'\[(?:INFO|WARN|ERROR)\] \[([0-9.]+)\]', lines[row['line']-1])
        row['log_system_stamp'] = float(match[1]) if match else None
    return result


def read_csv(path):
    """Read the unmodified collector's table."""
    with path.open() as stream:
        return list(csv.DictReader(stream))


def event(row, epoch):
    """Keep both dispatch-aligned and original evidence fields."""
    return None if row is None else dict(row, elapsed=row['t']-epoch)


def first(rows, predicate):
    """Find evidence without substituting a missing event with zero."""
    return next((r for r in rows if predicate(r)), None)


def sustained_stop(gt, after, before, angular=False):
    """Use 0.3s observed persistence and reject GT gaps above 0.25s."""
    begin = previous = None
    for g in gt:
        if not after <= g['t'] <= before:
            continue
        if previous is not None and g['t']-previous > .25:
            begin = None
        previous = g['t']
        if abs(g['vx']) <= .01 and (not angular or abs(g['wz']) <= .05):
            begin = g if begin is None else begin
            if g['t']-begin['t'] >= .3-1e-8:
                return begin
        else:
            begin = None
    return None


def cycles(directory, epoch):
    """Keep exact cumulative compute timing and approximate terminal phase timing."""
    lines = (directory / 'launch.log').read_text().splitlines()
    nav = (directory / 'trial.log').read_text()
    match = re.search(r'\[INFO\] \[([0-9.]+)\].*Navigating to goal', nav)
    dispatch_wall = float(match[1]) if match else None
    trajectory = read_csv(directory / 'trial/trajectory.csv')
    trajectory = [r for r in trajectory if float(r['sim_time_s']) >= epoch]
    result = []
    for n, line in enumerate(lines, 1):
        if '[DWA_DEBUG]' not in line:
            continue
        cumulative = re.search(r'cumulative\[calls,over50,mean_ms,max_ms\]=\[([^]]+)\]', line)
        wall = re.search(r'\[INFO\] \[([0-9.]+)\]', line)
        phase = re.search(r'goal_phase=(\w+)', line)
        if cumulative is None or wall is None:
            continue
        c = list(map(float, cumulative[1].split(',')))
        row = dict(line=n, log_system_stamp=float(wall[1]), calls=int(c[0]),
                   over50=int(c[1]), mean_ms=c[2], max_ms=c[3],
                   phase=phase[1] if phase else 'unknown')
        if dispatch_wall is not None and trajectory:
            relative = row['log_system_stamp']-dispatch_wall
            nearest = min(trajectory, key=lambda r: abs(float(r['wall_elapsed_s'])-relative))
            row['sim_estimate'] = float(nearest['sim_time_s'])
            row['elapsed_estimate'] = row['sim_estimate']-epoch
            row['wall_match_residual'] = abs(float(nearest['wall_elapsed_s'])-relative)
        for name, pattern in [('pose', r'pose=\(([^)]+)\)'),
                              ('input', r'input_vel=\(([^)]+)\)'),
                              ('chosen', r'best=\{present:1,v:([^,]+),w:([^,]+)')]:
            m = re.search(pattern, line)
            if m:
                row[name] = [float(x) for x in (m[1].split(',') if name != 'chosen' else m.groups())]
        for name in ('yaw_error', 'yaw_tolerance', 'true_goal_distance'):
            m = re.search(r'\b'+name+r'=([0-9.e+-]+)', line)
            if m:
                row[name] = float(m[1])
        result.append(row)
    return result


def analyze(directory, name, dest):
    """Write only to the new diagnosis directory; archived baselines remain read-only."""
    dest.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((directory / 'manifest.json').read_text())
    summary = json.loads((directory / 'trial/summary.json').read_text())
    epoch = summary['navigation_start_sim_s']
    end = summary['navigation_end_sim_s']
    scene = json.loads((directory / 'effective_scene.json').read_text())
    risk = records(directory / 'launch.log', '[PATH_CONFLICT]')
    tracks = records(directory / 'launch.log', '[SCAN_TRACK]')
    status = records(directory / 'launch.log', '[SCAN_STATUS]')
    fallback = records(directory / 'launch.log', '[FALLBACK_SELECTION]')
    gt = prior.rows(directory / 'trial/ground_truth.jsonl')
    contacts = prior.rows(directory / 'trial/contacts.jsonl')
    perception = prior.rows(directory / 'trial/perception.jsonl')
    commands = prior.rows(directory / 'trial/commands.jsonl')
    motion = prior.rows(directory / 'trial/motion.jsonl')
    control = cycles(directory, epoch)
    edges, static, scans, odom = prior.export_bag(directory)
    for group in (risk, fallback):
        for row in group:
            row['elapsed'] = row['t']-epoch
    for row in tracks:
        row['source_elapsed'] = row['stamp']-epoch
        row['query_elapsed'] = row['now']-epoch
    for row in status:
        row['source_elapsed'] = row['stamp']-epoch
        row['query_elapsed'] = row['now']-epoch
    for row in scans:
        row['source_elapsed'] = row['stamp']-epoch
        row['bag_receipt_elapsed'] = row['receipt']-epoch
        row['receipt_minus_source'] = row['receipt']-row['stamp']
    for rows, file in [(risk, 'risk'), (tracks, 'tracks'), (status, 'scan_status'),
                       (fallback, 'fallback'), (control, 'control_cycles'), (scans, 'scan_delivery'),
                       (gt, 'ground_truth'), (commands, 'commands')]:
        prior.write_csv(dest / (file+'.csv'), rows)
    transitions = []
    for row in risk:
        if not transitions or transitions[-1]['state'] != row['state']:
            transitions.append(row)
    r0 = first(risk, lambda r: r['state'] != 'normal')
    predicted = first(risk, lambda r: r['predicted'])
    clear = first(risk, lambda r: r.get('area_clear'))
    recovering = first(risk, lambda r: r['state'] == 'recovering')
    release = first(risk, lambda r: r.get('release'))
    risk_end = recovering['t'] if recovering else end
    linear_stop = sustained_stop(gt, r0['t'], risk_end) if r0 else None
    full_stop = sustained_stop(gt, r0['t'], risk_end, True) if r0 else None
    minimum = min(gt, key=lambda g: g['gap_approx_m'])
    actor_rows = []
    for t in tracks:
        if not t['observed']:
            continue
        g = min(gt, key=lambda g: abs(g['t']-t['now']))
        robot = prior.align.lookup_pose(edges, static, 'odom', 'base_footprint', t['now'])
        if robot is None or abs(g['t']-t['now']) > .15:
            continue
        world_odom = prior.align.compose(g['robot'], prior.align.inverse(robot))
        world = prior.align.compose(world_odom, [t['x'], t['y'], 0])
        error = math.hypot(world[0]-g['obstacle'][0], world[1]-g['obstacle'][1])
        if error > .45 or abs(world[0]-1.6) > .4:
            continue
        c, s = math.cos(world_odom[2]), math.sin(world_odom[2])
        actor_rows.append(dict(t, world_x=world[0], world_y=world[1],
                               world_vx=c*t['vx']-s*t['vy'], world_vy=s*t['vx']+c*t['vy'],
                               position_error=error, gt_receipt_delta=g['t']-t['now']))
    # Preserve all spatial candidates; velocities never select the truth label.
    prior.write_csv(dest / 'actor_candidates.csv', actor_rows)
    accepted = [r for r in actor_rows if r['motion'] == 'moving' and
                scene['keyframes'][1][0] <= r['source_elapsed'] <= scene['keyframes'][2][0]]
    first_motion = first(actor_rows, lambda r: r['motion'] == 'moving' and
                         r['source_elapsed'] >= scene['keyframes'][1][0])
    protected_gt = [g for g in gt if r0 and r0['t'] <= g['t'] <= risk_end]
    after_stop = [g for g in protected_gt if linear_stop and g['t'] >= linear_stop['t']]
    risk_wait = [r for r in risk if r['state'] == 'wait']
    stale = [r for r in risk if not r['fresh']]
    result = dict(name=name, directory=str(directory), epoch=epoch, valid=summary.get('valid'),
                  result=summary.get('action_result'), stop_reason=summary.get('stop_reason'),
                  duration=end-epoch, wall_duration=summary.get('navigation_wall_s'),
                  configured_motion_start=scene['keyframes'][1][0],
                  first_gt_motion=event(first(gt, lambda g: g['obstacle'][1] > -1.095), epoch),
                  physical_contact=bool(contacts), first_contact=contacts[0] if contacts else None,
                  contact_callbacks=summary['observation_counts']['contact_callbacks'],
                  min_gap_approx=minimum['gap_approx_m'], min_gap_event=event(minimum, epoch),
                  first_accepted_motion=first_motion, first_prediction=predicted, first_risk=r0,
                  transitions=transitions, first_local_clear=clear, first_recovering=recovering,
                  releases=[r for r in risk if r.get('release')],
                  first_linear_stop=event(linear_stop, epoch), first_full_stop=event(full_stop, epoch),
                  wait_records=len(risk_wait),
                  protected_max_abs_w=max((abs(g['wz']) for g in protected_gt), default=None),
                  after_linear_stop_max_abs_w=max((abs(g['wz']) for g in after_stop), default=None),
                  first_move_after_clear=event(first(gt, lambda g: clear and
                                                   g['t'] > clear['t'] and g['vx'] > .05), epoch),
                  accepted_actor_samples=len(accepted),
                  accepted_actor_ids=sorted({r['id'] for r in accepted}),
                  unknown_actor_samples=sum(r['motion'] == 'unknown' for r in actor_rows),
                  actor_candidates=len(actor_rows),
                  max_tf_failures=max((s['tf_failures'] for s in status), default=None),
                  max_scan_dropped=max((s['dropped'] for s in status), default=None),
                  stale_risk_records=len(stale),
                  stale_risk_times=[r['elapsed'] for r in stale],
                  max_track_age=max((r['age'] for r in tracks), default=None),
                  scan_count=len(scans), scan_max_gap=float(max(np.diff([s['stamp'] for s in scans]))),
                  scan_median_period=float(np.median(np.diff([s['stamp'] for s in scans]))),
                  status_max_query_age=max((s['now']-s['stamp'] for s in status), default=None),
                  last_control_cumulative=control[-1] if control else None,
                  first_terminal_phase=first(control, lambda r: r['phase'] != 'normal'),
                  logged_control_cycles=len(control),
                  monitor_ms_max=max((r['ms'] for r in risk), default=None),
                  plugin_sha256=manifest['plugin_sha256'],
                  mapping_verified=manifest.get('runtime_mapping_verified'),
                  parameters_sha256=__import__('hashlib').sha256(
                      (directory / 'params_enabled.yaml').read_bytes()).hexdigest(),
                  runtime_parameters=summary.get('runtime_parameters'),
                  source_sha256=manifest['source_sha256'])
    if accepted:
        vx = np.array([r['world_vx'] for r in accepted])
        vy = np.array([r['world_vy'] for r in accepted])
        result['accepted_motion_stats'] = dict(vx_median=float(np.median(vx)),
                                               vy_median=float(np.median(vy)),
                                               vy_min=float(min(vy)), vy_max=float(max(vy)),
                                               vector_rmse=float(np.sqrt(np.mean(vx**2+(vy-.25)**2))))
    return result


def clean(value):
    """Keep the summary JSON standards-compliant despite raw inf diagnostics."""
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, list):
        return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main():
    """Include only finished, individually admitted runs plus one read-only baseline."""
    results = []
    for p in sorted((HERE / 'runs').glob('*/manifest.json')):
        if not json.loads(p.read_text()).get('finished_utc'):
            continue
        results.append(analyze(p.parent, p.parent.name, HERE / 'analysis' / p.parent.name))
    baseline = ROOT / 'benchmark/scan_emergency_20261022/runs/round2_D01_t2'
    results.append(analyze(baseline, 'prior_v2_t2', HERE / 'analysis/prior_v2_t2'))
    (HERE / 'analysis.json').write_text(json.dumps(clean(results), indent=2, allow_nan=False)+'\n')
    for r in results:
        print(json.dumps({k:r[k] for k in ('name', 'result', 'stop_reason', 'duration',
                                          'physical_contact', 'min_gap_approx', 'wait_records')}))


if __name__ == '__main__':
    main()
