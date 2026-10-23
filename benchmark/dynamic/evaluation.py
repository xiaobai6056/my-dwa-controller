"""Offline evaluation, conservative geometry, and explicit terminal classification."""
import json
import math
from pathlib import Path

# Conservative planar enclosure of stock waffle collision shapes, relative to
# base_footprint: base x=-.064 +/- .1325; wheels y=+/- .144 +/- .009;
# camera forward <= .077. This enclosure is an APPROXIMATION, not exact union.
ROBOT_BOUNDS = [-.1965, -.153, .077, .153]


def box_polygon(x, y, yaw, bounds):
    c, s = math.cos(yaw), math.sin(yaw)
    a, b, d, e = bounds
    return [(x + c * u - s * v, y + s * u + c * v)
            for u, v in ((a, b), (d, b), (d, e), (a, e))]


def point_segment(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    t = max(0., min(1., ((p[0] -
                          a[0]) *
                         dx +
                         (p[1] -
                          a[1]) *
                         dy) /
                    den)) if den else 0.
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def polygon_gap(a, b):
    separated = False
    for poly in (a, b):
        for p, q in zip(poly, poly[1:] + poly[:1]):
            axis = (p[1] - q[1], q[0] - p[0])
            pa = [x * axis[0] + y * axis[1] for x, y in a]
            pb = [x * axis[0] + y * axis[1] for x, y in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                separated = True
    if not separated:
        return 0.
    return min(point_segment(p, u, v) for pts, edges in ((a, b), (b, a))
               for p in pts for u, v in zip(edges, edges[1:] + edges[:1]))


def termination(action, reason, collision, valid):
    if not valid or reason in (
        'clock_stalled',
        'clock_reset',
        'motion_error',
            'wall_watchdog'):
        return 'system_error'
    if collision:
        return 'collision'
    if reason == 'sim_timeout':
        return 'timeout'
    return 'success' if action == 'SUCCEEDED' else 'navigation_failed'


def direction_switches(samples, deadband=.05, persistence=.2):
    last, candidate, since, switches = 0, 0, None, 0
    for t, w in samples:
        sign = 1 if w > deadband else (-1 if w < -deadband else 0)
        if not sign:
            candidate, since = 0, None
            continue
        if sign != candidate:
            candidate, since = sign, t
        if t - since >= persistence and sign != last:
            if last:
                switches += 1
            last = sign
    return switches


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text(
    ).splitlines()] if path.exists() else []


def calculate(directory):
    directory = Path(directory)
    summary = json.loads((directory / 'summary.json').read_text())
    obs = read_jsonl(directory / 'ground_truth.jsonl')
    commands = read_jsonl(directory / 'commands.jsonl')
    events = read_jsonl(directory / 'dynamic_events.jsonl')
    contacts = read_jsonl(directory / 'contacts.jsonl')
    start, end = summary.get('navigation_start_sim_s'), summary.get(
        'navigation_end_sim_s')

    def in_window(row):
        return start is not None and end is not None and start <= row['t'] <= end
    obs, commands = [
        r for r in obs if in_window(r)], [
        r for r in commands if in_window(r)]
    contacts = [r for r in contacts if in_window(r)]
    gaps = [r['gap_approx_m'] for r in obs]
    angular = [(r['t'], r['wz']) for r in commands]
    stopped = sum(b['t'] - a['t'] for a, b in zip(obs, obs[1:])
                  if b['t'] - a['t'] <= .3 and abs(a['vx']) < .02 and
                  abs(a['wz']) < .05)
    releases = [e for e in events if e['event'] == 'corridor_clear']
    release = releases[0]['t'] if releases else None
    resume = None
    if release is not None:
        since = None
        for row in obs:
            if row['t'] < release:
                continue
            if row['vx'] > .03:
                if since is None:
                    since = row['t']
                if row['t'] - since >= .5:
                    resume = since - release
                    break
            else:
                since = None
    sensor = read_jsonl(directory / 'perception.jsonl')
    scan = [r for r in sensor if r['type'] == 'scan' and in_window(r)]
    cost = [r for r in sensor if r['type'] == 'costmap' and in_window(r)]
    result = dict(
        scene=summary.get('scene'),
        controller=summary.get('controller'),
        termination=termination(
            summary.get('action_result'),
            summary.get('stop_reason'),
            bool(contacts),
            summary.get('valid', False) and summary.get('action_error') is None),
        action_result=summary.get('action_result'),
        action_accepted=summary.get('action_accepted', False),
        navigation_sim_s=(end - start
                          if start is not None and end is not None else None),
        collision=bool(contacts),
        contact_messages=len(contacts),
        min_gap_approx_m=min(gaps) if gaps else None,
        angular_total_variation_rad_s=sum(
            abs(b[1] - a[1]) for a, b in zip(angular, angular[1:])),
        angular_direction_switches=direction_switches(angular),
        stopped_sim_s=stopped if obs else None, release_sim_s=release,
        resume_after_release_s=resume, recoveries=summary.get(
            'max_feedback_recoveries'),
        gt_samples=len(obs),
        scan_samples=len(scan),
        scan_hit_samples=sum(r.get('hit_rays', 0) > 0 for r in scan),
        costmap_samples=len(cost),
        costmap_hit_samples=sum(r.get('lethal_cells', 0) > 0 for r in cost),
        error=summary.get('error') or summary.get('action_error'),
        stop_reason=summary.get('stop_reason'))
    result['success'] = result['termination'] == 'success'
    result['successful_completion_sim_s'] = (
        result['navigation_sim_s'] if result['success'] else None)
    return result
