#!/usr/bin/env python3
"""Read-only V0.1 diagnostics: explicit event rules, native/reception clocks and plots."""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import struct

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[1] / 'results' / 'dynamic' / 'formal_v0_1_20261020'
ROBOT = [-.1965, -.153, .077, .153]


def distribution(values, budget=50.):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return dict(n=0, mean=None, median=None, p95=None, p99=None, maximum=None,
                    over_budget_fraction=None)
    def percentile(q):
        index = (len(values)-1)*q
        a = int(index)
        b = min(a+1, len(values)-1)
        return values[a]+(values[b]-values[a])*(index-a)
    return dict(n=len(values), mean=statistics.mean(values), median=statistics.median(values),
                p95=percentile(.95), p99=percentile(.99), maximum=max(values),
                over_budget_fraction=sum(v > budget for v in values)/len(values))


def first_response(samples, after, before, persistence=.2):
    since = None
    for t, vx, wz in samples:
        if t < after or t >= before:
            continue
        if vx <= .21 or abs(wz) >= .1:
            since = t if since is None else since
            if t-since >= persistence-1e-8:
                return since
        else:
            since = None
    return None


def in_interaction(actor, scene):
    # Physical obstacle has entered nominal straight-route robot sweep. This is
    # deliberately NOT a claim about earliest predictable future risk.
    return 0 <= actor[0] <= 4 and abs(actor[1]) <= .153+scene['size'][1]/2


def jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line]


def pose_at(records, t):
    return min(records, key=lambda row: abs(row['t']-t))


def scan_association(row, size):
    # Slab ray-box intersection + endpoint consistency, rather than nearest range.
    x, y, angle = row['laser_world']
    ox, oy, yaw = row['obstacle_world']
    c, s = math.cos(yaw), math.sin(yaw)
    u, v = c*(x-ox)+s*(y-oy), -s*(x-ox)+c*(y-oy)
    hits = []
    for i, radius in enumerate(row['ranges']):
        if radius is None or radius < row.get('range_min', struct.unpack('<f', struct.pack('<f', .12))[0]) or radius > row.get('range_max', 20):
            continue
        a = angle+row['angle_min']+i*row['angle_increment']-yaw
        near, far = 0., math.inf
        for offset, direction, half in ((u, math.cos(a), size[0]/2),
                                        (v, math.sin(a), size[1]/2)):
            if abs(direction) < 1e-12:
                if abs(offset) > half:
                    far = -1
                    break
            else:
                lo, hi = sorted(((-half-offset)/direction, (half-offset)/direction))
                near, far = max(near, lo), min(far, hi)
        if far >= near and abs(radius-near) <= .08:
            hits.append((i, radius))
    indices = {i for i, _ in hits}
    contiguous = any(i+1 in indices and i+2 in indices for i in indices)
    return dict(hit_rays=len(hits), valid=len(hits) >= 3 and contiguous,
                nearest_range_m=min((r for _, r in hits), default=None))


def cost_association(row, size):
    x, y, angle = row['obstacle_grid_pose']
    c, s = math.cos(angle), math.sin(angle)
    count = 0
    for j in range(row['height']):
        for i in range(row['width']):
            dx = row['origin'][0]+(i+.5)*row['resolution']-x
            dy = row['origin'][1]+(j+.5)*row['resolution']-y
            if abs(c*dx+s*dy) <= size[0]/2 and abs(-s*dx+c*dy) <= size[1]/2:
                count += row['data'][j*row['width']+i] == 100
    return dict(lethal_cells=count, valid=count >= 3)


def parse_diagnostics(log):
    result = []
    for line in log.splitlines():
        if '[DWA_DEBUG]' not in line:
            continue
        time_match = re.search(r'\[([0-9]+\.[0-9]+)\]', line)
        stages = re.search(r'ms\[total,state_wait,path,map_wait,predict,obstacle,score\]=\[([^]]+)\]', line)
        cumulative = re.search(r'cumulative\[calls,over50,mean_ms,max_ms\]=\[([^]]+)\]', line)
        best = re.search(r'best=\{present:([^,]+),v:([^,]+),w:([^,]+)', line)
        if stages and cumulative:
            result.append(dict(system_log_s=float(time_match.group(1)),
                timings=dict(zip(('total', 'state_wait', 'path', 'map_wait', 'predict', 'obstacle', 'score'),
                                 map(float, stages.group(1).split(',')))),
                cumulative=list(map(float, cumulative.group(1).split(','))),
                selected_v=float(best.group(2)) if best else None,
                selected_w=float(best.group(3)) if best else None))
    return result


def analyze_trial(source, controller, repeat, output):
    relative = f'D01/{controller}/trial_{repeat:03}'
    directory = source / relative
    summary = json.loads((directory / 'summary.json').read_text())
    epoch = summary['navigation_start_sim_s']
    gt = jsonl(directory / 'ground_truth.jsonl')
    contacts = jsonl(directory / 'contacts.jsonl')
    contact = min(contacts, key=lambda row: row['stamp'])
    contact_t = contact['stamp']
    scene = json.loads((directory / 'configs' / 'dynamic_scenarios.json').read_text())['D01']
    motion = jsonl(directory / 'motion.jsonl')
    initial_y = scene['keyframes'][0][2]
    moved = next(r for r in gt if abs(r['obstacle'][1]-initial_y) > .005)
    interaction = next((r for r in gt if in_interaction(r['obstacle'], scene)), None)
    scans, costs = [], []
    with gzip.open(directory / 'sensors.jsonl.gz', 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            result = scan_association(row, scene['size']) if row['type'] == 'scan' else cost_association(row, scene['size'])
            result.update(t=row['t'], stamp=row['stamp'])
            (scans if row['type'] == 'scan' else costs).append(result)
    scan = next((r for r in scans if r['valid']), None)
    cost = next((r for r in costs if r['valid']), None)
    trajectory = list(csv.DictReader((directory / 'trajectory.csv').open()))
    raw = []
    for row in trajectory:
        if row['controller_cmd_receipt_sim_s'] and row['controller_vx'] and row['controller_wz']:
            sample = (float(row['controller_cmd_receipt_sim_s']), float(row['controller_vx']), float(row['controller_wz']))
            if not raw or sample != raw[-1]:
                raw.append(sample)
    effective = [(r['t'], r['vx'], r['wz']) for r in jsonl(directory / 'commands.jsonl')]
    actual = [(r['t'], r['vx'], r['wz']) for r in gt]
    # Remove startup acceleration; check both after motion and after physical
    # route entry, and exclude contact/cancellation entirely.
    brake = first_response([(t,v,0) for t,v,w in raw], epoch+4, contact_t-.1)
    turn = first_response([(t,.26,w) for t,v,w in raw], epoch+4, contact_t-.1)
    response = min((t for t in (brake,turn) if t is not None), default=None)
    actual_candidate = first_response(actual, epoch+4, contact_t-.1)
    response_kind = 'deceleration' if brake==response and response is not None else ('turn' if turn is not None else None)
    matching_actual = [(t,v,0) if response_kind=='deceleration' else (t,.26,w) for t,v,w in actual]
    actual_response = first_response(matching_actual, response, contact_t-.1) if response is not None else None
    events = dict(T0=epoch, T1=moved['t'], T2=interaction['t'] if interaction else None,
                  T3=scan['stamp'] if scan else None, T4=cost['stamp'] if cost else None,
                  T5=response, T6=actual_response, T7=contact_t)
    locations = {}
    for name, t in events.items():
        if t is not None:
            observation = moved if name=='T1' else (interaction if name=='T2' else pose_at(gt, t))
            locations[name] = dict(robot=observation['robot'], obstacle=observation['obstacle'],
                gap_approx_m=observation['gap_approx_m'], nearest_gt_receipt_t=observation['t'],
                actual_vx=observation['vx'], actual_wz=observation['wz'])
    log = (source / 'logs' / f'D01_{controller}_trial_{repeat:03}_launch.log').read_text()
    diagnostics = parse_diagnostics(log)
    report = dict(run=relative, source=str(directory), epoch=epoch,
        events={k: None if v is None else v-epoch for k, v in events.items()},
        event_native_or_receipt_absolute=events, locations=locations,
        timing_limits=dict(T0='latest received /clock at dispatch, not exact acceptance',
          T1_T2_T5_T6='receipt /clock quantized about 0.1 s; controller sampled <=10 Hz',
          T3_T4='FIRST RETAINED associated source stamps after recorder enabled at T0; true onset may precede T0. Not perception latency.',
          T7='physical contact source stamp, nominal contact 100 Hz; epoch still has receipt-clock uncertainty'),
        first_scan=scan, first_costmap=cost, contact=contact,
        response_kind=response_kind,
        first_brake_sim_relative=None if brake is None else brake-epoch,
        first_turn_sim_relative=None if turn is None else turn-epoch,
        unmatched_actual_change_sim_relative=None if actual_candidate is None else actual_candidate-epoch,
        pre_contact_command_min_v=min((v for t,v,w in raw if epoch+4<=t<contact_t-.1), default=None),
        pre_contact_command_max_abs_w=max((abs(w) for t,v,w in raw if epoch+4<=t<contact_t-.1), default=None),
        raw_controller_samples=len(raw), effective_command_samples=len(effective),
        warnings=log.count('Control loop missed'), sparse_diagnostics=diagnostics,
        source_sha256={name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
                       for name in ('summary.json', 'ground_truth.jsonl', 'contacts.jsonl',
                                    'commands.jsonl', 'trajectory.csv', 'sensors.jsonl.gz')})
    destination = output / controller / f'trial_{repeat:03}'
    destination.mkdir(parents=True, exist_ok=True)
    (destination / 'events.json').write_text(json.dumps(report, indent=2)+'\n')
    plots(destination, controller, repeat, epoch, events, gt, raw, effective, locations)
    return report


def plots(destination, controller, repeat, epoch, events, gt, raw, effective, locations):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'figure.dpi': 130})
    label = f'D01 / {controller} / trial_{repeat:03} / V0.1 formal'
    source = 'Sources: Gazebo model_states/contact; trajectory.csv /cmd_vel_nav; native /cmd_vel'
    times = [r['t']-epoch for r in gt]
    def markers(axes):
        for ax in axes:
            ax.axvline(events['T7']-epoch, color='crimson', ls='--', label='first recorded physical contact')
            for name, color in [('T1', '#888888'), ('T2', '#b48213'), ('T5', '#168475'), ('T6', '#663399')]:
                if events[name] is not None:
                    ax.axvline(events[name]-epoch, color=color, ls=':', alpha=.8, label=name)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for ax, index, key, unit in [(axes[0], 1, 'vx', 'linear m/s'), (axes[1], 2, 'wz', 'angular rad/s')]:
        ax.step([r[0]-epoch for r in raw], [r[index] for r in raw], where='post', label='/cmd_vel_nav sampled <=10 Hz')
        ax.step([r[0]-epoch for r in effective], [r[index] for r in effective], where='post', alpha=.7, label='/cmd_vel smoothed')
        ax.plot(times, [r[key] for r in gt], alpha=.8, label='actual ground truth')
        ax.set_ylabel(unit)
    markers(axes)
    axes[0].legend(fontsize=8, loc='best')
    axes[1].set_xlabel('simulation s relative to dispatch (not exact action acceptance)')
    fig.suptitle(label+' / speed')
    fig.text(.02, .01, source, fontsize=8)
    fig.tight_layout(rect=[0, .03, 1, .96])
    fig.savefig(destination / 'speed.png')
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot([r['robot'][0] for r in gt], [r['robot'][1] for r in gt], label='robot GT')
    ax.plot([r['obstacle'][0] for r in gt], [r['obstacle'][1] for r in gt], label='obstacle GT')
    for name, color in [('T0', '#aaaaaa'), ('T1', '#ee9922'), ('T7', 'crimson')]:
        if name not in locations:
            continue
        row = locations[name]
        for kind, bounds in [('robot', ROBOT), ('obstacle', [-.25, -.25, .25, .25])]:
            x, y, angle = row[kind]
            c, s = math.cos(angle), math.sin(angle)
            a, b, d, e = bounds
            points = [(x+c*u-s*v, y+s*u+c*v) for u, v in ((a,b),(d,b),(d,e),(a,e))]
            ax.add_patch(Polygon(points, fill=False, color=color, lw=1.5))
        ax.annotate(name, row['robot'][:2])
    ax.set(xlabel='world x m', ylabel='world y m', title=label+' / XY + approximate envelopes', aspect='equal')
    ax.legend()
    fig.text(.02, .01, 'Source: ground_truth.jsonl; envelope is conservative rectangle; T7 envelopes use nearby receipt-clock GT, not exact contact pose.', fontsize=8)
    fig.tight_layout(rect=[0, .03, 1, 1])
    fig.savefig(destination / 'trajectory.png')
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(times, [r['gap_approx_m'] for r in gt], label='approximate dynamic clearance')
    markers([ax])
    ax.set(xlabel='simulation s relative to dispatch', ylabel='approximate gap m', title=label+' / clearance')
    ax.legend(fontsize=8)
    fig.text(.02, .01, 'Source: ground_truth.jsonl; 50 Hz nominal states, receipt /clock quantized ~0.1 s. Not exact collision union.', fontsize=8)
    fig.tight_layout(rect=[0, .04, 1, 1])
    fig.savefig(destination / 'clearance.png')
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 5))
    descriptions = {'T0':'dispatch clock proxy', 'T1':'observed first movement', 'T2':'nominal route envelope entry',
        'T3':'first RETAINED associated scan rays', 'T4':'first RETAINED lethal cells in box',
        'T5':'sustained command response', 'T6':'sustained actual response', 'T7':'first contact source stamp'}
    for i, (name, t) in enumerate(events.items()):
        if t is None:
            ax.text(.2, i, 'undetermined / criterion not observed before contact', va='center', fontsize=8)
        else:
            relative = t-epoch
            ax.errorbar(relative, i, xerr=.1 if name != 'T7' else .01, fmt='o', capsize=4)
            ax.text(relative+.12, i, f'{relative:.1f} s', va='center', fontsize=8)
    ax.set_yticks(range(8), [f'{k}: {descriptions[k]}' for k in events])
    ax.invert_yaxis()
    ax.set(xlabel='simulation s relative to dispatch', title=label+' / event timeline')
    fig.text(.02, .01, 'Bars: receipt-clock scale (~0.1 s), contact sampling 0.01 s; NOT detection onset bounds. T3/T4 may precede recording.', fontsize=8)
    fig.tight_layout(rect=[0, .04, 1, 1])
    fig.savefig(destination / 'timeline.png')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=SOURCE)
    parser.add_argument('--output', type=Path, default=HERE / 'outputs' / 'v0_1_analysis')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows = [analyze_trial(args.source, c, n, args.output)
            for c in ('mydwa', 'dwb') for n in (1, 2, 3)]
    (args.output / 'all_events.json').write_text(json.dumps(rows, indent=2)+'\n')
    print(json.dumps([{k:r[k] for k in ('run','events','warnings')} for r in rows], indent=2))


if __name__ == '__main__':
    main()