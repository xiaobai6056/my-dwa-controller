"""Explain the two retained collisions and replay their recorded governor evidence."""

import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import re
import shlex
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OLD = ROOT / 'benchmark/path_conflict_v01_20261022'
BUILD = Path('/home/user/ros2_ws/build_path_conflict_v01/my_dwa_controller')
CASES = ['prefix_fix_D01_t2', 'prefix_fix_D01_t4']


def write_csv(path, rows):
    """Write derived data without touching any original run file."""
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def build_replay(label):
    """Reuse the existing GTest target's compile/link metadata for a tiny native CLI."""
    target = BUILD / 'CMakeFiles/test_risk_governor.dir'
    flags = dict(re.findall(r'^(CXX_\w+) = (.*)$', (target / 'flags.make').read_text(), re.M))
    includes = shlex.split(flags['CXX_INCLUDES']) + shlex.split(flags['CXX_DEFINES'])
    link = shlex.split((target / 'link.txt').read_text())
    output = HERE / ('replay_' + label)
    command = [link[0], *includes]
    for token in link[1:]:
        if token.endswith('test_risk_governor.cpp.o'):
            command.append(str(HERE / 'replay_governor.cpp'))
        elif token == 'test_risk_governor':
            command.append(str(output))
        else:
            command.append(token)
    result = subprocess.run(command, cwd=BUILD, capture_output=True, text=True, check=True)
    (HERE / ('build_' + label + '.log')).write_text(result.stdout + result.stderr)
    return output, command


def records(directory, summary):
    """Extract logged evidence plus separate, source-stamped odometry observations."""
    odom = [json.loads(line) for line in
            (directory / 'trial/localization.jsonl').read_text().splitlines()
            if json.loads(line)['source'] == 'odom']
    # The collector's odometry localization file has pose, not velocity. Velocity
    # is retained in trajectory.csv, with its own odom_stamp_s.
    trajectory = list(csv.DictReader((directory / 'trial/trajectory.csv').open()))
    cp = summary['runtime_parameters']['controller_server']
    dt, a = 1.0 / cp['controller_frequency'], cp['FollowPath.path_conflict.brake_acceleration']
    delay = cp['FollowPath.path_conflict.reaction_time'] + dt
    result = []
    for index, text in enumerate((directory / 'launch.log').read_text().splitlines(), 1):
        if '[PATH_CONFLICT]' not in text:
            continue
        f = dict(re.findall(r'(\w+)=([^\s]+)', text.split('[PATH_CONFLICT]', 1)[1]))
        stamp = float(f['t'])
        if not summary['navigation_start_sim_s'] <= stamp <= summary['navigation_end_sim_s']:
            continue
        # Raw PATH_CONFLICT logs do not carry the exact input velocity. Invert the
        # printed stopping-distance formula and propagate its 0.001m rounding.

        def infer_v(distance):
            return a * (math.sqrt(delay * delay + 2.0 * max(0.0, distance) / a) - delay)
        stopping = float(f['stop'])
        samples = [r for r in trajectory
                   if r['odom_stamp_s'] and float(r['odom_stamp_s']) <= stamp]
        observation = max(samples, key=lambda r: float(r['odom_stamp_s'])) if samples else None
        core = f['core'].split('/')
        result.append(dict(
            line=index, stamp=stamp, relative_s=stamp - summary['navigation_start_sim_s'],
            state=f['state'], map_id=int(f['map']), kind=int(f['kind']),
            fresh=int(f['kind']) in (0, 2), known=int(f['known']), aligned=int(f['aligned']),
            inner=int(f['inner']), outer=int(f['outer']),
            inner_core=int(core[0]), outer_core=int(core[1]),
            region_start=float(f['region'].split(':')[0]), prefix=float(f['prefix']),
            cap=float(f['cap']), applied=float(f['applied']), stop=stopping,
            available=float(f['available']), inferred_v=infer_v(stopping),
            inferred_v_low=infer_v(stopping - 0.0005), inferred_v_high=infer_v(stopping + 0.0005),
            observed_v=None if observation is None else float(observation['actual_vx']),
            observed_w=0.0 if observation is None else float(observation['actual_wz']),
            odom_stamp=None if observation is None else float(observation['odom_stamp_s']),
            odom_pose_samples=len(odom)))
    return result, trajectory


def replay(executable, rows, summary, bound):
    """Replay evidence only; this is not a full controller or collision counterfactual."""
    cp = summary['runtime_parameters']['controller_server']
    mp = summary['runtime_parameters']['local_costmap/local_costmap']
    args = [mp['robot_radius'], mp['footprint_padding'],
            cp['FollowPath.path_conflict.brake_acceleration'],
            cp['FollowPath.path_conflict.reaction_time'],
            cp['FollowPath.path_conflict.stop_margin'],
            1.0 / cp['controller_frequency'], cp['FollowPath.trans_stopped_velocity'],
            cp['FollowPath.acc_lim_theta'], cp['FollowPath.path_conflict.clear_states'],
            cp['FollowPath.path_conflict.warning_speed']]
    velocity_key = 'inferred_v' + ('' if bound == 'mid' else '_' + bound)
    payload = '\n'.join(' '.join(map(str, [
        r['line'], r['stamp'], r['map_id'], r['kind'], int(r['fresh']), r['known'], r['aligned'],
        r['inner'], r['outer'], r['region_start'], r['prefix'], r[velocity_key], r['observed_w'],
        cp['FollowPath.max_vel_x']])) for r in rows) + '\n'
    completed = subprocess.run([str(executable), *map(str, args)], input=payload,
                               capture_output=True, text=True, check=True)
    return list(csv.DictReader(io.StringIO(completed.stdout))), payload, completed.stderr


def main():
    """Build a versioned replay and keep every input, bound check and source identity."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z_]+', args.label):
        parser.error('label must contain lowercase letters/underscores only')
    executable, build_command = build_replay(args.label)
    results = []
    for name in CASES:
        directory = OLD / 'runs' / name
        summary = json.loads((directory / 'trial/summary.json').read_text())
        rows, trajectory = records(directory, summary)
        write_csv(HERE / (name + '_observed.csv'), rows)
        write_csv(HERE / (name + '_trajectory.csv'), trajectory)
        bound_outputs = {}
        for bound in ['low', 'mid', 'high']:
            output, payload, diagnostic = replay(executable, rows, summary, bound)
            write_csv(HERE / f'{name}_{args.label}_{bound}.csv', output)
            (HERE / f'{name}_{bound}.input').write_text(payload)
            (HERE / f'{name}_{args.label}.radius.txt').write_text(diagnostic)
            bound_outputs[bound] = output
        mid = bound_outputs['mid']
        changes = [dict(line=r['line'], relative_s=r['relative_s'], logged=r['state'],
                        replay=q['state'], logged_cap=r['cap'], replay_cap=float(q['cap']))
                   for r, q in zip(rows, mid) if r['state'] != q['state']]
        stable = all([q['state'] for q in bound_outputs[b]] == [q['state'] for q in mid]
                     for b in ['low', 'high'])
        contacts = [json.loads(line) for line in
                    (directory / 'trial/contacts.jsonl').read_text().splitlines()]
        results.append(dict(case=name, records=len(rows), state_mismatches=changes,
                            state_stable_under_print_rounding=stable,
                            max_cap_error_vs_log=max(abs(float(q['cap']) - r['cap'])
                                                     for r, q in zip(rows, mid)),
                            contact_stamp=min(x['stamp'] for x in contacts),
                            epoch=summary['navigation_start_sim_s']))
    manifest = dict(label=args.label, cases=results, build_command=build_command,
                    governor_sha256=hashlib.sha256(
                        (ROOT / 'src/risk_governor.cpp').read_bytes()).hexdigest(),
                    replay_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
                    scope=('Recorded evidence, inferred velocity interval; '
                           'no internal Costmap replay.'))
    (HERE / (args.label + '_replay.json')).write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
