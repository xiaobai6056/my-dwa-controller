#!/usr/bin/env python3
"""Real Nav2 dynamic trial, reusing static preflight, collector and action client."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import os

import signal
import sys
import threading
import time


# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import run_trial as baseline  # noqa: E402
sys.path.insert(0, str(HERE))
from scenario import load  # noqa: E402
from motion import Motion  # noqa: E402
from observer import Observer  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=['D01', 'D02'], required=True)
    parser.add_argument(
        '--controller',
        choices=baseline.PLUGINS,
        required=True)
    parser.add_argument('--output', type=Path, required=True)
    parsed = parser.parse_args()
    scene = load(parsed.scene)
    output = parsed.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    # Baseline arguments and validations are reused without changing their
    # defaults.
    original = sys.argv
    sys.argv = [
        str(HERE.parent / 'run_trial.py'),
        '--controller', parsed.controller, '--start', *
        map(str, scene['start']),
        '--goal', *map(str, scene['goal']),
        '--timeout', str(scene['timeout']),
        '--ready-timeout', '100', '--record-navigation', '--world-file',
        str(HERE / 'assets' / parsed.scene / 'world.sdf'),
        '--startup-script', str(HERE / 'start_scene.sh')]
    args = baseline.arguments()
    sys.argv = original
    summary = dict(
        schema_version='dynamic_v0.1', scene=parsed.scene,
        controller=parsed.controller, status='preflight_failed', valid=False,
        start_valid=False,
        arguments={k: str(v) if isinstance(v, Path) else v for k,
                   v in vars(args).items()},
        command=original,
        timing='observed /clock at dispatch and action terminal',
        expected_controller_plugin=baseline.PLUGINS[parsed.controller])
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    c = observer = worker = nav = None
    initialized = False
    with (output / 'trajectory.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, baseline.FIELDS)
        writer.writeheader()
        try:
            import rclpy
            from rclpy.parameter import Parameter
            from rclpy.signals import SignalHandlerOptions
            from geometry_msgs.msg import PoseStamped, Twist
            from nav2_simple_commander.robot_navigator import BasicNavigator
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            initialized = True
            c = baseline.Collector(args, writer)
            c.start_navigation_recording(output)
            observer = Observer(c, scene, output)
            c.motion = Motion(c, scene, observer.emit)
            c.subs.append(
                c.node.create_subscription(
                    Twist,
                    '/cmd_vel',
                    observer.command,
                    100))
            nav = BasicNavigator(node_name='dynamic_benchmark_navigator')
            nav.set_parameters([Parameter('use_sim_time', value=True)])
            baseline.preflight(c, nav, args, stop, summary, output)
            c.wait(lambda: observer.robot is not None and c.motion.client.service_is_ready()
                   and observer.contact_callbacks > 0,
                   time.monotonic() + 15, 'ground truth / obstacle service / contacts', stop)
            physical_error = baseline.pose_error(
                observer.robot, scene['start'])
            summary['physical_start_error'] = physical_error
            if not baseline.start_matches(physical_error, .15, .3):
                raise RuntimeError('physical start mismatch')
            for topic in ('/benchmark/model_states', '/benchmark/contacts',
                          '/local_costmap/costmap'):
                summary['topic_endpoints'][topic] = dict(
                    publishers=c.endpoints(topic),
                    subscribers=c.endpoints(topic, False))
            subscribers = summary['topic_endpoints'][
                '/benchmark/model_states']['subscribers']
            if any(e['node'] != '/benchmark_collector' for e in subscribers):
                raise RuntimeError(
                    'unexpected ground truth subscriber: ' +
                    repr(subscribers))
            for source in sorted(
                    HERE.glob('*.py')) + [HERE / 'scenarios.json']:
                baseline.save_config(source, output, 'dynamic', summary)
            plugin = Path(
                str(ARCHIVE_WS / 'install/my_dwa_controller/lib/libmy_dwa_controller.so'))
            summary['mydwa_library_sha256'] = hashlib.sha256(
                plugin.read_bytes()).hexdigest()
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = c.node.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = scene['goal'][:2]
            goal.pose.orientation.z = math.sin(scene['goal'][2] / 2)
            goal.pose.orientation.w = math.cos(scene['goal'][2] / 2)
            c.recording = True
            c.origin = time.monotonic()
            start = c.clock.sim
            summary['navigation_start_sim_s'] = start
            c.motion.begin(start)
            worker = baseline.ActionWorker(nav, goal)
            worker.start()
            print(
                f'{parsed.scene} {parsed.controller}: dispatch sim={start}',
                flush=True)
            reason, cancel_at = None, None
            while True:
                c.tick()
                c.motion.tick()
                now = time.monotonic()
                if reason is None:
                    if observer.contacts:
                        reason = 'collision'
                    elif c.clock.problem(now, args.clock_stall_timeout):
                        reason = c.clock.problem(now, args.clock_stall_timeout)
                    elif c.motion.error:
                        reason = 'motion_error'
                    elif observer.last_gt_wall is None or now - observer.last_gt_wall > 2.:
                        reason = 'ground_truth_stalled'
                    elif stop.is_set():
                        reason = 'user_cancel'
                    elif c.clock.sim - start >= scene['timeout'] and not worker.done.is_set():
                        reason = 'sim_timeout'
                    elif now - c.origin > 3 * scene['timeout'] + 30:
                        reason = 'wall_watchdog'
                    if reason:
                        worker.cancel.set()
                        cancel_at = now
                if worker.done.is_set() or (cancel_at and now - cancel_at > args.cancel_timeout):
                    break
            summary.update(
                action_result=worker.result, action_status_code=worker.status,
                action_accepted=worker.accepted, action_error=worker.error,
                stop_reason=reason, navigation_end_sim_s=c.clock.sim,
                navigation_wall_s=time.monotonic() - c.origin,
                status='finished', motion_error=c.motion.error,
                valid=bool(
                    summary['start_valid']
                    and worker.done.is_set() and worker.error is None and observer.gt_count > 0
                    and observer.contact_callbacks > 0
                    and
                    reason
                    not
                    in (
                        'clock_stalled', 'clock_reset', 'motion_error',
                        'ground_truth_stalled', 'user_cancel',
                        'wall_watchdog')))
            summary['max_feedback_recoveries'] = max(
                (f['number_of_recoveries'] for f in worker.feedback_samples), default=None)
            (output / 'feedback.jsonl').write_text(''.join(json.dumps(f) + '\n'
                                                           for f in worker.feedback_samples))
        except Exception as exc:
            summary.update(error=repr(exc), valid=False)
            if worker is not None:
                summary.update(baseline.cancel_after_error(worker, 5.))
            print(repr(exc), file=sys.stderr, flush=True)
        finally:
            if c is not None:
                summary.update(samples=c.rows, anomalies=c.events,
                               clock_reset_detected=c.clock.reset)
                if hasattr(c, 'motion'):
                    summary['motion_service'] = dict(
                        sent=c.motion.sent,
                        acknowledged=c.motion.acknowledged,
                        error=c.motion.error)
                for sidecar in getattr(c, 'navigation_streams', {}).values():
                    sidecar.close()
            if observer is not None:
                summary['observation_counts'] = dict(
                    gt=observer.gt_count,
                    contact_callbacks=observer.contact_callbacks,
                    robot_contacts=observer.contacts,
                    scans=observer.scan_count,
                    scan_hits=observer.scan_hits,
                    costmaps=observer.cost_count,
                    costmap_hits=observer.cost_hits)
                observer.close()
            (output / 'summary.json').write_text(json.dumps(summary,
                                                            indent=2, allow_nan=False) + '\n')
            stream.flush()
            if c is not None:
                c.executor.shutdown(timeout_sec=.5)
                c.node.destroy_node()
            if initialized:
                rclpy.try_shutdown()
            if worker is not None:
                worker.join(timeout=.5)
            if nav is not None and (worker is None or not worker.is_alive()):
                nav.destroy_node()
    print(json.dumps({k: summary.get(k) for k in (
        'status', 'valid', 'action_result', 'stop_reason', 'error')}), flush=True)
    return 0 if summary.get('valid') and summary.get(
        'action_result') == 'SUCCEEDED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
