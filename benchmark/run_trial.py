#!/usr/bin/env python3
"""One Humble NavigateToPose trial. Run from source; no custom action client."""
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path

import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET


# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

PLUGINS = {'mydwa': 'my_dwa_controller::MyDWAController',
           'dwb': 'dwb_core::DWBLocalPlanner'}
LIFECYCLE = ('amcl', 'map_server', 'controller_server', 'planner_server',
             'smoother_server', 'behavior_server', 'bt_navigator',
             'waypoint_follower', 'velocity_smoother',
             'local_costmap/local_costmap', 'global_costmap/global_costmap')
FIELDS = ('sim_time_s wall_elapsed_s phase tf_stamp_s x y yaw '
          'odom_stamp_s odom_age_s odom_frame odom_child_frame actual_vx actual_vy actual_wz '
          'controller_cmd_receipt_sim_s controller_cmd_age_s '
          'controller_vx controller_vy controller_wz '
          'robot_cmd_receipt_sim_s robot_cmd_age_s robot_cmd_vx robot_cmd_vy robot_cmd_wz '
          'flags').split()


def pose_error(pose, target):
    return {'position_m': math.hypot(pose[0] - target[0], pose[1] - target[1]),
            'yaw_rad': abs(math.atan2(math.sin(pose[2] - target[2]),
                                      math.cos(pose[2] - target[2])))}


def start_matches(error, xy_tolerance, yaw_tolerance):
    return error['position_m'] <= xy_tolerance and error['yaw_rad'] <= yaw_tolerance


def fresh_value(value, stamp, now, max_age):
    # Gazebo publishes /clock at ~10 Hz, odom/TF at ~30 Hz. Their latest
    # source stamps legitimately lead the last received clock by up to a tick.
    return value if stamp is not None and -.15 <= now - stamp <= max_age else None


class ClockMonitor:
    def __init__(self):
        self.sim = None
        self.last_advance = None
        self.reset = False
        self.advances = 0

    def update(self, sim, wall):
        if self.sim is not None and sim < self.sim - 1e-6:
            self.reset = True
        if self.sim is None or sim > self.sim:
            self.last_advance = wall
            self.advances += 1
        self.sim = sim

    def problem(self, wall, stall_timeout):
        if self.reset:
            return 'clock_reset'
        if self.last_advance is None or wall - self.last_advance > stall_timeout:
            return 'clock_stalled'
        return None


class ActionWorker(threading.Thread):
    """Only this thread spins BasicNavigator; collector uses its own executor."""

    def __init__(self, navigator, goal):
        super().__init__(daemon=True)
        self.navigator, self.goal = navigator, goal
        self.cancel = threading.Event()
        self.done = threading.Event()
        self.accepted = False
        self.result = None
        self.status = None
        self.error = None
        self.feedback_samples = []

    def run(self):
        try:
            self.accepted = bool(self.navigator.goToPose(self.goal))
            if not self.accepted:
                self.result = 'REJECTED'
                return
            cancel_sent = False
            while not self.navigator.isTaskComplete():
                feedback = getattr(self.navigator, 'getFeedback', lambda: None)()
                if feedback is not None:
                    self.feedback_samples.append({
                        'navigation_time_s': stamp_seconds(feedback.navigation_time),
                        'distance_remaining_m': feedback.distance_remaining,
                        'number_of_recoveries': feedback.number_of_recoveries})
                if self.cancel.is_set() and not cancel_sent:
                    self.navigator.cancelTask()
                    cancel_sent = True
            self.result = self.navigator.getResult().name
            self.status = self.navigator.status
        except Exception as exc:  # surfaced in summary, never treated as success
            self.error = repr(exc)
        finally:
            self.done.set()


def watch_task(worker, tick, clock, timeout, cancel_timeout, stall_timeout, user_cancel):
    start = time.monotonic()
    reason, requested_at = None, None
    while True:
        tick()
        now = time.monotonic()
        if reason is None:
            problem = clock.problem(now, stall_timeout)
            if user_cancel.is_set():
                reason = 'user_cancel'
            elif problem:
                reason = problem
            elif now - start >= timeout and not worker.done.is_set():
                reason = 'wall_timeout'
            if reason:
                worker.cancel.set()
                requested_at = now
        if worker.done.is_set() or (requested_at is not None and
                                    now - requested_at >= cancel_timeout):
            break
    status = {'SUCCEEDED': 'succeeded', 'FAILED': 'failed', 'CANCELED': 'canceled',
              'REJECTED': 'rejected'}.get(worker.result, 'error')
    if reason:
        status = 'canceled' if reason == 'user_cancel' else (
            'invalid' if reason == 'clock_reset' else 'timeout')
    return {'status': status, 'stop_reason': reason,
            'action_result': worker.result, 'action_status_code': worker.status,
            'action_accepted': worker.accepted, 'action_error': worker.error,
            'cancellation_confirmed': worker.result == 'CANCELED',
            'worker_finished': worker.done.is_set(),
            'navigation_wall_s': time.monotonic() - start}


def cancel_after_error(worker, grace):
    """Keep the action executor alive while collector-independent cancellation runs."""
    worker.cancel.set()
    worker.done.wait(timeout=grace)
    return {'action_result': worker.result, 'action_status_code': worker.status,
            'action_accepted': worker.accepted, 'action_error': worker.error,
            'worker_finished': worker.done.is_set(),
            'cancellation_confirmed': worker.result == 'CANCELED'}


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def path_record(message, sim):
    points = [[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)]
              for p in message.poses]
    return {'receipt_sim_s': sim, 'header_stamp_s': stamp_seconds(message.header.stamp),
            'frame_id': message.header.frame_id, 'points': points,
            'geometry_sha256': hashlib.sha256(json.dumps(points).encode()).hexdigest()}


def gazebo_world_pose():
    """Diagnostic world pose, never used as clearance or as the map trajectory."""
    started = time.monotonic()
    try:
        raw = subprocess.check_output(['gz', 'model', '-m', 'turtlebot3_waffle', '-p'],
                                      timeout=3., stderr=subprocess.STDOUT).decode()
        values = list(map(float, raw.split()))
        if len(values) != 6:
            raise ValueError('unexpected gz model pose: ' + raw)
        return {'pose_xyz_rpy': values, 'query_wall_s': time.monotonic()-started,
                'source': 'Gazebo world via gz model -p'}
    except Exception as exc:
        return {'error': repr(exc), 'query_wall_s': time.monotonic()-started}


class Collector:
    def __init__(self, args, writer):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from rclpy.parameter import Parameter
        from tf2_ros import Buffer, TransformListener
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock

        self.rclpy, self.Time = rclpy, Time
        self.args, self.writer = args, writer
        self.node = Node('benchmark_collector')
        self.node.set_parameters([Parameter('use_sim_time', value=True)])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.buffer = Buffer(node=self.node)
        self.listener = TransformListener(self.buffer, self.node)
        self.clock = ClockMonitor()
        self.cache, self.events, self.counts = {}, {}, Counter()
        self.origin, self.next_sample = time.monotonic(), 0.
        self.recording, self.phase = False, 'navigation'
        self.rows = 0
        self.tf_last_stamp, self.tf_last_advance = None, None
        self.subs = [self.node.create_subscription(
            Clock, '/clock', lambda msg: self.clock.update(
                stamp_seconds(msg.clock), time.monotonic()), qos_profile_sensor_data)]
        self.subs.append(self.node.create_subscription(
            Odometry, args.odom_topic, self.odom_callback, qos_profile_sensor_data))
        for key, topic in [('controller', args.controller_cmd_topic),
                           ('robot', args.robot_cmd_topic)]:
            self.subs.append(self.node.create_subscription(
                Twist, topic, lambda msg, k=key: self.cache_command(k, msg),
                qos_profile_sensor_data))

    def start_navigation_recording(self, output):
        from action_msgs.msg import GoalStatusArray
        from nav_msgs.msg import Path as NavPath
        from nav2_msgs.msg import BehaviorTreeLog
        from rclpy.qos import qos_profile_action_status_default
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import QoSProfile, DurabilityPolicy
        names = ('plans', 'nav_events', 'bt_events', 'localization')
        self.navigation_streams = {name: (output / (name + '.jsonl')).open('w') for name in names}
        self.nav_event_cache = {}
        self.monitored_actions = ('spin', 'backup', 'drive_on_heading', 'wait',
                                  'follow_path', 'compute_path_to_pose')
        self.subs.append(self.node.create_subscription(
            NavPath, '/plan', self.plan_callback, 10))
        self.subs.append(self.node.create_subscription(
            BehaviorTreeLog, '/behavior_tree_log', self.bt_callback, 10))
        self.subs.append(self.node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        for action in self.monitored_actions:
            self.subs.append(self.node.create_subscription(
                GoalStatusArray, '/' + action + '/_action/status',
                lambda msg, name=action: self.action_status(name, msg),
                qos_profile_action_status_default))

    def plan_callback(self, message):
        if self.recording:
            self.navigation_streams['plans'].write(
                json.dumps(path_record(message, self.clock.sim)) + '\n')

    def action_status(self, action, message):
        if not self.recording:
            return
        for item in message.status_list:
            goal_id = bytes(item.goal_info.goal_id.uuid).hex()
            key = (action, goal_id)
            if self.nav_event_cache.get(key) == item.status:
                continue
            self.nav_event_cache[key] = item.status
            self.navigation_streams['nav_events'].write(json.dumps({
                'receipt_sim_s': self.clock.sim, 'action': action, 'goal_id': goal_id,
                'goal_stamp_s': stamp_seconds(item.goal_info.stamp),
                'status': item.status}) + '\n')

    def bt_callback(self, message):
        if self.recording:
            for event in message.event_log:
                self.navigation_streams['bt_events'].write(json.dumps({
                    'receipt_sim_s': self.clock.sim,
                    'bt_internal_stamp_s': stamp_seconds(event.timestamp),
                    'node': event.node_name, 'previous': event.previous_status,
                    'current': event.current_status}) + '\n')

    def event(self, code, detail=''):
        event = self.events.setdefault(code, {'count': 0, 'first_sim_s': self.clock.sim,
                                              'last_sim_s': None, 'detail': detail})
        event['count'] += 1
        event['last_sim_s'] = self.clock.sim

    def localization_record(self, source, header, pose, covariance):
        if self.recording and hasattr(self, 'navigation_streams'):
            self.navigation_streams['localization'].write(json.dumps({
                'source': source, 'receipt_sim_s': self.clock.sim,
                'source_stamp_s': stamp_seconds(header.stamp), 'frame_id': header.frame_id,
                'x': pose.position.x, 'y': pose.position.y, 'yaw': yaw(pose.orientation),
                'covariance': list(covariance)}) + '\n')

    def amcl_callback(self, message):
        self.localization_record('amcl', message.header,
                                 message.pose.pose, message.pose.covariance)

    def odom_callback(self, msg):
        self.cache['odom'] = (stamp_seconds(msg.header.stamp), time.monotonic(),
                              msg.twist.twist, msg.header.frame_id, msg.child_frame_id)
        self.localization_record('odom', msg.header, msg.pose.pose, msg.pose.covariance)

    def cache_command(self, key, msg):
        self.cache[key] = (self.clock.sim, time.monotonic(), msg)

    def pose(self):
        try:
            transform = self.buffer.lookup_transform('map', self.args.base_frame, self.Time())
            stamp = stamp_seconds(transform.header.stamp)
            wall = time.monotonic()
            if stamp != self.tf_last_stamp:
                self.tf_last_stamp, self.tf_last_advance = stamp, wall
            if wall - self.tf_last_advance > self.args.max_data_age:
                raise RuntimeError('map TF stamp stopped advancing in wall time')
            if self.clock.sim is None or fresh_value(True, stamp, self.clock.sim,
                                                     self.args.max_data_age) is None:
                raise RuntimeError(f'stale/future map TF at {stamp}, clock={self.clock.sim}')
            t = transform.transform
            return (t.translation.x, t.translation.y, yaw(t.rotation)), stamp
        except Exception as exc:
            if self.recording:
                self.event('tf_unavailable', str(exc))
            return None, None

    def tick(self):
        self.executor.spin_once(timeout_sec=.02)
        now = time.monotonic()
        if self.recording and now >= self.next_sample:
            self.sample()
            self.next_sample = now + 1. / self.args.sample_hz

    def sample(self):
        now, sim = time.monotonic(), self.clock.sim
        row = {'sim_time_s': sim, 'wall_elapsed_s': now - self.origin, 'phase': self.phase}
        flags = []
        pose, stamp = self.pose()
        row['tf_stamp_s'] = stamp
        if pose is None:
            flags.append('tf_unavailable')
        else:
            row.update(zip(('x', 'y', 'yaw'), pose))
            self.counts['pose'] += 1
        for key in ('odom', 'controller', 'robot'):
            item = self.cache.get(key)
            age = now - item[1] if item else None
            valid = (item is not None and sim is not None and age <= self.args.max_data_age
                     and fresh_value(True, item[0], sim, self.args.max_data_age) is not None)
            if key == 'odom':
                row.update(odom_stamp_s=item[0] if item else None, odom_age_s=age,
                           odom_frame=item[3] if item else None,
                           odom_child_frame=item[4] if item else None)
                columns = ('actual_vx', 'actual_vy', 'actual_wz')
            else:
                prefix = 'controller_cmd' if key == 'controller' else 'robot_cmd'
                row[prefix + '_receipt_sim_s'] = item[0] if item else None
                row[prefix + '_age_s'] = age
                columns = (('controller_vx', 'controller_vy', 'controller_wz')
                           if key == 'controller'
                           else ('robot_cmd_vx', 'robot_cmd_vy', 'robot_cmd_wz'))
            if valid:
                twist = item[2]
                row.update(zip(columns, (twist.linear.x, twist.linear.y, twist.angular.z)))
                self.counts[key] += 1
            else:
                code = key + '_missing_or_stale'
                flags.append(code)
                self.event(code)
        row['flags'] = ';'.join(flags)
        self.writer.writerow(row)
        self.rows += 1

    def wait(self, predicate, deadline, description, stop):
        while not predicate():
            if stop.is_set():
                raise RuntimeError('user canceled during preflight')
            if time.monotonic() >= deadline:
                raise TimeoutError('preflight timeout: ' + description)
            self.tick()

    def service(self, service_type, name, request, deadline, stop):
        """Bounded read-only queries; retry replies lost during DDS discovery."""
        client = self.node.create_client(service_type, name)
        try:
            self.wait(client.service_is_ready, deadline, name, stop)
            while time.monotonic() < deadline:
                future = client.call_async(request)
                response_deadline = min(deadline, time.monotonic() + 2.)
                try:
                    self.wait(future.done, response_deadline, name + ' response', stop)
                    return future.result()
                except TimeoutError:
                    future.cancel()
                    self.event('service_response_retry', name)
            raise TimeoutError('preflight timeout: ' + name + ' response')
        finally:
            self.node.destroy_client(client)

    def parameters(self, node, names, deadline, stop):
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        request = GetParameters.Request(names=names)
        values = self.service(GetParameters, '/' + node + '/get_parameters',
                              request, deadline, stop).values
        return dict(zip(names, (parameter_value_to_python(v) for v in values)))

    def all_parameters(self, node, deadline, stop):
        from rcl_interfaces.srv import ListParameters
        result = self.service(ListParameters, '/' + node + '/list_parameters',
                              ListParameters.Request(), deadline, stop)
        return self.parameters(node, result.result.names, deadline, stop)

    def endpoints(self, topic, publishers=True):
        method = (self.node.get_publishers_info_by_topic if publishers
                  else self.node.get_subscriptions_info_by_topic)
        return [{'node': (e.node_namespace.rstrip('/') + '/' + e.node_name),
                 'type': e.topic_type, 'reliability': e.qos_profile.reliability.name,
                 'durability': e.qos_profile.durability.name} for e in method(topic)]


def save_config(path, output, label, summary):
    path = Path(path).resolve()
    content = path.read_bytes()
    destination = output / 'configs' / (label + '_' + path.name)
    destination.parent.mkdir(exist_ok=True)
    destination.write_bytes(content)
    summary.setdefault('config_files', []).append({
        'source': str(path), 'saved': str(destination.relative_to(output)),
        'sha256': hashlib.sha256(content).hexdigest()})


def preflight(c, nav, args, stop, summary, output):
    from lifecycle_msgs.srv import GetState
    from ament_index_python.packages import get_package_share_directory
    deadline = time.monotonic() + args.ready_timeout
    for node in LIFECYCLE:
        def active(n=node):
            return c.service(GetState, '/' + n + '/get_state', GetState.Request(),
                             deadline, stop).current_state.id == 3
        c.wait(active, deadline, node + ' active', stop)
    runtime = summary['runtime_parameters'] = {}
    for node in LIFECYCLE + ('robot_state_publisher',):
        runtime[node] = c.parameters(node, ['use_sim_time'], deadline, stop)
        if runtime[node]['use_sim_time'] is not True:
            raise RuntimeError(node + ': use_sim_time must be true')
    runtime['controller_server'] = c.all_parameters('controller_server', deadline, stop)
    runtime['velocity_smoother'] = c.all_parameters('velocity_smoother', deadline, stop)
    bt_names = ['default_nav_to_pose_bt_xml']
    runtime['bt_navigator'].update(c.parameters('bt_navigator', bt_names, deadline, stop))
    runtime['map_server'].update(c.parameters('map_server', ['yaml_filename'], deadline, stop))
    control = runtime['controller_server']
    if control.get('controller_plugins') != ['FollowPath']:
        raise RuntimeError('minimal trial requires exactly one controller: FollowPath')
    actual = control.get('FollowPath.plugin')
    summary['actual_controller_plugin'] = actual
    if actual != PLUGINS[args.controller]:
        raise RuntimeError(
            f'controller mismatch: expected {PLUGINS[args.controller]}, got {actual}')
    bt = runtime['bt_navigator']['default_nav_to_pose_bt_xml'] or str(
        Path(get_package_share_directory('nav2_bt_navigator')) /
        'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml')
    tree = ET.parse(bt)
    bindings = [element.get('controller_id') for element in tree.iter('FollowPath')]
    if not bindings or any(binding != 'FollowPath' for binding in bindings):
        raise RuntimeError('BT must explicitly bind FollowPath; unsupported BT: ' + str(bt))
    summary['bt_controller_bindings'] = bindings
    bringup = Path(get_package_share_directory('nav2_bringup'))
    for label, path in [('params', args.params_file), ('bt', bt),
                        ('map', runtime['map_server']['yaml_filename']),
                        ('launch', bringup / 'launch/tb3_simulation_launch.py'),
                        ('navigation', bringup / 'launch/navigation_launch.py'),
                        ('startup', args.startup_script), ('runner', __file__)]:
        save_config(path, output, label, summary)
    c.wait(lambda: c.clock.advances >= 2 and c.pose()[0] is not None and 'odom' in c.cache,
           deadline, 'advancing /clock, map TF and odometry', stop)
    if c.clock.problem(time.monotonic(), args.clock_stall_timeout):
        raise RuntimeError('simulation clock is stalled or reset')
    c.wait(nav.nav_to_pose_client.server_is_ready, deadline, 'NavigateToPose action server', stop)
    graph = summary['topic_endpoints'] = {}
    for topic in ('/clock', '/scan', args.odom_topic,
                  args.controller_cmd_topic, args.robot_cmd_topic):
        graph[topic] = {'publishers': c.endpoints(topic), 'subscribers': c.endpoints(topic, False)}
        if not graph[topic]['publishers']:
            raise RuntimeError('no publisher: ' + topic)
    for topic, node, side in [(args.controller_cmd_topic, '/controller_server', 'publishers'),
                              (args.controller_cmd_topic, '/velocity_smoother', 'subscribers'),
                              (args.robot_cmd_topic, '/velocity_smoother', 'publishers')]:
        endpoints = graph[topic][side]
        if not any(e['node'] == node and e['type'] == 'geometry_msgs/msg/Twist'
                   for e in endpoints):
            raise RuntimeError(f'wrong command topic: {node} not in {topic} {side}')
    if args.record_navigation:
        for node in LIFECYCLE + ('robot_state_publisher',):
            runtime[node] = c.all_parameters(node, deadline, stop)
        for action in c.monitored_actions:
            topic = '/' + action + '/_action/status'
            c.wait(lambda t=topic: bool(c.endpoints(t)), deadline, topic + ' publisher', stop)
            graph[topic] = {'publishers': c.endpoints(topic)}
        c.wait(lambda: bool(c.endpoints('/plan')), deadline, '/plan publisher', stop)
        summary['event_monitor_ready'] = True
        summary['bt_log_publisher_present'] = bool(c.endpoints('/behavior_tree_log'))
        import yaml
        map_yaml = Path(runtime['map_server']['yaml_filename'])
        map_image = yaml.safe_load(map_yaml.read_text())['image']
        for label, path in [('map_image', map_yaml.parent / map_image),
                            ('world', args.world_file or bringup / 'worlds/world_only.model'),
                            ('robot', bringup / 'worlds/waffle.model'),
                            ('scenarios', Path(__file__).with_name('scenarios.yaml'))]:
            save_config(path, output, label, summary)
    odom = c.cache['odom']
    if (odom[4] not in ('base_link', 'base_footprint') or
            fresh_value(True, odom[0], c.clock.sim, args.max_data_age) is None):
        raise RuntimeError('missing/future/stale odometry or unsupported odom child frame')
    pose, _ = c.pose()
    summary['observed_start'] = pose
    error = summary['start_error'] = pose_error(pose, args.start)
    if not start_matches(error, args.start_xy_tolerance, args.start_yaw_tolerance):
        summary['status'] = 'invalid_start'
        raise RuntimeError('robot is not at the prescribed start: ' + json.dumps(error))
    summary['start_valid'] = True


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', type=float, nargs=3, default=[-1., -.5, 0.],
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--start', type=float, nargs=3, default=[-2., -.5, 0.],
                        metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--controller', choices=PLUGINS, default='mydwa')
    parser.add_argument('--params-file', type=Path)
    parser.add_argument('--startup-script', type=Path,
                        default=(ARCHIVE_WS / 'scripts/start_headless_baseline.sh'))
    parser.add_argument('--results-dir', type=Path,
                        default=Path(__file__).resolve().parent / 'results')
    parser.add_argument('--trial-id',
                        default=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    parser.add_argument('--timeout', type=float, default=120., help='navigation wall seconds')
    parser.add_argument('--ready-timeout', type=float, default=60.,
                        help='total preflight wall seconds')
    parser.add_argument('--cancel-timeout', type=float, default=5.,
                        help='cancellation grace wall seconds')
    parser.add_argument('--clock-stall-timeout', type=float, default=3.)
    parser.add_argument('--max-data-age', type=float, default=.5,
                        help='seconds, sim AND wall freshness')
    parser.add_argument('--sample-hz', type=float, default=10.)
    parser.add_argument('--start-xy-tolerance', type=float, default=.3)
    parser.add_argument('--start-yaw-tolerance', type=float, default=.3)
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--controller-cmd-topic', default='/cmd_vel_nav')
    parser.add_argument('--robot-cmd-topic', default='/cmd_vel')
    parser.add_argument('--record-navigation', action='store_true',
                        help='save global plans, BT/actions, feedback and full runtime parameters')
    parser.add_argument('--world-file', type=Path, help='actual launched world, for provenance')
    args = parser.parse_args()
    if Path(args.trial_id).name != args.trial_id or args.trial_id in ('.', '..'):
        parser.error('--trial-id must be a single directory name')
    for key, value in vars(args).items():
        if isinstance(value, float) and (not math.isfinite(value) or value <= 0.):
            parser.error(key + ' must be finite and positive')
    if not all(math.isfinite(v) for v in args.goal + args.start):
        parser.error('start and goal must be finite')
    if args.params_file is None:
        suffix = '' if args.controller == 'mydwa' else '_dwb'
        args.params_file = (Path(__file__).resolve().parent.parent / 'config' /
                            ('nav2_params' + suffix + '.yaml'))
    return args


def main():
    args = arguments()
    output = args.results_dir.resolve() / args.trial_id
    output.mkdir(parents=True, exist_ok=False)
    summary = {'schema_version': 1, 'trial_id': args.trial_id, 'status': 'preflight_failed',
               'created_utc': datetime.now(timezone.utc).isoformat(),
               'arguments': {k: str(v) if isinstance(v, Path) else v
                             for k, v in vars(args).items()},
               'command': sys.argv, 'expected_controller_plugin': PLUGINS[args.controller],
               'pose_frame': 'map', 'start_valid': False, 'valid': False,
               'environment': {k: os.environ.get(k) for k in
                               ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
                                'TURTLEBOT3_MODEL')}}
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    c, nav, worker, initialized = None, None, None, False
    with (output / 'trajectory.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, FIELDS)
        writer.writeheader()
        try:
            import rclpy
            from rclpy.signals import SignalHandlerOptions
            from rclpy.parameter import Parameter
            from nav2_simple_commander.robot_navigator import BasicNavigator
            from geometry_msgs.msg import PoseStamped
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            initialized = True
            c = Collector(args, writer)
            if args.record_navigation:
                c.start_navigation_recording(output)
            nav = BasicNavigator(node_name='benchmark_navigator')
            nav.set_parameters([Parameter('use_sim_time', value=True)])
            preflight(c, nav, args, stop, summary, output)
            if args.record_navigation:
                summary['gazebo_world_start'] = gazebo_world_pose()
                world_pose = summary['gazebo_world_start'].get('pose_xyz_rpy')
                if world_pose and math.hypot(world_pose[0]-args.start[0],
                                             world_pose[1]-args.start[1]) > .15:
                    summary.update(status='invalid_start', start_valid=False)
                    raise RuntimeError('Gazebo physical position does not match prescribed start')
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = c.node.get_clock().now().to_msg()
            goal.pose.position.x, goal.pose.position.y = args.goal[:2]
            goal.pose.orientation.z = math.sin(args.goal[2] / 2.)
            goal.pose.orientation.w = math.cos(args.goal[2] / 2.)
            c.recording = True
            c.origin = time.monotonic()
            c.sample()
            sim_start = c.clock.sim
            summary['navigation_start_sim_s'] = sim_start
            print(f'Trial {args.trial_id}: {summary["actual_controller_plugin"]}, '
                  f'goal={args.goal}', flush=True)
            worker = ActionWorker(nav, goal)
            worker.start()
            summary.update(watch_task(worker, c.tick, c.clock, args.timeout, args.cancel_timeout,
                                      args.clock_stall_timeout, stop))
            c.sample()
            summary['navigation_end_sim_s'] = c.clock.sim
            summary['navigation_sim_s'] = c.clock.sim - sim_start
            pose, _ = c.pose()
            summary['final_pose'] = pose
            summary['final_error'] = pose_error(pose, args.goal) if pose is not None else None
            if args.record_navigation:
                summary['gazebo_world_end'] = gazebo_world_pose()
                world_pose = summary['gazebo_world_end'].get('pose_xyz_rpy')
                if world_pose and pose is not None:
                    summary['end_map_world_disagreement_m'] = math.hypot(
                        pose[0]-world_pose[0], pose[1]-world_pose[1])
            if summary['stop_reason']:
                c.event(summary['stop_reason'])
            if not worker.done.is_set():
                c.event('cancellation_unconfirmed',
                        'worker still blocked; check Nav2 before next run')
            summary['valid'] = (summary['start_valid'] and worker.done.is_set() and
                                pose is not None and all(
                                    c.counts[k] > 0 for k in
                                    ('pose', 'odom', 'controller', 'robot')) and
                                summary['stop_reason'] not in ('clock_reset', 'clock_stalled'))
        except Exception as exc:
            summary['error'] = repr(exc)
            if worker is not None:
                summary.update(cancel_after_error(worker, args.cancel_timeout))
                summary['valid'] = False
                summary['status'] = 'error'
                if not worker.done.is_set() and c is not None:
                    c.event('cancellation_unconfirmed', 'error cleanup grace expired')
            print(str(exc), file=sys.stderr, flush=True)
        finally:
            if worker is not None and args.record_navigation:
                feedback = worker.feedback_samples
                summary['max_feedback_recoveries'] = max(
                    (f['number_of_recoveries'] for f in feedback), default=None)
                (output / 'feedback.jsonl').write_text(''.join(
                    json.dumps(f) + '\n' for f in feedback))
            if c is not None:
                summary.update(samples=c.rows, valid_samples=dict(c.counts), anomalies=c.events,
                               clock_reset_detected=c.clock.reset)
                for sidecar in getattr(c, 'navigation_streams', {}).values():
                    sidecar.close()
            temporary = output / 'summary.json.tmp'
            temporary.write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
            temporary.replace(output / 'summary.json')
            stream.flush()
            print(f'{summary["status"]}: {output}', flush=True)
            if c is not None:
                c.executor.shutdown(timeout_sec=.5)
                c.node.destroy_node()
            if initialized:
                rclpy.try_shutdown()
            if worker is not None:
                worker.join(timeout=.5)
            if nav is not None and (worker is None or not worker.is_alive()):
                nav.destroy_node()
    return 0 if summary['status'] == 'succeeded' and summary['valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
