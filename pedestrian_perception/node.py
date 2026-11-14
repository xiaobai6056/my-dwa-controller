"""ROS adapter for SORT-guided LiDAR pedestrian measurements."""

from bisect import bisect_left, bisect_right, insort
from collections import deque
from dataclasses import asdict, dataclass
import json
from math import isclose, isfinite
from pathlib import Path
from time import monotonic_ns, perf_counter_ns

from ament_index_python.packages import get_package_share_directory
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg, Time as TimeMsg
from geometry_msgs.msg import Point, Vector3
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import ColorRGBA, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker

from my_dwa_controller.msg import PedestrianState, PedestrianStateArray

from .association import (CIRCLE_RADIUS_M, CLUSTER_DISTANCE_M, MIN_POINTS, POSITION_ESTIMATORS,
                          associate,
                          estimate_positions)
from .geometry import Calibration, pose, project, read_scan, read_tracks, stamp_ns
from .kalman import FilterConfig, KalmanHistory
from .state import SECOND, StateHistory

CLOCK_LEAD_NS = 500_000_000
CACHE_NS = 4 * SECOND
MAX_PENDING = 32


def ros_time(timestamp, kind=TimeMsg):
    """Build a ROS Time or Duration from integral nanoseconds."""
    sec, nanosec = divmod(timestamp, SECOND)
    return kind(sec=sec, nanosec=nanosec)


class SensorTransforms:
    """Resolve fixed mounts and bound interpolation of the audited encoder edge."""

    def __init__(self, base, max_gap_ns):
        self.base = base
        self.max_gap_ns = max_gap_ns
        self.static = Buffer()
        self.reset()

    def reset(self):
        """Discard dynamic poses from a previous clock epoch."""
        self.dynamic = Buffer(cache_time=Duration(nanoseconds=CACHE_NS))
        self.stamps = []

    def add_static(self, msg):
        """Accept fixed mount transforms from the transient-local TF topic."""
        for transform in msg.transforms:
            self.static.set_transform_static(transform, 'tf_static')

    def add_dynamic(self, transform, timestamp):
        """Store one encoder TF with a bounded source-time index."""
        if self.stamps and timestamp < self.stamps[-1] - CACHE_NS:
            return
        index = bisect_left(self.stamps, timestamp)
        if index < len(self.stamps) and self.stamps[index] == timestamp:
            return
        t, q = transform.transform.translation, transform.transform.rotation
        if (not all(map(isfinite, (t.x, t.y, t.z, q.x, q.y, q.z, q.w)))
                or not isclose(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1., abs_tol=.001)):
            return
        self.dynamic.set_transform(transform, 'odom_tf')
        self.stamps.insert(index, timestamp)
        del self.stamps[:bisect_left(self.stamps, self.stamps[-1] - CACHE_NS)]

    def at(self, frame, timestamp):
        """Return odom-to-sensor pose only at an exact or tightly bracketed time."""
        index = bisect_left(self.stamps, timestamp)
        if index == len(self.stamps):
            return None
        if self.stamps[index] != timestamp and (
                index == 0 or self.stamps[index] - self.stamps[index - 1] > self.max_gap_ns):
            return None
        # All timestamps used here are positive. Time() would ask for latest.
        when = Time(nanoseconds=timestamp, clock_type=ClockType.ROS_TIME)
        try:
            base_origin, base_rotation = pose(
                self.dynamic.lookup_transform('odom', self.base, when).transform)
            if frame == self.base:
                return base_origin, base_rotation
            mount_origin, mount_rotation = pose(
                self.static.lookup_transform(self.base, frame, when).transform)
            return base_origin + base_rotation.apply(mount_origin), base_rotation * mount_rotation
        except (TransformException, ValueError):
            return None


@dataclass
class Pending:
    """One scan waiting for its source-time dependencies."""

    stamp: int
    frame: str
    received: int
    rays: list | None
    projected: list | None = None
    projected_image: int = 0
    projected_calibration: Calibration | None = None
    processing_ns: int = 0


class PedestrianNode(Node):
    """Publish independent typed pedestrian states for downstream consumers."""

    def __init__(self):
        super().__init__('pedestrian_fusion')
        self.base = self.declare_parameter('base_frame', 'base_footprint').value
        self.laser = self.declare_parameter('laser_frame', 'base_scan').value
        self.optical = self.declare_parameter('optical_frame', 'camera_rgb_optical_frame').value
        self.cadence = self.duration_parameter('scan_period_s', .2)
        self.pair_limit = self.duration_parameter('pair_tolerance_s', .05)
        self.sort_wait = self.duration_parameter('sort_pair_wait_s', .110)
        self.wait = self.duration_parameter('dependency_wait_s', .5)
        self.depth_near = self.declare_parameter('camera_near_m', .02).value
        self.depth_far = self.declare_parameter('camera_far_m', 30.).value
        self.distance = self.declare_parameter('cluster_distance_m', CLUSTER_DISTANCE_M).value
        self.min_points = self.declare_parameter('min_cluster_points', MIN_POINTS).value
        self.position_estimator = self.declare_parameter(
            'position_estimator', 'trimmed_centroid', ParameterDescriptor(read_only=True)).value
        if self.position_estimator not in POSITION_ESTIMATORS:
            raise ValueError(f'unknown position_estimator: {self.position_estimator}')
        self.get_logger().info(f'Position estimator: {self.position_estimator}')
        if (not all(map(isfinite, (self.depth_near, self.depth_far, self.distance)))
                or not 0 < self.depth_near < self.depth_far or self.distance <= 0
                or self.min_points < 1):
            raise ValueError('invalid camera clip or clustering parameters')
        max_age = self.duration_parameter('max_observation_age_s', .5)
        retention = self.duration_parameter('state_retention_s', 5.)
        self.states = StateHistory(self.cadence, max_age, retention)
        method = self.declare_parameter(
            'state_estimator', 'kf', ParameterDescriptor(read_only=True)).value
        if method not in ('legacy', 'kf'):
            raise ValueError(f'unknown state_estimator: {method}')
        self.get_logger().info(f'State estimator: {method}')
        self.kalman = method == 'kf'
        if self.kalman:
            installed = Path(get_package_share_directory('my_dwa_controller'))
            default_config = installed / 'config' / 'kf_frozen.json'
            path = self.declare_parameter('kf_config', str(default_config)).value
            with open(path, encoding='utf-8') as stream:
                config = FilterConfig(**json.load(stream)[self.position_estimator])
            self.states = KalmanHistory(self.states, config)
        self.transforms = SensorTransforms(self.base, self.duration_parameter('tf_max_gap_s', .07))
        self.publisher = self.create_publisher(PedestrianStateArray, '/my_dwa/pedestrians', 10)
        self.cluster_pub = None
        if self.declare_parameter('visualize_clusters', False).value:
            self.cluster_pub = self.create_publisher(Marker, '/my_dwa/demo/clusters', 1)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/sort_prototype/tracks', self.on_sort, 50)
        self.create_subscription(CameraInfo, '/visual_prototype/camera/camera_info',
                                 self.on_camera, qos_profile_sensor_data)
        self.create_subscription(String, '/state_compare/vision', self.on_ipm, 50)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(
            TFMessage, '/tf_static', self.transforms.add_static,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.epoch = 0
        self.reset_until = 0
        self.pending = deque()
        self.sort_frames = {}
        self.sort_times = []
        self.calibration_times = []
        self.calibrations = {}
        self.ipm = {}
        self.last_scan = 0
        self.last_gap_output = 0
        path = self.declare_parameter('diagnostic_file', '').value
        self.log = open(path, 'x', encoding='utf-8', buffering=1) if path else None
        self.retry_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.01, self.drain, clock=self.retry_clock)
        self.jump = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=False), post_callback=self.on_clock_jump)

    def duration_parameter(self, name, seconds):
        """Read a positive finite ROS parameter expressed in seconds."""
        value = self.declare_parameter(name, seconds).value
        if not isfinite(value) or value < 1 / SECOND:
            raise ValueError(f'{name} must be finite and positive')
        return round(value * SECOND)

    def destroy_node(self):
        """Close optional diagnostics with the ROS node."""
        if self.log is not None:
            self.log.close()
        return super().destroy_node()

    def on_clock_jump(self, _):
        """Clear source-time caches and invalidate tracks after a backward jump."""
        self.epoch += 1
        self.reset_until = self.get_clock().now().nanoseconds + CLOCK_LEAD_NS
        self.pending.clear()
        self.sort_times.clear()
        self.sort_frames.clear()
        self.calibration_times.clear()
        self.calibrations.clear()
        self.ipm.clear()
        self.states.reset()
        self.transforms.reset()
        self.last_scan = self.last_gap_output = 0
        self.publish(None, 0, 'clock_reset', [])

    def plausible_stamp(self, timestamp):
        """Reject zero and stamps from the wrong clock domain or clock epoch."""
        now = self.get_clock().now().nanoseconds
        lead = 0 if now < self.reset_until else CLOCK_LEAD_NS
        return now > 0 and 0 < timestamp <= now + lead

    def on_tf(self, message):
        """Read only the configured odom-to-base encoder transform."""
        for transform in message.transforms:
            if (transform.header.frame_id, transform.child_frame_id) != ('odom', self.base):
                continue
            timestamp = stamp_ns(transform.header.stamp)
            if self.plausible_stamp(timestamp):
                self.transforms.add_dynamic(transform, timestamp)

    def on_camera(self, message):
        """Keep camera calibrations indexed by their source timestamp."""
        timestamp = stamp_ns(message.header.stamp)
        if message.header.frame_id != self.optical or not self.plausible_stamp(timestamp):
            return
        calibration = Calibration.from_message(message)
        if timestamp not in self.calibrations:
            insort(self.calibration_times, timestamp)
        elif calibration != self.calibrations[timestamp]:
            calibration = None
        self.calibrations[timestamp] = calibration
        # Retain one older entry for a fixed camera publishing CameraInfo once.
        cutoff = self.calibration_times[-1] - CACHE_NS
        keep_from = max(0, bisect_left(self.calibration_times, cutoff) - 1)
        for old in self.calibration_times[:keep_from]:
            del self.calibrations[old]
        del self.calibration_times[:keep_from]

    def on_sort(self, message):
        """Cache source-stamped SORT batches, including empty detections."""
        timestamp = stamp_ns(message.header.stamp)
        if not self.plausible_stamp(timestamp):
            return
        if self.sort_times and timestamp < self.sort_times[-1] - CACHE_NS:
            return
        tracks = read_tracks(message) if message.header.frame_id == self.optical else None
        if timestamp not in self.sort_frames:
            insort(self.sort_times, timestamp)
        elif tracks != self.sort_frames[timestamp]:
            tracks = None
        self.sort_frames[timestamp] = tracks
        cutoff = self.sort_times[-1] - CACHE_NS
        expired = bisect_left(self.sort_times, cutoff)
        for old in self.sort_times[:expired]:
            del self.sort_frames[old]
        del self.sort_times[:expired]

    def on_ipm(self, message):
        """Read valid measured IPM positions from the existing same-image batch."""
        try:
            row = json.loads(message.data)
            stamp = row['stamp_ns']
            if (type(stamp) is not int or not self.plausible_stamp(stamp)
                    or row['epoch'] != self.epoch or row['frame_id'] != 'odom'
                    or row['source_frame'] != self.optical):
                return
            positions = {}
            for item in row['objects']:
                if item['position_valid'] is not True:
                    continue
                key, xy, box = item['id'], item['position'], item['bbox']
                if (not isinstance(key, str) or not key or key in positions
                        or len(xy) != 2 or len(box) != 4 or not all(map(isfinite, (*xy, *box)))):
                    return
                positions[key] = tuple(xy), tuple(box)
        except (KeyError, TypeError, ValueError):
            return
        self.ipm[stamp] = positions if self.ipm.get(stamp, positions) == positions else {}
        cutoff = self.get_clock().now().nanoseconds - CACHE_NS
        for old in [key for key in self.ipm if key < cutoff]:
            del self.ipm[old]

    def on_scan(self, message):
        """Queue scans by source order without running inference or blocking TF."""
        timestamp = stamp_ns(message.header.stamp)
        if not self.plausible_stamp(timestamp) or timestamp <= self.last_scan:
            return
        self.last_scan = timestamp
        if len(self.pending) == MAX_PENDING:
            self.finish(self.pending.popleft(), 0, [], 'queue_overflow')
        self.pending.append(Pending(timestamp, message.header.frame_id, monotonic_ns(),
                                    read_scan(message, timestamp)))

    def pair(self, stamp, expired):
        """Choose R2's nearest SORT source stamp, preferring the earlier tie."""
        index = bisect_left(self.sort_times, stamp)
        if not expired and index == len(self.sort_times):
            return 0
        neighbors = self.sort_times[max(0, index - 1):index + 1]
        nearest = min(neighbors, key=lambda t: (abs(t - stamp), t), default=0)
        return nearest if abs(nearest - stamp) <= self.pair_limit else 0

    def project_scan(self, entry, image, tracks, calibration):
        """Compute R2 projection, or return the missing time-specific dependency."""
        if calibration is None:
            return None, 'camera_info_unavailable'
        stamps = {entry.stamp, *(stamp for _, stamp, _ in entry.rays)}
        poses = {stamp: self.transforms.at(self.laser, stamp) for stamp in stamps}
        if any(value is None for value in poses.values()):
            return None, 'scan_tf_unavailable'
        optical = self.transforms.at(self.optical, image)
        if optical is None:
            return None, 'camera_tf_unavailable'
        return project(entry.rays, poses, optical, calibration, tracks,
                       self.depth_near, self.depth_far), 'ok'

    def process(self, entry, sort_expired, expired, now):
        """Finalize a scan once paired and supported, or after the wait expires."""
        image = self.pair(entry.stamp, sort_expired or expired)
        tracks = self.sort_frames.get(image)
        if entry.frame != self.laser or entry.rays is None:
            return image, [], 'invalid_scan'
        if not image:
            return (0, [], 'missing_sort_pair') if sort_expired or expired else None
        last_ray = entry.rays[-1][1] if entry.rays else entry.stamp
        if max(image, last_ray) > now:
            return (image, [], 'source_time_ahead') if expired else None
        if tracks is None:
            return image, [], 'invalid_sort'
        index = bisect_right(self.calibration_times, image) - 1
        calibration = self.calibrations[self.calibration_times[index]] if index >= 0 else None
        if (entry.projected is None or entry.projected_image != image
                or entry.projected_calibration != calibration):
            entry.projected_image = image
            entry.projected_calibration = calibration
            entry.projected, reason = self.project_scan(entry, image, tracks, calibration)
            if entry.projected is None:
                return (image, [], reason) if expired else None
        if not expired and image not in self.ipm and any(
                len(beam.candidates) > 1 for beam in entry.projected):
            return None
        ipm = {key: xy for key, (xy, box) in self.ipm.get(image, {}).items()
               if key in tracks and all(abs(a - b) <= 1e-6 for a, b in zip(box, tracks[key]))}
        groups = associate(entry.projected, ipm, self.distance)
        return image, groups, 'ok' if tracks else 'empty_sort'

    def drain(self):
        """Process pending scans and publish invalid states during scan loss."""
        now = self.get_clock().now().nanoseconds
        while self.pending and self.pending[0].stamp <= now:
            entry = self.pending[0]
            age = monotonic_ns() - entry.received
            sort_expired = age >= self.sort_wait
            expired = age >= self.wait
            start = perf_counter_ns()
            result = self.process(entry, sort_expired, expired, now)
            entry.processing_ns += perf_counter_ns() - start
            if result is None:
                break
            self.pending.popleft()
            self.finish(entry, *result)
        if (not self.pending and self.last_scan and now - self.last_scan > 1.5 * self.cadence
                and now - self.last_gap_output >= self.cadence):
            self.last_gap_output = now
            # Source-time continuity is decided when the next scan is committed.
            self.publish(None, 0, 'scan_timeout', [])

    def finish(self, entry, image, groups, status):
        """Commit one scan and publish its state, including unresolved frames."""
        start = perf_counter_ns()
        positions = estimate_positions(
            entry.projected or [], groups, self.min_points, estimator=self.position_estimator)
        self.states.update(entry.stamp, positions)
        entry.processing_ns += perf_counter_ns() - start
        self.publish(entry, image, status, groups)

    def publish(self, entry, image, status, groups):
        """Publish the typed state and optionally record scan-level diagnostics."""
        now = self.get_clock().now().nanoseconds
        scan = entry.stamp if entry else self.states.last_scan_stamp
        rows = self.states.snapshot(
            now, invalidate=entry is None, hold_velocity=status in ('ok', 'empty_sort'))
        message = PedestrianStateArray(scan_stamp=ros_time(scan), image_stamp=ros_time(image),
                                       status=status, epoch=self.epoch,
                                       kalman_estimate=self.kalman)
        message.header.frame_id, message.header.stamp = 'odom', ros_time(now)
        for row in rows:
            age = row.observation_age_ns if row.observation_age_ns is not None else -SECOND
            message.states.append(PedestrianState(
                track_id=row.track_id, position=Point(x=row.position[0], y=row.position[1]),
                occupancy_radius=CIRCLE_RADIUS_M,
                velocity=Vector3(x=row.velocity[0], y=row.velocity[1]),
                direct_velocity=Vector3(x=row.direct[0], y=row.direct[1]),
                diagnostic_velocity_0_8s=Vector3(x=row.diagnostic[0], y=row.diagnostic[1]),
                position_valid=row.position_valid, velocity_valid=row.velocity_valid,
                direct_velocity_valid=row.direct_valid,
                diagnostic_velocity_valid=row.diagnostic_valid,
                last_observation_stamp=ros_time(row.last_observation_stamp),
                observation_age=ros_time(age, DurationMsg),
            ))
        self.publisher.publish(message)
        if self.cluster_pub is not None:
            self.publish_clusters(message.header, entry, groups)
        if self.log is not None:
            latency = (monotonic_ns() - entry.received) / 1e6 if entry else None
            logged_states = [{**asdict(row), "occupancy_radius_m": CIRCLE_RADIUS_M}
                             for row in rows]
            record = {
                'epoch': self.epoch, 'scan_stamp_ns': scan, 'image_stamp_ns': image or None,
                'publish_stamp_ns': now, 'status': status, 'states': logged_states,
                'groups': [asdict(group) for group in groups],
                'kf': self.states.diagnostics if self.kalman and entry else [],
                'processing_ms': entry.processing_ns / 1e6 if entry else None,
                'callback_to_publish_ms': latency,
            }
            self.log.write(json.dumps(record, allow_nan=False, separators=(',', ':')) + '\n')

    def publish_clusters(self, header, entry, groups):
        """Color the actual projected points assigned to each SORT track."""
        marker = Marker(header=header, ns='associated_lidar', id=0, type=Marker.POINTS)
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = .049
        marker.lifetime = Duration(seconds=.4).to_msg()
        palette = ((.18, 1., .68), (.97, .26, .80), (.34, .68, 1.), (.71, .9, .18))
        points = {beam.index: beam.xy for beam in (entry.projected or [])} if entry else {}
        for group in groups:
            if group.track_id is None:
                continue
            index = sum(group.track_id.encode()) % len(palette)
            red, green, blue = palette[index]
            for beam in group.beams:
                x, y = points[beam]
                marker.points.append(Point(x=x, y=y, z=.11))
                marker.colors.append(ColorRGBA(r=red, g=green, b=blue, a=1.))
        marker.action = Marker.ADD if marker.points else Marker.DELETE
        self.cluster_pub.publish(marker)


def main():
    """Spin the standalone fusion node."""
    rclpy.init()
    node = None
    try:
        node = PedestrianNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
