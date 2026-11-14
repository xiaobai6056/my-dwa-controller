"""Exercise the typed wire interface with exact-time transforms and a scan gap."""

import json
import math
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection2D, Detection2DArray

from my_dwa_controller.msg import PedestrianStateArray
from pedestrian_perception.node import PedestrianNode

MS = 1_000_000


def stamp(ns):
    """Convert integer nanoseconds to a ROS message."""
    return Time(nanoseconds=ns).to_msg()


def ns(value):
    """Convert a builtin Time or Duration to nanoseconds."""
    return value.sec * 1000 * MS + value.nanosec


def transform(parent, child, timestamp, x=0., optical=False, y=0.):
    """Create an encoder or static transform for the fixture."""
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = parent, child
    tf.header.stamp = stamp(timestamp)
    tf.transform.translation.x, tf.transform.translation.y = x, y
    q = tf.transform.rotation
    if optical:
        q.x, q.y, q.z, q.w = -.5, .5, -.5, .5
    else:
        q.w = 1.
    return tf


class RosDefaultsTest(unittest.TestCase):
    """Load the deployed estimator and its installed calibration."""

    def test_default_estimator(self):
        """The standalone node defaults to the frozen trimmed-centroid KF."""
        rclpy.init(args=[])
        node = None
        try:
            node = PedestrianNode()
            self.assertEqual(node.position_estimator, 'trimmed_centroid')
            self.assertTrue(node.kalman)
            config = node.get_parameter('kf_config').value
            self.assertTrue(config.endswith('/share/my_dwa_controller/config/kf_frozen.json'))
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()


class RosInterfaceTest(unittest.TestCase):
    """Send real DDS inputs and observe the independent state topic."""

    def setUp(self):
        """Create both nodes in one executor and discover endpoints."""
        rclpy.init(args=['--ros-args', '-p', 'state_estimator:=legacy',
                         '-p', 'position_estimator:=aabb'])
        self.fusion = PedestrianNode()
        self.source = rclpy.create_node('pedestrian_test_source')
        self.executor = SingleThreadedExecutor()
        for node in (self.fusion, self.source):
            self.executor.add_node(node)
        self.rows = []
        self.source.create_subscription(
            PedestrianStateArray, '/my_dwa/pedestrians', self.rows.append, 10)
        self.scan = self.source.create_publisher(LaserScan, '/scan', 10)
        self.sort = self.source.create_publisher(Detection2DArray, '/sort_prototype/tracks', 10)
        self.info = self.source.create_publisher(
            CameraInfo, '/visual_prototype/camera/camera_info', 10)
        self.tf = self.source.create_publisher(TFMessage, '/tf', 10)
        self.ipm = self.source.create_publisher(String, '/state_compare/vision', 10)
        static_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static = self.source.create_publisher(TFMessage, '/tf_static', static_qos)
        publishers = (self.scan, self.sort, self.info, self.tf, self.static, self.ipm)
        self.assertTrue(self.until(
            lambda: all(pub.get_subscription_count() for pub in publishers)))

    def tearDown(self):
        """Dispose of DDS endpoints."""
        self.executor.shutdown()
        self.fusion.destroy_node()
        self.source.destroy_node()
        rclpy.try_shutdown()

    def until(self, condition, timeout=2.):
        """Spin both nodes until an event is observed."""
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)
        return condition()

    def frame(self, when, robot_x=0., bracket=True, sweeping=False, ipm_matches=None,
              enough_returns=True):
        """Publish source-stamped TF, a SORT box, and a three-return scan."""
        # Image time is 20 ms after scan time. Moving-base extrinsics matter.
        image_time = when + 20 * MS
        encoder = [
            transform('odom', 'base_footprint', when - 10 * MS, robot_x),
            transform('odom', 'base_footprint', image_time + 10 * MS if bracket else
                      image_time + 100 * MS, robot_x + (.08 if sweeping else 0.),
                      y=.32 if sweeping else 0.),
        ]
        if not bracket:
            # These timestamps would appear to fill the hole if tf2-rejected samples
            # were counted as valid interpolation endpoints.
            for timestamp in (when - 2 * MS, image_time + 2 * MS):
                invalid = transform('odom', 'base_footprint', timestamp, robot_x)
                invalid.transform.rotation.w = 0.
                encoder.append(invalid)
        self.tf.publish(TFMessage(transforms=encoder))
        track = Detection2D(id='17')
        track.bbox.center.position.x = 338. if sweeping else 320.
        track.bbox.center.position.y = 240.
        track.bbox.size_x, track.bbox.size_y = 28. if sweeping else 200., 200.
        tracks = Detection2DArray(detections=[track])
        tracks.header.frame_id = 'camera_rgb_optical_frame'
        tracks.header.stamp = stamp(image_time)
        track.header = tracks.header
        if ipm_matches is not None:
            other = Detection2D(id='28', header=tracks.header, bbox=track.bbox)
            tracks.detections.append(other)
            self.ipm.publish(String(data=json.dumps({
                'stamp_ns': image_time, 'epoch': 0, 'frame_id': 'odom',
                'source_frame': tracks.header.frame_id,
                'objects': [
                    {'id': '17', 'position_valid': True, 'position': [2., 0.],
                     'bbox': [220., 140., 420., 340.]},
                    {'id': '28', 'position_valid': True, 'position': [4., 0.],
                     'bbox': [220. if ipm_matches else 221., 140., 420., 340.]},
                ],
            })))
        self.sort.publish(tracks)
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'base_scan', stamp(when)
        scan.angle_min, scan.angle_max, scan.angle_increment = -.01, .01, .01
        scan.range_min, scan.range_max = .1, 10.
        scan.time_increment = .005 if sweeping else 0.
        scan.ranges = [2., 2., 2.] if enough_returns else [2., 2., float('inf')]
        self.scan.publish(scan)
        self.assertTrue(self.until(lambda: any(ns(row.scan_stamp) == when and
                                               row.status != 'scan_timeout' for row in self.rows)))
        return next(row for row in self.rows if ns(row.scan_stamp) == when)

    def fixed_mounts_and_calibration(self, now):
        """Publish the stationary sensor extrinsics and source camera intrinsics."""
        self.static.publish(TFMessage(transforms=[
            transform('base_footprint', 'base_scan', 0),
            transform('base_footprint', 'camera_rgb_optical_frame', 0, optical=True),
        ]))
        info = CameraInfo(width=640, height=480)
        info.header.frame_id = 'camera_rgb_optical_frame'
        info.header.stamp = stamp(now - 300 * MS)
        info.k = [320., 0., 320., 0., 320., 240., 0., 0., 1.]
        self.info.publish(info)

    def start_sim_clock(self):
        """Use controlled /clock messages while the real steady timer keeps running."""
        result, = self.fusion.set_parameters([Parameter('use_sim_time', value=True)])
        self.assertTrue(result.successful)
        self.clock = self.source.create_publisher(Clock, '/clock', 10)
        self.assertTrue(self.until(lambda: self.clock.get_subscription_count() > 0))

    def advance_clock(self, now):
        """Wait until fusion has applied the given ROS clock tick."""
        self.clock.publish(Clock(clock=stamp(now)))
        self.assertTrue(self.until(lambda: self.fusion.get_clock().now().nanoseconds == now))

    def seed_velocity(self):
        """Produce an OLS history at the source stamps preceding the audited timeout."""
        self.start_sim_clock()
        self.advance_clock(7952 * MS)
        self.fixed_mounts_and_calibration(7952 * MS)
        for ms in (7852, 8060, 8268):
            self.advance_clock((ms + 100) * MS)
            output = self.frame(ms * MS, robot_x=(ms - 7852) / 2000)
        state, = output.states
        self.assertTrue(state.position_valid)
        self.assertTrue(state.velocity_valid)
        self.assertAlmostEqual(state.velocity.x, .5)
        return state

    def scan_timeout_at(self, now):
        """Advance only the ROS clock and wait for the scan timeout publication."""
        self.advance_clock(now)
        self.assertTrue(self.until(lambda: self.rows[-1].status == 'scan_timeout'
                                   and ns(self.rows[-1].header.stamp) == now))
        return self.rows[-1]

    def test_sort_pair_times_out_before_next_scan(self):
        """Finalize an unpaired scan on the steady timer while ROS time is paused."""
        self.start_sim_clock()
        when = 10_000 * MS
        self.advance_clock(when)
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'base_scan', stamp(when)
        scan.angle_min, scan.angle_max, scan.angle_increment = -.01, .01, .01
        scan.range_min, scan.range_max = .1, 10.
        scan.ranges = [2., 2., 2.]
        self.scan.publish(scan)
        self.assertTrue(self.until(lambda: bool(self.fusion.pending)))
        received = self.fusion.pending[0].received
        self.assertTrue(self.until(lambda: bool(self.rows), timeout=.18))
        elapsed_ms = (time.monotonic_ns() - received) / MS
        row, = self.rows
        self.assertEqual(row.status, 'missing_sort_pair')
        self.assertEqual(ns(row.scan_stamp), when)
        self.assertEqual(ns(row.image_stamp), 0)
        self.assertGreaterEqual(elapsed_ms, 110.)
        self.assertLess(elapsed_ms, 200.)

    def test_sort_pair_uses_inclusive_50_ms_source_tolerance(self):
        """Accept a 50 ms source offset and reject a 50 ms plus 1 ns offset."""
        when = 10_000 * MS
        self.fusion.sort_times = [when + 50 * MS]
        self.assertEqual(self.fusion.pair(when, False), when + 50 * MS)
        self.fusion.sort_times = [when + 50 * MS + 1]
        self.assertEqual(self.fusion.pair(when, True), 0)

    def test_in_tolerance_sort_keeps_tf_dependency_budget(self):
        """Allow a paired scan to acquire TF after the SORT deadline."""
        self.start_sim_clock()
        when = 10_000 * MS
        self.advance_clock(when + 100 * MS)
        self.fixed_mounts_and_calibration(when + 100 * MS)
        sort = Detection2DArray()
        sort.header.stamp = stamp(when + 20 * MS)
        sort.header.frame_id = 'camera_rgb_optical_frame'
        track = Detection2D(id='17', header=sort.header)
        track.bbox.center.position.x = 320.
        track.bbox.center.position.y = 240.
        track.bbox.size_x, track.bbox.size_y = 200., 200.
        sort.detections.append(track)
        self.sort.publish(sort)
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'base_scan', stamp(when)
        scan.angle_min, scan.angle_max, scan.angle_increment = -.01, .01, .01
        scan.range_min, scan.range_max = .1, 10.
        scan.ranges = [2., 2., 2.]
        self.scan.publish(scan)
        self.assertTrue(self.until(lambda: bool(self.fusion.pending)))
        self.assertFalse(self.until(lambda: bool(self.rows), timeout=.14))
        self.tf.publish(TFMessage(transforms=[
            transform('odom', 'base_footprint', when - 10 * MS),
            transform('odom', 'base_footprint', when + 30 * MS),
        ]))
        self.assertTrue(self.until(lambda: bool(self.rows)))
        row, = self.rows
        self.assertEqual(row.status, 'ok')
        self.assertEqual(ns(row.image_stamp), when + 20 * MS)
        self.assertTrue(row.states[0].position_valid)

    def test_scan_timeout_preserves_continuous_velocity_history(self):
        """Keep the OLS fit across the audited 211 ms source interval."""
        self.seed_velocity()
        self.scan_timeout_at(8600 * MS)
        resumed = self.frame(8479 * MS, robot_x=(8479 - 7852) / 2000)
        state, = resumed.states
        self.assertTrue(state.position_valid)
        self.assertEqual(ns(state.last_observation_stamp), 8479 * MS)
        self.assertTrue(state.velocity_valid)
        self.assertTrue(state.direct_velocity_valid)
        self.assertAlmostEqual(state.velocity.x, .5)
        self.assertAlmostEqual(state.direct_velocity.x, .5)

    def test_scan_timeout_then_source_gap_restarts_velocity_history(self):
        """Reset at a real 301 ms gap and refit after receiving new samples."""
        self.seed_velocity()
        self.scan_timeout_at(8600 * MS)
        resumed = self.frame(8569 * MS, robot_x=.36)
        state, = resumed.states
        self.assertTrue(state.position_valid)
        self.assertFalse(state.velocity_valid)
        self.assertFalse(state.direct_velocity_valid)
        self.assertFalse(state.diagnostic_velocity_valid)

        self.advance_clock(8869 * MS)
        next_state, = self.frame(8769 * MS, robot_x=.56).states
        self.assertTrue(next_state.direct_velocity_valid)
        self.assertFalse(next_state.velocity_valid)

        self.advance_clock(9069 * MS)
        refitted, = self.frame(8969 * MS, robot_x=.76).states
        self.assertTrue(refitted.velocity_valid)
        self.assertAlmostEqual(refitted.velocity.x, 1.)

    def test_sustained_scan_timeout_retains_last_observation(self):
        """Publish invalid states during continued loss with increasing observation age."""
        measured = self.seed_velocity()
        for ms in (8600, 8800, 9000):
            with self.subTest(clock_ms=ms):
                timeout = self.scan_timeout_at(ms * MS)
                stale, = timeout.states
                self.assertFalse(stale.position_valid)
                self.assertFalse(stale.velocity_valid)
                self.assertFalse(stale.direct_velocity_valid)
                self.assertFalse(stale.diagnostic_velocity_valid)
                self.assertEqual(stale.track_id, measured.track_id)
                self.assertEqual(stale.position, measured.position)
                self.assertEqual(stale.velocity, measured.velocity)
                self.assertEqual(ns(timeout.scan_stamp), 8268 * MS)
                self.assertEqual(ns(timeout.image_stamp), 0)
                self.assertEqual(ns(stale.last_observation_stamp), 8268 * MS)
                self.assertEqual(ns(stale.observation_age), (ms - 8268) * MS)

    def test_short_lidar_miss_publishes_held_velocity_with_original_stamp(self):
        """The wire flags distinguish a held OLS velocity from a measured position."""
        measured = self.seed_velocity()
        self.advance_clock(8568 * MS)
        missed = self.frame(8468 * MS, enough_returns=False)
        held, = missed.states
        self.assertEqual(held.track_id, measured.track_id)
        self.assertFalse(held.position_valid)
        self.assertTrue(held.velocity_valid)
        self.assertFalse(held.direct_velocity_valid)
        self.assertEqual(held.position, measured.position)
        self.assertEqual(held.velocity, measured.velocity)
        self.assertEqual(ns(held.last_observation_stamp), 8268 * MS)

        self.advance_clock(8768 * MS)
        resumed = self.frame(8668 * MS, robot_x=(8668 - 7852) / 2000)
        fresh, = resumed.states
        self.assertEqual(fresh.track_id, measured.track_id)
        self.assertTrue(fresh.position_valid)
        self.assertFalse(fresh.velocity_valid)
        self.assertFalse(fresh.direct_velocity_valid)

    def test_clock_rewind_clears_measured_state(self):
        """A backward /clock jump discards even history retained during timeout."""
        self.seed_velocity()
        self.scan_timeout_at(8600 * MS)
        self.advance_clock(7500 * MS)
        self.assertTrue(self.until(lambda: self.rows[-1].status == 'clock_reset'))
        reset = self.rows[-1]
        self.assertEqual(reset.epoch, 1)
        self.assertEqual(len(reset.states), 0)
        self.assertEqual(ns(reset.scan_stamp), 0)

        self.advance_clock(7700 * MS)
        self.fixed_mounts_and_calibration(7700 * MS)
        resumed = self.frame(7600 * MS)
        state, = resumed.states
        self.assertEqual(resumed.epoch, 1)
        self.assertTrue(state.position_valid)
        self.assertFalse(state.velocity_valid)
        self.assertFalse(state.direct_velocity_valid)
        self.assertFalse(state.diagnostic_velocity_valid)
        self.assertEqual(ns(state.last_observation_stamp), 7600 * MS)

    def test_overlap_requires_same_sort_boxes_in_ipm(self):
        """A conflicting IPM box cannot eliminate the competing SORT ID."""
        now = self.source.get_clock().now().nanoseconds
        self.fixed_mounts_and_calibration(now)
        rejected = self.frame(now - 100 * MS, ipm_matches=False)
        self.assertTrue(all(not state.position_valid for state in rejected.states))
        resolved = self.frame(now + 200 * MS, ipm_matches=True)
        self.assertGreaterEqual(ns(resolved.header.stamp), ns(resolved.image_stamp))
        self.assertEqual(sum(state.position_valid for state in resolved.states), 1)
        resolved_rows = self.fusion.states.snapshot(ns(resolved.header.stamp))
        self.assertEqual([row.sort_id for row in resolved_rows if row.position_valid], ['17'])

    def test_exact_source_tf_typed_output_and_recovery(self):
        """Reject a TF hole, retain stale state, and restart after the interruption."""
        now = self.source.get_clock().now().nanoseconds
        self.fixed_mounts_and_calibration(now)
        when = now - 100 * MS
        output = self.frame(when, robot_x=.2, sweeping=True)
        self.assertEqual(output.header.frame_id, 'odom')
        self.assertEqual(ns(output.image_stamp), when + 20 * MS)
        state, = output.states
        self.assertEqual(state.track_id, 'motion-1')
        self.assertTrue(state.position_valid)
        self.assertFalse(state.velocity_valid)
        self.assertAlmostEqual(state.position.x, .23 + 2 * math.cos(.01), places=6)
        self.assertAlmostEqual(state.position.y, .12, places=6)
        self.assertAlmostEqual(state.occupancy_radius, .3515418249751633)
        self.assertEqual(ns(state.last_observation_stamp), when)
        self.assertEqual(ns(state.observation_age), ns(output.header.stamp) - when)

        missing = self.frame(when + 200 * MS, robot_x=.4, bracket=False)
        self.assertIn(missing.status, ('scan_tf_unavailable', 'camera_tf_unavailable'))
        stale, = missing.states
        self.assertFalse(stale.position_valid)
        self.assertFalse(stale.velocity_valid)
        self.assertEqual(stale.position, state.position)
        self.assertEqual(ns(stale.last_observation_stamp), when)
        recovered = self.frame(self.source.get_clock().now().nanoseconds - 50 * MS, robot_x=.5)
        recovered_state, = [state for state in recovered.states if state.position_valid]
        self.assertFalse(recovered_state.direct_velocity_valid)


if __name__ == '__main__':
    unittest.main()
