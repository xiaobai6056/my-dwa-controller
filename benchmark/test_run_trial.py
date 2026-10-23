"""Pure-Python boundary tests; run with python3 -m unittest discover here."""
import importlib.util
import csv
from concurrent.futures import Future
import io
import json
import math
import os
from pathlib import Path
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch


class TrialTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).with_name('run_trial.py')
        if path.exists():
            spec = importlib.util.spec_from_file_location('run_trial', path)
            cls.m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.m)
        else:
            cls.m = None

    def setUp(self):
        self.assertIsNotNone(self.m, 'run_trial.py is not implemented yet')

    def test_start_wraps_yaw_at_pi_and_rejects_wrong_start(self):
        error = self.m.pose_error((0, 0, math.pi - .01), (0, 0, -math.pi + .01))
        self.assertAlmostEqual(error['yaw_rad'], .02)
        self.assertTrue(self.m.start_matches(error, .1, .05))
        self.assertFalse(self.m.start_matches(self.m.pose_error((1, 0, 0), (0, 0, 0)), .2, .2))

    @unittest.skipUnless(os.environ.get('ROS_DISTRO') == 'humble',
                         'source ROS Humble for integration check')
    def test_humble_collector_constructs(self):
        import rclpy
        rclpy.init()
        collector = None
        try:
            collector = self.m.Collector(SimpleNamespace(
                odom_topic='/odom', controller_cmd_topic='/cmd_vel_nav',
                robot_cmd_topic='/cmd_vel'),
                csv.DictWriter(io.StringIO(), self.m.FIELDS))
            self.assertEqual(collector.node.get_parameter('use_sim_time').value, True)
        finally:
            if collector is not None:
                collector.executor.shutdown()
                collector.node.destroy_node()
            rclpy.shutdown()

    def test_missing_stale_and_future_measurements_stay_empty(self):
        for stamp in (None, 1.0, 3.0):
            self.assertIsNone(self.m.fresh_value(8.0, stamp, 2.0, .5))
        self.assertEqual(self.m.fresh_value(8.0, 1.9, 2.0, .5), 8.0)

    def test_sensor_stamp_can_lead_ten_hz_clock_by_one_tick(self):
        self.assertEqual(self.m.fresh_value(8.0, 2.07, 2.0, .5), 8.0)

    def test_tf_becomes_stale_in_wall_time_when_sim_clock_freezes(self):
        c = object.__new__(self.m.Collector)
        c.args = SimpleNamespace(base_frame='base_link', max_data_age=.5)
        c.clock = SimpleNamespace(sim=10.)
        c.tf_last_stamp, c.tf_last_advance = None, None
        c.recording, c.events = True, {}
        c.Time = lambda: None
        transform = SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
            transform=SimpleNamespace(translation=SimpleNamespace(x=1., y=2.),
                                      rotation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
        c.buffer = SimpleNamespace(lookup_transform=lambda *args: transform)
        with patch('time.monotonic', return_value=100.):
            self.assertIsNotNone(c.pose()[0])
        with patch('time.monotonic', return_value=100.6):
            self.assertIsNone(c.pose()[0])
        self.assertIn('tf_unavailable', c.events)

    def test_error_cleanup_waits_for_cancellation_before_shutdown(self):
        nav = FakeNavigator(None)
        original_poll = nav.isTaskComplete
        nav.isTaskComplete = lambda: (time.sleep(.02) or original_poll())
        worker = self.m.ActionWorker(nav, object())
        worker.start()
        outcome = self.m.cancel_after_error(worker, .2)
        self.assertTrue(outcome['worker_finished'])
        self.assertTrue(outcome['cancellation_confirmed'])
        self.assertEqual(outcome['action_result'], 'CANCELED')

    def test_read_only_preflight_retries_a_lost_service_response(self):
        c = object.__new__(self.m.Collector)
        c.clock, c.events = SimpleNamespace(sim=1.), {}
        wall, calls = [0.], []

        def call(request):
            calls.append(request)
            future = Future()
            if len(calls) > 1:
                future.set_result('active')
            return future

        client = SimpleNamespace(service_is_ready=lambda: True, call_async=call)
        c.node = SimpleNamespace(create_client=lambda *args: client,
                                 destroy_client=lambda client: None)
        c.tick = lambda: wall.__setitem__(0, wall[0] + .5)
        with patch('time.monotonic', side_effect=lambda: wall[0]):
            self.assertEqual(c.service(object, '/read_only', object(), 5.,
                                       threading.Event()), 'active')
        self.assertEqual(len(calls), 2)

    def test_plan_record_keeps_map_frame_and_polyline(self):
        pose = SimpleNamespace(position=SimpleNamespace(x=1., y=2.),
                               orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
        message = SimpleNamespace(header=SimpleNamespace(
            frame_id='map', stamp=SimpleNamespace(sec=4, nanosec=500000000)),
            poses=[SimpleNamespace(pose=pose), SimpleNamespace(pose=pose)])
        record = self.m.path_record(message, 4.4)
        self.assertEqual(record['frame_id'], 'map')
        self.assertEqual(record['header_stamp_s'], 4.5)
        self.assertEqual(record['points'], [[1., 2., 0.], [1., 2., 0.]])

    def test_action_status_records_each_transition_once(self):
        c = object.__new__(self.m.Collector)
        c.recording, c.nav_event_cache = True, {}
        c.clock = SimpleNamespace(sim=1.)
        c.navigation_streams = {'nav_events': io.StringIO()}
        info = SimpleNamespace(goal_id=SimpleNamespace(uuid=[1]*16),
                               stamp=SimpleNamespace(sec=0, nanosec=0))
        status = SimpleNamespace(goal_info=info, status=2)
        message = SimpleNamespace(status_list=[status])
        c.action_status('spin', message)
        c.action_status('spin', message)
        status.status = 4
        c.action_status('spin', message)
        lines = c.navigation_streams['nav_events'].getvalue().splitlines()
        events = [json.loads(line) for line in lines]
        self.assertEqual([e['status'] for e in events], [2, 4])

    def test_clock_stall_uses_wall_time_even_if_clock_messages_repeat(self):
        clock = self.m.ClockMonitor()
        clock.update(1.0, 10.0)
        clock.update(1.0, 12.0)
        self.assertEqual(clock.problem(12.1, 2.0), 'clock_stalled')
        clock.update(.1, 12.2)
        self.assertEqual(clock.problem(12.2, 2.0), 'clock_reset')

    def run_worker(self, navigator, timeout=.04):
        clock = self.m.ClockMonitor()
        clock.update(1.0, time.monotonic())
        worker = self.m.ActionWorker(navigator, object())
        worker.start()
        outcome = self.m.watch_task(worker, lambda: time.sleep(.002), clock,
                                    timeout, .04, 10., threading.Event())
        return worker, outcome

    def test_action_failure_is_not_reclassified_as_success(self):
        nav = FakeNavigator('FAILED')
        worker, result = self.run_worker(nav)
        self.assertEqual(result['status'], 'failed')
        self.assertEqual(worker.result, 'FAILED')

    def test_success_comes_from_action(self):
        worker, result = self.run_worker(FakeNavigator('SUCCEEDED'))
        self.assertEqual(result['status'], 'succeeded')
        self.assertTrue(worker.accepted)

    def test_timeout_cancels_and_keeps_timeout_status(self):
        nav = FakeNavigator(None)
        worker, result = self.run_worker(nav)
        self.assertEqual(result['status'], 'timeout')
        self.assertEqual(worker.result, 'CANCELED')
        self.assertTrue(nav.cancelled)

    def test_blocked_goal_ack_does_not_block_result_save(self):
        release = threading.Event()
        nav = FakeNavigator(None)
        nav.goToPose = lambda goal: release.wait(1.)
        started = time.monotonic()
        worker, result = self.run_worker(nav)
        self.assertLess(time.monotonic() - started, .4)
        self.assertFalse(result['cancellation_confirmed'])
        self.assertEqual(result['status'], 'timeout')
        release.set()
        worker.cancel.set()
        worker.join(.1)


class FakeNavigator:
    def __init__(self, result):
        self.result = result
        self.cancelled = False
        self.status = None

    def goToPose(self, goal):
        return True

    def isTaskComplete(self):
        self.status = {'SUCCEEDED': 4, 'FAILED': 6, 'CANCELED': 5}.get(self.result)
        return self.result is not None

    def getResult(self):
        return SimpleNamespace(name=self.result)

    def cancelTask(self):
        self.cancelled = True
        self.result = 'CANCELED'


if __name__ == '__main__':
    unittest.main()
