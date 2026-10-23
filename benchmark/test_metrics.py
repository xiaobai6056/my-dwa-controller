"""Analytic fixtures for offline static-navigation metrics."""
import importlib.util
import math
import csv
import json
from pathlib import Path
import tempfile
import unittest


class MetricTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).with_name('metrics.py')
        cls.m = None
        if path.exists():
            spec = importlib.util.spec_from_file_location('metrics', path)
            cls.m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.m)

    def setUp(self):
        self.assertIsNotNone(self.m, 'offline metrics are not implemented')

    def test_polyline_distance_uses_segment_interior_and_degenerate_segments(self):
        path = [(0., 0.), (0., 0.), (10., 0.)]
        self.assertAlmostEqual(self.m.polyline_distance((5., 2.), path), 2.)
        self.assertAlmostEqual(self.m.polyline_distance((-1., 0.), path), 1.)

    def test_weighted_rmse_and_length_use_irregular_source_intervals(self):
        samples = [(0., 0., 0.), (.1, 1., 2.), (.4, 4., 2.)]
        result = self.m.pose_metrics(samples, [(0., 0.), (10., 0.)], max_gap=1.)
        self.assertAlmostEqual(result['lateral_rmse_m'], math.sqrt(3.5))
        self.assertAlmostEqual(result['lateral_max_m'], 2.)
        self.assertAlmostEqual(result['trajectory_length_m'], math.sqrt(5.) + 3.)

    def test_gaps_are_not_bridged_by_fabricated_motion(self):
        result = self.m.pose_metrics([(0., 0., 0.), None, (.4, 10., 0.)],
                                     [(0., 0.), (10., 0.)])
        self.assertIsNone(result['trajectory_length_m'])
        self.assertEqual(result['integrated_time_s'], 0.)
        self.assertIsNone(result['lateral_rmse_m'])

    def test_absent_data_is_null_but_observed_stationary_motion_is_zero(self):
        self.assertIsNone(self.m.velocity_metrics([])['total_variation'])
        self.assertIsNone(self.m.direction_switches([]))
        self.assertIsNone(self.m.pose_metrics([], None)['trajectory_length_m'])
        self.assertEqual(self.m.velocity_metrics([(0., 0.), (.1, 0.)])['total_variation'], 0.)
        self.assertEqual(self.m.pose_metrics([(0., 1., 0.), (.1, 1., 0.)], None)[
            'trajectory_length_m'], 0.)

    def test_duplicate_and_backward_timestamps_do_not_generate_derivatives(self):
        result = self.m.velocity_metrics([(0., 0.), (0., 1.), (-.1, 10.), (.2, 1.)])
        self.assertGreaterEqual(result['discarded_intervals'], 2)
        self.assertTrue(math.isfinite(result['mean_abs_acceleration']))

    def test_velocity_change_uses_actual_dt(self):
        result = self.m.velocity_metrics([(0., 0.), (.1, .2), (.4, .2)])
        self.assertAlmostEqual(result['mean'], .175)
        self.assertAlmostEqual(result['total_variation'], .2)
        self.assertAlmostEqual(result['mean_abs_acceleration'], .5)
        self.assertAlmostEqual(result['rms_acceleration'], 1.)
        self.assertAlmostEqual(result['max_abs_acceleration'], 2.)

    def test_direction_changes_ignore_noise_and_brief_spikes(self):
        samples = [(0., .2), (.2, .2), (.3, .01), (.4, -.2),
                   (.45, -.2), (.5, .2), (.7, .2), (.8, -.2),
                   (1.0, -.2), (1.1, .01), (1.2, .2), (1.4, .2)]
        self.assertEqual(self.m.direction_switches(samples, .05, .15), 2)

    def test_angle_wrap_near_pi(self):
        self.assertAlmostEqual(self.m.angle_error(math.pi - .01, -math.pi + .01), .02)

    def test_native_planner_timing_excludes_stale_clock_before_diagnostic_query(self):
        summary = {'navigation_sim_s': 15.5, 'navigation_end_sim_s': 19.3}
        events = [{'action': 'compute_path_to_pose', 'goal_stamp_s': 4.2,
                   'status': 2, 'goal_id': 'first'},
                  {'action': 'compute_path_to_pose', 'goal_stamp_s': 5.2,
                   'status': 4, 'goal_id': 'second'}]
        result = self.m.timing_metrics(summary, events, [{'navigation_time_s': 15.0}])
        self.assertAlmostEqual(result['first_planner_to_observed_stop_sim_s'], 15.1)
        self.assertEqual(result['legacy_phase_clock_span_s'], 15.5)
        self.assertEqual(result['nav2_last_feedback_duration_s'], 15.0)

    def test_missing_native_timing_stays_missing_not_an_invented_stamp(self):
        result = self.m.timing_metrics({'navigation_sim_s': 3.}, [], [])
        self.assertIsNone(result['first_planner_to_observed_stop_sim_s'])
        self.assertEqual(result['legacy_phase_clock_span_s'], 3.)

    def test_source_window_excludes_old_pre_goal_samples(self):
        rows = [{'stamp': '0.5', 'v': '0'}, {'stamp': '1.0', 'v': '0'},
                {'stamp': '1.2', 'v': '.2'}]
        samples = self.m.samples_from_rows(rows, 'stamp', ['v'], minimum_stamp=1.)
        self.assertIsNone(samples[0])
        self.assertAlmostEqual(self.m.velocity_metrics(samples)['mean'], .1)

    def test_native_anchor_rejects_unknown_and_out_of_trial_goals(self):
        summary = {'navigation_start_sim_s': 3., 'navigation_end_sim_s': 8.}
        events = [{'action': 'compute_path_to_pose', 'goal_stamp_s': stamp,
                   'status': status, 'goal_id': str(stamp)} for stamp, status in
                  [(1., 4), (4., 0), (9., 2), (float('nan'), 2), (5., 2), (5., 4)]]
        result = self.m.timing_metrics(summary, events, [])
        self.assertEqual(result['first_planner_goal_stamp_s'], 5.)
        self.assertEqual(result['first_planner_to_observed_stop_sim_s'], 3.)

    def test_missing_reference_is_explicit(self):
        result = self.m.pose_metrics([(0., 0., 0.), (.1, 1., 0.)], None)
        self.assertIsNone(result['lateral_rmse_m'])
        self.assertEqual(result['reference_missing_reason'], 'no map-frame reference path')

    def test_follow_path_and_planner_actions_are_not_recovery_behaviors(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / 'summary.json').write_text(json.dumps({
                'trial_id': 'fixture', 'status': 'succeeded', 'valid': True,
                'action_result': 'SUCCEEDED', 'event_monitor_ready': True,
                'arguments': {'goal': [0., 0., 1.57]}}))
            with (directory / 'trajectory.csv').open('w') as stream:
                writer = csv.DictWriter(stream, ['x', 'y', 'actual_vx', 'actual_wz'])
                writer.writeheader()
                writer.writerow({'x': 0., 'y': 0., 'actual_vx': 0., 'actual_wz': .2})
            (directory / 'nav_events.jsonl').write_text(''.join(json.dumps({
                'action': name, 'goal_id': name, 'status': 2}) + '\n'
                for name in ('follow_path', 'compute_path_to_pose')))
            result = self.m.calculate(directory)
            self.assertEqual(result['navigation_system']['behavior_goals_executed'], 0)
            self.assertEqual(result['terminal_rotation']['attribution'], 'controller')
            with (directory / 'nav_events.jsonl').open('a') as stream:
                stream.write(json.dumps({'action': 'spin', 'goal_id': 'missed_execution',
                                         'status': 4}) + '\n')
            result = self.m.calculate(directory)
            self.assertEqual(result['terminal_rotation']['attribution'], 'unconfirmed')


if __name__ == '__main__':
    unittest.main()
