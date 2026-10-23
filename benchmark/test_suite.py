"""Scheduler aggregation must retain failed attempts and audit common parameters."""
import importlib.util
import math
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


class SuiteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).with_name('run_suite.py')
        cls.m = None
        if path.exists():
            spec = importlib.util.spec_from_file_location('run_suite', path)
            cls.m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.m)

    def setUp(self):
        self.assertIsNotNone(self.m, 'suite runner is not implemented')

    def test_failed_attempt_values_are_included_and_missing_values_report_n(self):
        result = self.m.statistics([4., 8., None])
        self.assertEqual(result['n'], 2)
        self.assertEqual(result['mean'], 6.)
        self.assertAlmostEqual(result['sample_sd'], math.sqrt(8.))

    def test_fairness_ignores_algorithm_specific_weights_but_checks_constraints(self):
        a = {'arguments': {'start': [0, 0, 0], 'goal': [1, 0, 0]},
             'runtime_parameters': {'controller_server': {
                 'FollowPath.plugin': 'a', 'FollowPath.speed_weight': 1.,
                 'FollowPath.max_vel_x': .26, 'general_goal_checker.xy_goal_tolerance': .25}},
             'config_files': []}
        import copy
        b = copy.deepcopy(a)
        b['runtime_parameters']['controller_server']['FollowPath.plugin'] = 'b'
        b['runtime_parameters']['controller_server']['FollowPath.speed_weight'] = 10.
        self.assertEqual(self.m.fairness_signature(a), self.m.fairness_signature(b))
        b['runtime_parameters']['controller_server']['FollowPath.max_vel_x'] = .5
        self.assertNotEqual(self.m.fairness_signature(a), self.m.fairness_signature(b))

    def test_fairness_requires_both_controllers_and_resource_hashes(self):
        self.assertFalse(self.m.fairness_status({'same'}, {'mydwa'}, {'mydwa', 'dwb'})[
            'common_runtime_matches'])
        self.assertTrue(self.m.fairness_status({'same'}, {'mydwa', 'dwb'}, {'mydwa', 'dwb'})[
            'common_runtime_matches'])
        a = {'arguments': {'start': [], 'goal': []}, 'config_files': [
            {'saved': 'configs/map_map.yaml', 'sha256': 'a'},
            {'saved': 'configs/bt_tree.xml', 'sha256': 'b'}]}
        import copy
        b = copy.deepcopy(a)
        b['config_files'][0]['sha256'] = 'changed_origin'
        self.assertNotEqual(self.m.fairness_signature(a), self.m.fairness_signature(b))
        b = copy.deepcopy(a)
        b['config_files'][1]['sha256'] = 'changed_bt'
        self.assertNotEqual(self.m.fairness_signature(a), self.m.fairness_signature(b))

    def test_scheduler_keeps_failure_artifacts_and_continues_next_scene(self):
        calls = []

        def fake_run(directory, trial, env):
            calls.append(trial['scene'])
            if len(calls) == 1:
                raise RuntimeError('deliberate launch failure fixture')
            path = directory / trial['directory']
            path.mkdir(parents=True)
            (path / 'trajectory.csv').write_text('sim_time_s,x,y,yaw\n')
            self.m.write_json(path / 'summary.json', {
                'trial_id': path.name, 'status': 'preflight_failed', 'valid': False,
                'arguments': {'start': [0, 0, 0], 'goal': [1, 0, 0]}})
            trial['state'] = 'preflight_failed'

        argv = ['run_suite.py', '--controllers', 'mydwa', '--scenes', 'S01', 'S02',
                '--repeats', '1', '--run-id', 'failure_fixture']
        with tempfile.TemporaryDirectory() as temporary, patch.object(
                self.m, 'HERE', Path(temporary)), patch.object(self.m, 'run_one', fake_run), (
                patch.object(self.m, 'resources', return_value=(None, None))), (
                patch.object(self.m, 'audit_configurations', return_value={})), (
                patch('sys.argv', argv)), patch.dict(os.environ, {'ROS_DISTRO': 'humble'}):
            self.m.main()
            root = Path(temporary) / 'results/v1/failure_fixture'
            manifest = json.loads((root / 'manifest.json').read_text())
            self.assertEqual(calls, ['S01', 'S02'])
            self.assertEqual([t['state'] for t in manifest['trials']],
                             ['supervisor_error', 'preflight_failed'])
            for trial in manifest['trials']:
                path = root / trial['directory']
                self.assertTrue((path / 'summary.json').exists())
                metrics = json.loads((path / 'metrics.json').read_text())
                self.assertIsNone(metrics['motion']['trajectory_length_m'])


if __name__ == '__main__':
    unittest.main()
