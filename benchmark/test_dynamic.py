"""Independent dynamic benchmark boundary and geometry regression tests."""
import math
import io
import json
from types import SimpleNamespace
from unittest.mock import patch
import tempfile
import unittest
from pathlib import Path
from dynamic import evaluation as ev
from dynamic import scenario


class DynamicTests(unittest.TestCase):
    def test_gap_uses_boundaries_and_rotation(self):
        a = ev.box_polygon(0, 0, 0, [-.2, -.1, .2, .1])
        b = ev.box_polygon(1, 0, 0, [-.3, -.3, .3, .3])
        self.assertAlmostEqual(ev.polygon_gap(a, b), .5)
        self.assertEqual(ev.polygon_gap(a, a), 0)
        rotated = ev.box_polygon(0, 0, math.pi / 2, [-.2, -.1, .2, .1])
        self.assertAlmostEqual(ev.polygon_gap(rotated, b), .6)

    def test_motion_is_independent_and_clamped(self):
        s = scenario.load('D01')
        self.assertEqual(scenario.position(s, -1), [1.6, -1.1])
        self.assertAlmostEqual(scenario.position(s, 8.4)[1], 0)
        self.assertEqual(scenario.position(s, 100), [1.6, 1.1])
        d = scenario.load('D02')
        self.assertEqual(scenario.position(d, 10), [2., 0.])
        self.assertEqual(scenario.position(d, 21)[0], 2.)
        self.assertGreater(scenario.position(d, 24.4)[1] - .4, .45)

    def test_generated_model_has_collision_and_sensors(self):
        import xml.etree.ElementTree as ET
        with tempfile.TemporaryDirectory() as tmp:
            scenario.generate('D02', Path(tmp))
            tree = ET.parse(Path(tmp) / 'world.sdf')
            model = tree.find('.//model[@name="dynamic_obstacle"]')
            self.assertIsNotNone(model.find('.//collision/geometry/box'))
            self.assertIsNotNone(model.find('.//sensor[@type="contact"]'))
            self.assertIsNotNone(
                tree.find('.//plugin[@filename="libgazebo_ros_state.so"]'))
            names = [p.get('name') for p in tree.findall('.//plugin')]
            self.assertEqual(
                len(names),
                len(set(names)),
                'Gazebo ROS node names must be unique')

    def test_action_and_collision_classification(self):
        self.assertEqual(
            ev.termination(
                'SUCCEEDED',
                None,
                False,
                True),
            'success')
        self.assertEqual(
            ev.termination(
                'SUCCEEDED',
                None,
                True,
                True),
            'collision')
        self.assertEqual(
            ev.termination(
                'FAILED',
                None,
                False,
                True),
            'navigation_failed')
        self.assertEqual(
            ev.termination(
                'SUCCEEDED',
                None,
                False,
                False),
            'system_error')
        self.assertEqual(
            ev.termination(
                'CANCELED',
                'sim_timeout',
                False,
                True),
            'timeout')
        self.assertEqual(
            ev.termination(
                None,
                'clock_stalled',
                False,
                True),
            'system_error')

    def test_command_deadband_and_persistence(self):
        self.assertEqual(
            ev.direction_switches(
                [(0, .2),
                 (.21, .2),
                 (.3, .001),
                 (.4, -.2),
                 (.7, -.2),
                 (1., .2),
                 (1.3, .2)]),
            2)
        self.assertEqual(ev.direction_switches([(0, .2), (.4, .2), (.5, -.2),
                                               (.6, .2), (1., .2)]), 0)

    def test_action_client_exception_is_system_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            (directory / 'summary.json').write_text(json.dumps(dict(
                scene='D01', controller='mydwa', valid=True,
                action_result=None, action_error='client exception',
                navigation_start_sim_s=0, navigation_end_sim_s=1)))
            result = ev.calculate(directory)
            self.assertEqual(result['termination'], 'system_error')
            self.assertEqual(result['error'], 'client exception')

    def test_cleanup_exception_prevents_next_trial(self):
        from dynamic import run_suite as suite
        trial = {}
        runner, launch = object(), object()
        with patch.object(suite, 'stop_process', side_effect=[RuntimeError('stop'), None]) as stop:
            response = SimpleNamespace(returncode=0)
            with patch.object(suite.subprocess, 'run', return_value=response) as cleanup:
                suite.cleanup_processes(
                    runner, launch, {}, io.StringIO(), trial)
        self.assertEqual(stop.call_count, 2)
        cleanup.assert_called_once()
        self.assertEqual(trial['cleanup_returncode'], 1)
        self.assertIn('stop', trial['cleanup_error'])


if __name__ == '__main__':
    unittest.main()
