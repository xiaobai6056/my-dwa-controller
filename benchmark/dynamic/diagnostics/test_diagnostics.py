"""Diagnostic rules and GUI command isolation tests."""
import math
import unittest
from dynamic.diagnostics import analyze
from dynamic.diagnostics import run_once


class DiagnosticTests(unittest.TestCase):
    def test_percentiles_and_deadline_are_explicit(self):
        result = analyze.distribution([10, 20, 30, 40, 50, 60], 50)
        self.assertEqual(result['n'], 6)
        self.assertEqual(result['median'], 35)
        self.assertAlmostEqual(result['over_budget_fraction'], 1/6)
        self.assertAlmostEqual(result['p95'], 57.5)

    def test_response_requires_persistence_and_excludes_post_contact(self):
        samples = [(0, .26, 0), (1, .26, 0), (2, .1, 0),
                   (2.1, .26, 0), (3, .15, 0), (3.3, .15, 0)]
        self.assertEqual(analyze.first_response(samples, 1.5, 4), 3)
        self.assertIsNone(analyze.first_response(samples, 1.5, 2.2))

    def test_interaction_is_swept_robot_envelope_not_center_distance(self):
        scene = dict(size=[.5, .5, .7])
        self.assertTrue(analyze.in_interaction([1.6, -.3, 0], scene))
        self.assertFalse(analyze.in_interaction([1.6, -.8, 0], scene))

    def test_gui_changes_only_visual_launch_arguments(self):
        gui = run_once.launch_command('D01', 'mydwa', True)
        headless = run_once.launch_command('D01', 'mydwa', False)
        self.assertEqual(gui[:-1], headless[:-1])
        self.assertEqual(gui[-1], 'gui')
        self.assertEqual(headless[-1], 'headless')


class FailureAndAssociationTests(unittest.TestCase):
    def test_recorded_collision_is_completed_measurement(self):
        self.assertEqual(run_once.exit_status(dict(cleanup_returncode=0, runner_returncode=0), True), 0)

    def test_infrastructure_failure_and_incomplete_run_fail(self):
        for extra in ({'supervisor_error':'launch_exited'}, {'runner_returncode':2}, {'interrupted':True}, {'cleanup_returncode':1}):
            state = dict(cleanup_returncode=0, runner_returncode=0)
            state.update(extra)
            self.assertNotEqual(run_once.exit_status(state, True), 0)
        self.assertNotEqual(run_once.exit_status(dict(cleanup_returncode=0, runner_returncode=0), False), 0)

    def test_inscribed_inflation_is_not_lethal(self):
        row = dict(obstacle_grid_pose=[0,0,0], width=3, height=1,
                   origin=[-.15,-.05], resolution=.1, data=[99,99,99])
        self.assertFalse(analyze.cost_association(row,[.5,.5,.7])['valid'])
        row['data'] = [100,100,100]
        self.assertEqual(analyze.cost_association(row,[.5,.5,.7])['lethal_cells'],3)

    def test_scan_association_requires_matching_contiguous_rays(self):
        row = dict(laser_world=[0,0,0], obstacle_world=[2,0,0],
                   angle_min=-.01, angle_increment=.01, ranges=[1.75,1.75,1.75])
        self.assertTrue(analyze.scan_association(row,[.5,.5,.7])['valid'])
        row['ranges'] = [.5,.5,.5]
        self.assertFalse(analyze.scan_association(row,[.5,.5,.7])['valid'])

if __name__ == '__main__':
    unittest.main()
