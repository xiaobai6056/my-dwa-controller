"""Timing-only parameterization, observation verification and safe classification."""
import importlib.util
import unittest

class SweepTests(unittest.TestCase):
    def api(self):
        name='dynamic.timing_sweep.rules'
        self.assertIsNotNone(importlib.util.find_spec(name),'timing sweep rules must exist')
        from dynamic.timing_sweep import rules
        return rules

    def test_timing_shift_preserves_geometry_and_velocity(self):
        r=self.api()
        baseline=r.scene(4.)
        self.assertEqual(baseline,r.original_scene())
        for start in (2.,3.,5.,6.):
            value=r.scene(start)
            self.assertEqual(value['size'],baseline['size'])
            self.assertEqual(value['start'],baseline['start'])
            self.assertEqual(value['goal'],baseline['goal'])
            self.assertEqual([f[1:] for f in value['keyframes']],[f[1:] for f in baseline['keyframes']])
            self.assertAlmostEqual(value['keyframes'][-1][0]-value['keyframes'][1][0],8.8)
            self.assertEqual(value['keyframes'][1][0],start)

    def test_wrong_motion_timing_is_rejected_from_actual_trace(self):
        r=self.api();epoch=10.
        gt=[dict(t=epoch+t,obstacle=[1.6,y,0]) for t,y in [(0,-1.1),(2,-1.1),(2.1,-1.075),(3,-.875)]]
        self.assertTrue(r.verify_motion(gt,[],r.scene(2),epoch)['valid'])
        self.assertFalse(r.verify_motion(gt,[],r.scene(4),epoch)['valid'])
        self.assertFalse(r.verify_motion([gt[0],gt[1]],[],r.scene(2),epoch)['valid'])

    def test_sustained_response_excludes_startup_and_short_spikes(self):
        r=self.api()
        samples=[(0,.1,0),(1,.26,0),(2,.1,0),(2.1,.26,0),(3,.2,0),(3.3,.2,0)]
        self.assertEqual(r.response(samples,1.5,4,'brake'),3)
        self.assertIsNone(r.response(samples,1.5,2.5,'brake'))

class BehaviorTests(unittest.TestCase):
    api=SweepTests.api
    def test_clear_temporal_separation_without_response_is_not_avoidance(self):
        r=self.api()
        self.assertTrue(hasattr(r,'classify'),'conservative classification must exist')
        features=dict(success=True,collision=False,brake_s=None,turn_s=None,actual_brake_s=None,
            waiting_before_zone_s=0,lateral_excursion_m=.01,robot_windows=[(5,6)],obstacle_windows=[(8,10)],min_gap_m=.7)
        self.assertEqual(r.classify(features),'no_interaction_robot_first')
        features['min_gap_m']=.14
        self.assertEqual(r.classify(features),'no_interaction_robot_first')
        features['robot_window_right_censored']=True
        self.assertEqual(r.classify(features),'interaction_pass_unknown')
        features['robot_window_right_censored']=False
        features.update(robot_windows=[(11,12)],obstacle_windows=[(5,8)])
        self.assertEqual(r.classify(features),'no_interaction_obstacle_first')
        features.update(waiting_before_zone_s=1,brake_s=4,actual_brake_s=4)
        self.assertEqual(r.classify(features),'observed_wait_then_pass')

    def test_collision_and_uncertainty_cannot_be_claimed_as_successful_avoidance(self):
        r=self.api()
        self.assertTrue(hasattr(r,'classify'),'conservative classification must exist')
        features=dict(success=False,collision=True,brake_s=None,turn_s=4,actual_brake_s=None,
            waiting_before_zone_s=0,lateral_excursion_m=.2,robot_windows=[(5,8)],obstacle_windows=[(6,9)],min_gap_m=0)
        self.assertEqual(r.classify(features),'collision_with_turn')
        features.update(success=True,collision=False,turn_s=None,lateral_excursion_m=.01,
            robot_windows=[(5,6)],obstacle_windows=[(6.05,9)],min_gap_m=.5)
        self.assertEqual(r.classify(features),'interaction_pass_unknown')

class CommandEvidenceTests(unittest.TestCase):
    api=SweepTests.api
    def test_absent_or_sparse_command_evidence_is_unknown(self):
        r=self.api()
        self.assertTrue(hasattr(r,'command_coverage'),'command coverage must be validated')
        self.assertFalse(r.command_coverage([],1.5,4)['complete'])
        self.assertFalse(r.command_coverage([(1.5,.26,0),(4,.26,0)],1.5,4)['complete'])
        samples=[(1.5+i*.1,.26,0) for i in range(26)]
        self.assertTrue(r.command_coverage(samples,1.5,4)['complete'])
        features=dict(success=True,collision=False,command_evidence_complete=False,
            brake_s=None,turn_s=None,actual_brake_s=None,waiting_before_zone_s=0,
            lateral_excursion_m=0,robot_windows=[(5,6)],obstacle_windows=[(8,10)],min_gap_m=.14)
        self.assertEqual(r.classify(features),'pass_behavior_unknown_command_evidence')
        features.update(collision=True,success=False)
        self.assertEqual(r.classify(features),'collision_behavior_unknown_command_evidence')

if __name__=='__main__':unittest.main()