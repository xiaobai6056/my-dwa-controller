"""Numerical contracts for offline observations; no navigation policy tests."""
import unittest
import numpy as np

from scoring import rollout, sample_statistics, weighted_terms, verify_observation, selection_order, admission_errors


def grid(data, width, origin=(0., 0.), resolution=1.):
    return dict(data=data, width=width, height=len(data)//width,
                origin=origin, resolution=resolution)


class ScoringTests(unittest.TestCase):
    def test_euler_uses_start_heading_and_includes_initial_and_short_tail(self):
        result = rollout([0., 0., 0.], [[1., np.pi]], .25, .1)[0]
        self.assertEqual(result.shape, (4, 3))
        np.testing.assert_allclose(result[1], [.1, 0., np.pi*.1])
        self.assertAlmostEqual(result[-1, 2], np.pi*.25)
        self.assertAlmostEqual(result[-1, 0], .1+.1*np.cos(np.pi*.1)+.05*np.cos(np.pi*.2))

    def test_default_horizon_has_35_points(self):
        self.assertEqual(rollout([1., 2., 3.], [[.26, .1]], 1.7, .05).shape, (1, 35, 3))

    def test_repeated_cell_counts_and_two_topk_definitions(self):
        poses = np.array([[[.1,.1,0.],[.2,.1,0.],[1.1,.1,0.],[2.1,.1,0.]]])
        stats = sample_statistics(poses, grid([0, 100, 200], 3), 2)[0]
        self.assertEqual(stats['different_cells'], 3)
        self.assertEqual(stats['sample_count'], 4)
        self.assertEqual(stats['mean'], 75.)
        self.assertEqual(stats['max'], 200)
        self.assertEqual(stats['topk_sample_mean'], 150.)
        self.assertEqual(stats['topk_unique_mean'], 150.)

    def test_sample_and_unique_topk_differ(self):
        poses=np.array([[[.1,.1,0.],[1.1,.1,0.],[1.2,.1,0.],[1.3,.1,0.]]])
        s=sample_statistics(poses,grid([0,200],2),2)[0]
        self.assertEqual(s['topk_sample_mean'],200.)
        self.assertEqual(s['topk_unique_mean'],100.)
        self.assertEqual(s['different_cells'],2)

    def test_out_of_bounds_does_not_wrap_or_use_negative_truncation(self):
        poses=np.array([[[-.01,.1,0.],[.1,.1,0.]],[[2.,.1,0.],[.1,.1,0.]]])
        for s in sample_statistics(poses,grid([10,20],2),5):
            self.assertFalse(s['complete_trajectory'])
            self.assertEqual(s['out_of_bounds'],1)
            self.assertEqual(s['mean'],None)
            self.assertEqual(s['different_cells'],1)

    def test_lethal_and_unknown_are_not_smoothed_into_feasibility(self):
        poses=np.array([[[.1,.1,0.],[1.1,.1,0.],[2.1,.1,0.]]])
        s=sample_statistics(poses,grid([252,254,255],3),5)[0]
        self.assertFalse(s['collision_free'])
        self.assertEqual(s['threshold_cells'],2)
        self.assertEqual(s['topk_k_samples'],3)

    def test_threshold_cell_count_does_not_count_repeated_poses(self):
        poses=np.array([[[.1,.1,0.],[.2,.1,0.],[.3,.1,0.]]])
        s=sample_statistics(poses,grid([254],1),2)[0]
        self.assertEqual(s['threshold_cells'],1)
        self.assertEqual(s['threshold_samples'],3)

    def test_weighted_terms_not_normalized(self):
        np.testing.assert_allclose(weighted_terms([1,1,1,1,1],[.1,.1,.6,.2,.05]),[.1,.1,.6,.2,.05])
        self.assertAlmostEqual(sum(weighted_terms([1]*5,[.1,.1,.6,.2,.05])),1.05)

    def test_validation_rejects_mismatched_original_mean(self):
        poses=rollout([0.,0.,0.],[[.1,0.]],1.,.5)
        stats=sample_statistics(poses,grid([20],1),5)[0]
        obs=dict(end=poses[0,-1].tolist(),mean=21.,max=20,free=True)
        errors=verify_observation(obs,poses[0],stats)
        self.assertIn('mean',errors)

    def test_slowing_order_respects_velocity_before_yaw_score(self):
        candidates=[dict(v=0.,w=.2,scores=[0,0,0,0,0,.42]),
                    dict(v=.01,w=.2,scores=[0,0,0,0,0,.42]),
                    dict(v=0.,w=.17,scores=[0,0,0,0,0,.45])]
        self.assertEqual(selection_order(candidates,'slowing'),[0,2,1])
        self.assertEqual(selection_order(candidates,'normal'),[0,1,2])

    def test_invalid_acquisition_excluded_but_valid_collision_retained(self):
        manifest=dict(runtime_mapping_verified=True,cleanup_returncode=0,runner_returncode=2,
                      controller_parameters_match_historical=True,bag_returncode=0)
        self.assertEqual(admission_errors(dict(valid=True,stop_reason='collision'),manifest),[])
        self.assertIn('invalid_trial',admission_errors(dict(valid=False),manifest))
        self.assertIn('supervisor_error',admission_errors(dict(valid=True),dict(manifest,supervisor_error='failed')))
        self.assertIn('parameter_mismatch',admission_errors(dict(valid=True),dict(manifest,controller_parameters_match_historical=False)))

    def test_fresh_acquisition_evidence_must_be_complete(self):
        self.assertIn('missing_manifest',admission_errors(dict(valid=True)))
        m=dict(runtime_mapping_verified=True,cleanup_returncode=0,runner_returncode=2)
        self.assertIn('parameter_mismatch',admission_errors(dict(valid=True),m))
        self.assertIn('recording_failed',admission_errors(dict(valid=True),m))
        self.assertEqual(admission_errors(dict(valid=True),m,legacy=True),[])


if __name__=='__main__':
    unittest.main()
