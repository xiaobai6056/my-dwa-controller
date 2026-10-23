"""Regression tests: only source-stamped, registered known overlap counts as motion."""
import copy
import unittest
import temporal


def grid(t=1., ox=0., data=None, frame='odom', resolution=.05):
    return dict(type='costmap',stamp=t,t=t,frame=frame,resolution=resolution,
                origin=[ox,0.],width=5,height=3,data=data or [0]*15)

class TemporalTests(unittest.TestCase):
    def test_rolling_static_obstacle_has_no_change(self):
        a=grid(data=[0,0,100,0,0]+[0]*10)
        b=grid(t=1.6,ox=.05,data=[0,100,0,0,0]+[0]*10)
        r=temporal.History().feed(a)
        h=temporal.History();h.feed(a);r=h.feed(b)
        self.assertEqual((r['added_n'],r['removed_n']),(0,0))
        self.assertEqual(r['overlap_n'],12)
    def test_inflation_and_inscribed_only_do_not_move_lethal(self):
        a=grid();b=grid(t=1.6,data=[99,80,50,0,0]+[0]*10)
        h=temporal.History();h.feed(a);r=h.feed(b)
        self.assertEqual((r['added_n'],r['removed_n']),(0,0))
        self.assertEqual(r['inscribed_after_n'],1)
        self.assertFalse(r['motion_candidate'])
    def test_unknown_is_not_cleared_obstacle(self):
        a=grid(data=[100]+[0]*14);b=grid(t=1.6,data=[-1]+[0]*14)
        h=temporal.History();h.feed(a);r=h.feed(b)
        self.assertEqual(r['removed_n'],0)
        self.assertEqual(r['known_to_unknown_n'],1)
        self.assertEqual(r['known_overlap_n'],14)
    def test_unknown_to_lethal_is_observation_not_addition(self):
        a=grid(data=[-1]+[0]*14);b=grid(t=1.6,data=[100]+[0]*14)
        h=temporal.History();h.feed(a);r=h.feed(b)
        self.assertEqual(r['added_n'],0)
        self.assertEqual(r['unknown_to_known_n'],1)
    def test_exact_stamp_duplicate_does_not_add_observation(self):
        h=temporal.History();a=grid();h.feed(a)
        self.assertEqual(h.feed(copy.deepcopy(a))['status'],'duplicate_stamp')
        self.assertEqual(h.unique_count,1)
    def test_conflicting_stamp_rejected(self):
        h=temporal.History();h.feed(grid())
        with self.assertRaisesRegex(ValueError,'conflicting'):h.feed(grid(data=[100]+[0]*14))
    def test_backward_stamp_rejected(self):
        h=temporal.History();h.feed(grid())
        with self.assertRaisesRegex(ValueError,'backward'):h.feed(grid(t=.9))
    def test_nonodom_initial_frame_rejected(self):
        with self.assertRaisesRegex(ValueError,'frame'):temporal.History().feed(grid(frame='map'))
    def test_frame_change_rejected(self):
        h=temporal.History();h.feed(grid())
        with self.assertRaisesRegex(ValueError,'frame'):h.feed(grid(t=1.6,frame='map'))
    def test_explicit_rotated_grid_rejected(self):
        a=grid();a['origin_yaw']=.1
        with self.assertRaisesRegex(ValueError,'orientation'):temporal.History().feed(a)
    def test_nonintegral_origin_rejected(self):
        h=temporal.History();h.feed(grid())
        with self.assertRaisesRegex(ValueError,'integral'):h.feed(grid(t=1.6,ox=.025))
    def test_resolution_change_rejected(self):
        h=temporal.History();h.feed(grid())
        with self.assertRaisesRegex(ValueError,'resolution'):h.feed(grid(t=1.6,resolution=.06))
    def test_float32_resolution_shift_tolerance(self):
        h=temporal.History();h.feed(grid(resolution=.05000000074505806))
        r=h.feed(grid(t=1.6,ox=.05,resolution=.05000000074505806))
        self.assertEqual(r['overlap_n'],12)
    def test_repeated_content_distinct_stamp_recorded(self):
        h=temporal.History();h.feed(grid());r=h.feed(grid(t=1.6))
        self.assertTrue(r['repeated_content']);self.assertEqual(h.unique_count,2)
    def test_motion_requires_two_consistent_pairs(self):
        def moving(t,j):
            d=[0]*15;d[j]=d[j+5]=100;return grid(t=t,data=d)
        h=temporal.History();h.feed(moving(1,0));r=h.feed(moving(1.6,1))
        self.assertTrue(r['motion_candidate']);self.assertFalse(r['sustained_direction'])
        self.assertAlmostEqual(r['centroid_displacement_xy'][0],.05)
        r=h.feed(moving(2.2,2));self.assertTrue(r['sustained_direction'])
        self.assertEqual(r['support_stamps'],[1.,1.6,2.2])
    def test_gap_breaks_direction(self):
        a=grid(data=[100,0,0,0,0]*3);b=grid(t=3.,data=[0,100,0,0,0]*3)
        h=temporal.History();h.feed(a);r=h.feed(b)
        self.assertTrue(r['gap']);self.assertFalse(r['motion_candidate'])
    def test_coverage_edge_reveal_is_not_motion(self):
        h=temporal.History();h.feed(grid());r=h.feed(grid(t=1.6,ox=.05,data=[0,0,0,0,100]*3))
        self.assertEqual(r['added_n'],0);self.assertEqual(r['entered_lethal_n'],3)
    def test_no_overlap_rejected(self):
        h=temporal.History();h.feed(grid())
        with self.assertRaisesRegex(ValueError,'overlap'):h.feed(grid(t=1.6,ox=2.))
    def test_roi_cannot_use_truth_fields(self):
        a=grid(data=[100]+[0]*14);b=grid(t=1.6,data=[0,100]+[0]*13)
        a['obstacle_grid_pose']=[999,999,999];b['obstacle_grid_pose']=[-999,-999,-999]
        h=temporal.History(roi=[0.,0.,.25,.15],min_changed=1);h.feed(a);r=h.feed(b)
        self.assertEqual((r['added_n'],r['removed_n']),(1,1))
        self.assertTrue(r['motion_candidate'])

class AxisEvidenceTests(unittest.TestCase):
    def test_positive_y_requires_full_cell_and_consecutive_pairs(self):
        def moved(t,j):
            data=[0]*25;data[j*5]=data[j*5+1]=100
            return dict(grid(t=t,data=data),height=5)
        h=temporal.History();h.feed(moved(1.,0));a=h.feed(moved(1.6,1))
        self.assertTrue(a.get('axis_motion_candidate',False))
        self.assertFalse(a.get('sustained_axis_direction',False))
        b=h.feed(moved(2.2,2))
        self.assertTrue(b.get('sustained_axis_direction',False))
        self.assertEqual(b.get('axis_support_stamps'),[1.,1.6,2.2])
    def test_sideways_change_is_not_positive_y_evidence(self):
        h=temporal.History();h.feed(grid(data=[100,0,0,0,0]*3))
        a=h.feed(grid(t=1.6,data=[0,100,0,0,0]*3))
        self.assertFalse(a.get('axis_motion_candidate',True))

class StreamCliTests(unittest.TestCase):
    def test_stream_override_binds_to_preceding_trial(self):
        import analyze_maps
        from pathlib import Path
        a=analyze_maps.parse_cli(['--trial','/native','--label','native','--trial','/static','--label','static','--sensor-stream','/bag.jsonl.gz'])
        self.assertEqual(a.trial,[Path('/native'),Path('/static')])
        self.assertEqual(a.sensor_stream,{1:Path('/bag.jsonl.gz')})
    def test_override_must_follow_trial(self):
        import analyze_maps
        from contextlib import redirect_stderr
        from io import StringIO
        with redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit):analyze_maps.parse_cli(['--sensor-stream','/orphan'])

class TimingTests(unittest.TestCase):
    def test_availability_uses_receipt_margin_and_source_time(self):
        rows=[dict(stamp=1.,receipt=1.,status='initial'),dict(stamp=1.6,receipt=1.6,status='pair',motion_candidate=True,sustained_direction=False)]
        a=temporal.availability(rows,1.65)
        self.assertEqual(a['available_unique_maps'],1)
        self.assertIsNone(a['motion_pair_lead_s'])
        a=temporal.availability(rows,1.75)
        self.assertEqual(a['available_unique_maps'],2)
        self.assertAlmostEqual(a['motion_pair_lead_s'],.05)
    def test_availability_does_not_count_duplicates_or_future_stamps(self):
        rows=[dict(stamp=1.,receipt=1.,status='initial'),dict(stamp=1.,receipt=1.,status='duplicate_stamp'),dict(stamp=2.,receipt=1.,status='pair')]
        self.assertEqual(temporal.availability(rows,1.5)['available_unique_maps'],1)

if __name__=='__main__':unittest.main(verbosity=2)
