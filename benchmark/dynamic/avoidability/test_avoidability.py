"""Stopping must account for full body and subsequent lateral impact."""
import importlib.util
import unittest

class AvoidabilityTests(unittest.TestCase):
    def api(self):
        self.assertIsNotNone(importlib.util.find_spec('dynamic.avoidability.kinematics'))
        from dynamic.avoidability import kinematics
        return kinematics
    def test_parking_inside_crossing_is_not_safe(self):
        k=self.api()
        self.assertFalse(k.rollout([1.5,0,0,.26,0],4,4,'brake',2.5,0,0)['safe'])
        self.assertTrue(k.rollout([1.1,0,0,.26,0],4,4,'brake',1,.15,.03)['safe'])
    def test_holding_body_must_use_rear_extent(self):
        k=self.api()
        self.assertFalse(k.rollout([2.0,0,0,0,0],7,4,'brake',2.5,0,0)['safe'])
        self.assertTrue(k.rollout([2.2,0,0,0,0],7,4,'brake',2.5,0,0)['safe'])
    def test_braking_distance_and_full_geometry(self):
        k=self.api()
        self.assertAlmostEqual(k.braking_distance(.26,2.5,.15),.05252)
        bounds=k.robot_bounds()
        self.assertLessEqual(bounds[0],-.1965)
        self.assertGreaterEqual(bounds[2],.073-1e-9)
        self.assertGreaterEqual(bounds[3],.153)
    def test_hold_and_coupled_wheel_bounds(self):
        import numpy as np
        k=self.api()
        hold=k.rollout([0,0,0,.26,.6],0,4,'hold',trace=True)['trace']
        self.assertAlmostEqual(hold['omega'][-1],.6)
        trace=k.rollout([0,0,0,.26,.6],0,4,'brake',1,.15,.03,trace=True,wheel_accel=1)['trace']
        v,w=np.array(trace['v']),np.array(trace['omega'])
        for wheel in (v-.287*w/2,v+.287*w/2):
            self.assertLessEqual(max(abs(np.diff(wheel)))/.01,1+1e-9)
    def test_probe_fixture_forwarding_capture(self):
        import json
        from pathlib import Path
        here=Path(__file__).resolve().parent
        paths=list((here/'evidence/calibration').glob('decisions_*.jsonl'))
        self.assertTrue(paths,'run calibrate_probe.sh first')
        rows=[json.loads(s) for s in paths[-1].read_text().splitlines()]
        self.assertEqual([r['threw'] for r in rows],[False,True])
        for r in rows:
            self.assertEqual(r['best_index'],0)
            self.assertEqual(len(r['costmap']['data']),400)
            self.assertEqual(set(r['costmap']['data']),{0})
            self.assertEqual(r['candidates'][0]['scores'],[.1,.2,.3,.4,.6,.5])
            self.assertEqual(r['best_poses'],[[.1,0,0]])
if __name__=='__main__':unittest.main()