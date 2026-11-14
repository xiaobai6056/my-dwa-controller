"""Source-time and validity regressions for the experimental estimator."""
import unittest

import numpy as np

from pedestrian_perception.kalman import FilterConfig, KalmanHistory
from pedestrian_perception.state import StateHistory


class KalmanTests(unittest.TestCase):
    def setUp(self):
        self.raw = StateHistory()
        self.history = KalmanHistory(self.raw, FilterConfig(.025, .03, .06, .11, .35))

    def step(self, t, x=None, y=0.):
        stamp = round(t * 1e9)
        self.history.update(stamp, {} if x is None else {'p': (x, y)})
        return [row for row in self.history.snapshot(stamp) if row.position_valid]

    def test_source_time_velocity(self):
        self.assertFalse(self.step(1., 0.)[0].velocity_valid)
        self.step(1.17, .051)
        self.step(1.40, .12)
        row = self.step(1.58, .174)[0]
        self.assertTrue(row.velocity_valid)
        self.assertAlmostEqual(row.velocity[0], .3, delta=.06)
        self.assertEqual(row.last_observation_stamp, row.velocity_stamp_ns)
        self.assertTrue(np.all(np.linalg.eigvalsh(self.history.diagnostics[0]['covariance']) > 0))

    def test_reject_and_recover(self):
        self.step(1., 0.)
        self.step(1.2, 0.)
        self.step(1.4, 0.)
        self.assertEqual(self.step(1.6, .29), [])
        self.assertEqual(self.history.diagnostics[0]['event'], 'rejected')
        self.assertGreater(self.history.diagnostics[0]['nis'], 9.21)
        row = self.step(1.8, .30)[0]
        self.assertTrue(row.velocity_valid)
        self.assertEqual(self.history.diagnostics[0]['event'], 'reacquired')

    def test_gap_and_clock_reset(self):
        self.step(1., 0.)
        self.step(1.2, .04)
        self.assertEqual(self.step(1.4), [])
        self.assertFalse(self.history.snapshot(1_800_000_000, invalidate=True)[0].velocity_valid)
        self.assertFalse(self.step(1.81, .18)[0].velocity_valid)
        self.history.reset()
        self.assertFalse(self.step(.1, 3.)[0].velocity_valid)

    def test_deadband_exits_on_start(self):
        for i in range(6):
            row = self.step(1. + i * .2, .002 * (-1)**i)[0]
        self.assertEqual(row.velocity, (0., 0.))
        self.step(2.2, .055)
        row = self.step(2.4, .11)[0]
        self.assertGreater(row.velocity[0], .11)


if __name__ == '__main__':
    unittest.main()
