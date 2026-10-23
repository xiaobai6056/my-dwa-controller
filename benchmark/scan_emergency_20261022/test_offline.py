"""Regression against native outputs from the unchanged original Scan/TF recording."""
import csv
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent


class RecordedEmergencyRegression(unittest.TestCase):
    """Keep motion validity separate from geometric clearance and control state."""

    @classmethod
    def setUpClass(cls):
        """Load every replay result; do not filter by estimated direction."""
        with (HERE / 'replay_final_risk.csv').open() as stream:
            cls.risk = list(csv.DictReader(stream))
        with (HERE / 'replay_final_tracks.csv').open() as stream:
            cls.tracks = list(csv.DictReader(stream))

    def test_real_transverse_trend_remains(self):
        """Known +Y crossing remains observable before the visible-face switch."""
        rows = [r for r in self.tracks if 2.8 < float(r['elapsed']) < 4.2
                and abs(float(r['x'])-1.6) < .3 and r['motion'] == 'moving']
        self.assertGreaterEqual(len(rows), 3)
        self.assertTrue(all(.12 < float(r['vy']) < .35 for r in rows))

    def test_visible_face_drift_is_not_reliable_motion(self):
        """Large raw vx from shrinking visible support must be explicitly unknown."""
        rows = [r for r in self.tracks if 5 < float(r['elapsed']) < 6
                and abs(float(r['x'])-1.6) < .4 and abs(float(r['vx'])) > .15]
        self.assertTrue(rows)
        self.assertTrue(all(r['motion'] == 'unknown' for r in rows))

    def test_observed_local_clearance_releases_protection(self):
        """The old object ID need not survive once the swept region is observed clear."""
        clear = [r for r in self.risk if r['clear'] == '1']
        released = [r for r in self.risk if r['release'] == '1']
        self.assertTrue(clear)
        self.assertTrue(released)
        self.assertGreater(float(clear[0]['elapsed']), 8)
        self.assertLess(float(released[0]['elapsed']), 15)
        self.assertEqual(self.risk[-1]['state'], 'normal')

    def test_late_static_scene_does_not_create_new_transverse_risk(self):
        """The stationary post-crossing scene creates no new scan prediction."""
        rows = [r for r in self.risk if float(r['elapsed']) > 12]
        self.assertGreater(len(rows), 200)
        self.assertTrue(all(r['predicted'] == '0' for r in rows))


if __name__ == '__main__':
    unittest.main(verbosity=2)
