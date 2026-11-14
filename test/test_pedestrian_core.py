"""Exercise frame association, position geometry and causal track history."""

import unittest

from pedestrian_perception.association import Beam, associate, estimate_positions
from pedestrian_perception.state import StateHistory

MS = 1_000_000


class AssociationTest(unittest.TestCase):
    """Check R2 A subclusters and conservative overlap rejection."""

    def test_small_clusters_combine_for_one_track(self):
        """Two legs may contribute two returns each to one observation."""
        beams = [Beam(1, (1., 0.), ('1',)), Beam(2, (1.03, 0.), ('1',)),
                 Beam(6, (1.01, .30), ('1',)), Beam(7, (1.02, .30), ('1',))]
        groups = associate(beams, {})
        self.assertEqual([group.beams for group in groups], [(1, 2), (6, 7)])
        self.assertEqual([group.track_id for group in groups], ['1', '1'])
        self.assertEqual(estimate_positions(beams, groups), {'1': (1.0150000000000001, .15)})

    def test_ipm_only_rejects_candidates_in_overlap(self):
        """Assign sole candidates, require complete and decisive IPM in overlaps."""
        beams = [Beam(1, (1., 0.), ('1', '2')), Beam(2, (1.04, 0.), ('1', '2')),
                 Beam(3, (1.05, 0.), ('2',)), Beam(4, (1.06, 0.), ())]
        self.assertEqual([g.track_id for g in associate(beams, {'1': (1., 0.)})],
                         [None, '2', None])
        self.assertEqual([g.track_id for g in associate(beams, {'1': (1., 0.), '2': (3., 0.)})],
                         ['1', '2', None])
        self.assertIsNone(associate(beams[:2], {'1': (1., 0.), '2': (1.4, 0.)})[0].track_id)
        self.assertIsNone(associate(beams[:2], {'1': (0., 0.), '2': (3., 0.)})[0].track_id)

    def test_trimmed_centroid_uses_the_same_pool_after_validation(self):
        points = [(0., .04), (.01, .03), (.02, .02), (.03, .01), (.30, 0.)]
        beams = [Beam(i, point, ('1',)) for i, point in enumerate(points)]
        groups = associate(beams, {})
        self.assertEqual([group.beams for group in groups], [(0, 1, 2, 3), (4,)])
        self.assertEqual(estimate_positions(beams, groups), {'1': (.15, .02)})
        trimmed = estimate_positions(beams, groups, estimator='trimmed_centroid')
        self.assertAlmostEqual(trimmed['1'][0], .02)
        self.assertAlmostEqual(trimmed['1'][1], .02)

        small = beams[:3]
        trimmed = estimate_positions(small, associate(small, {}), estimator='trimmed_centroid')
        self.assertAlmostEqual(trimmed['1'][0], .01)
        self.assertAlmostEqual(trimmed['1'][1], .03)

        distant = beams[:-1] + [Beam(4, (.8, 0.), ('1',))]
        groups = associate(distant, {})
        self.assertEqual(estimate_positions(distant, groups, estimator='trimmed_centroid'), {})

    def test_background_span_rejects_entire_measurement(self):
        """A detached background return must invalidate the pooled target."""
        beams = [Beam(1, (1., 0.), ('1',)), Beam(2, (1.03, 0.), ('1',)),
                 Beam(3, (1.01, .04), ('1',)), Beam(7, (1.02, .70), ('1',))]
        self.assertEqual(estimate_positions(beams, associate(beams, {})), {})


class StateTest(unittest.TestCase):
    """Check continuity and source-time differentiation."""

    def test_window_uses_irregular_times_and_is_per_id(self):
        """Require window coverage and use only observed source times."""
        state = StateHistory()
        for ms in (1000, 1190, 1405):
            t = ms / 1000
            state.update(ms * MS, {'1': (2*t, -3*t)})
        one, = state.snapshot(1435 * MS)
        self.assertTrue(one.position_valid)
        self.assertAlmostEqual(one.velocity[0], 2.)
        self.assertAlmostEqual(one.velocity[1], -3.)
        self.assertEqual(one.sample_stamps, (1000 * MS, 1190 * MS, 1405 * MS))
        self.assertEqual(one.observation_age_ns, 30 * MS)

    def test_missing_observation_and_recovery(self):
        """Preserve the measurement and restart velocity on a miss or scan gap."""
        state = StateHistory()
        for ms in (1000, 1200, 1400):
            state.update(ms * MS, {'1': (ms / 1000, 0.)})
        state.update(1600 * MS, {})
        row, = state.snapshot(1650 * MS)
        self.assertEqual(row.position, (1.4, 0.))
        self.assertEqual(row.last_observation_stamp, 1400 * MS)
        self.assertFalse(row.position_valid)
        self.assertTrue(row.velocity_held)
        self.assertEqual(row.sample_stamps, (1000 * MS, 1200 * MS, 1400 * MS))
        state.update(1800 * MS, {'1': (1.8, 0.)})
        row, = state.snapshot(1810 * MS)
        self.assertTrue(row.position_valid)
        self.assertFalse(row.direct_valid)
        state.update(2200 * MS, {'1': (2.05, 0.)})
        row, = state.snapshot(2210 * MS)
        self.assertFalse(row.direct_valid)
        self.assertFalse(state.snapshot(2800 * MS)[0].position_valid)

    def test_duplicate_and_out_of_order_scan_do_not_mutate_state(self):
        """Older and duplicate scans cannot overwrite a newer observation."""
        state = StateHistory()
        state.update(1200 * MS, {'1': (1., 0.)})
        for ms in (1200, 1100):
            state.update(ms * MS, {'1': (5., 0.)})
        self.assertEqual(state.snapshot(1200 * MS)[0].position, (1., 0.))


class CrossingTest(unittest.TestCase):
    """Exercise motion identity and sample provenance independently of SORT."""

    def setUp(self):
        """Seed two opposing trajectories with a valid fit for each."""
        self.state = StateHistory()
        for ms, distance in ((1000, .9), (1200, .6), (1400, .3)):
            self.observe(ms, {'a': (-distance, 0.), 'b': (distance, 0.)})
        self.left, self.right = self.state.snapshot(1400 * MS)
        self.assertGreater(self.left.velocity[0], 0.)
        self.assertLess(self.right.velocity[0], 0.)

    def observe(self, ms, positions):
        """Commit one scan and index the resulting states by fused ID."""
        self.state.update(ms * MS, positions)
        return {row.track_id: row for row in self.state.snapshot(ms * MS)}

    def check_crossing(self, swapped, overlap):
        """Verify conservative hold and independent recovery on real samples."""
        rows = self.observe(1600, overlap)
        for before in (self.left, self.right):
            held = rows[before.track_id]
            self.assertFalse(held.position_valid)
            self.assertTrue(held.velocity_valid)
            self.assertEqual(held.position, before.position)
            self.assertEqual(held.last_observation_stamp, 1400 * MS)
            self.assertEqual(held.sample_stamps, before.sample_stamps)
        a, b = ('b', 'a') if swapped else ('a', 'b')
        for ms, distance in ((1800, .3), (2000, .6), (2200, .9)):
            rows = self.observe(ms, {a: (distance, 0.), b: (-distance, 0.)})
            for before, sort_id, sign in ((self.left, a, 1), (self.right, b, -1)):
                row = rows[before.track_id]
                self.assertTrue(row.position_valid)
                self.assertEqual(row.position, (sign * distance, 0.))
                self.assertEqual(row.sort_id, sort_id)
                self.assertEqual(row.velocity_valid, ms == 2200)
                self.assertEqual(row.direct_valid, ms >= 2000)
                if ms == 1800:
                    self.assertEqual(row.velocity_stamp_ns, 1400 * MS)
                if row.direct_valid:
                    self.assertAlmostEqual(row.direct[0], sign * 1.5)
                if row.velocity_valid:
                    self.assertEqual(row.sample_stamps, (1800 * MS, 2000 * MS, 2200 * MS))
                    self.assertAlmostEqual(row.velocity[0], sign * 1.5)
        self.assertEqual(len(rows), 2)

    def test_crossing_with_unchanged_sort_ids(self):
        """Reject both position candidates while both histories predict the overlap."""
        self.check_crossing(False, {'a': (-.02, 0.), 'b': (.02, 0.)})

    def test_crossing_with_explicit_sort_id_exchange(self):
        """Reattach by motion after an injected exchange of the two visual labels."""
        self.check_crossing(True, {'b': (0., 0.)})

    def test_crossing_with_short_lidar_dropout(self):
        """Recover clean segments across a scan without usable returns."""
        self.check_crossing(False, {})

    def test_unmatched_observation_starts_clean_history(self):
        """A reused SORT label outside the position gate can start a fresh history."""
        rows = self.observe(1600, {'a': (2., 0.)})
        newborn, = [row for row in rows.values() if row.position_valid]
        self.assertNotEqual(newborn.track_id, self.left.track_id)
        self.assertFalse(newborn.direct_valid)
        self.assertFalse(newborn.velocity_valid)
        self.assertFalse(rows[self.left.track_id].position_valid)
        rows = self.observe(1800, {'a': (2.2, 0.)})
        self.assertTrue(rows[newborn.track_id].direct_valid)
        self.assertFalse(rows[newborn.track_id].velocity_valid)

    def test_hold_expires_without_erasing_velocity_or_timestamp(self):
        """Retain the previous estimate as invalid after the existing age limit."""
        rows = self.observe(2000, {})
        stale = rows[self.left.track_id]
        self.assertFalse(stale.velocity_valid)
        self.assertEqual(stale.velocity, self.left.velocity)
        self.assertEqual(stale.velocity_stamp_ns, 1400 * MS)
        self.assertEqual(stale.last_observation_stamp, 1400 * MS)

    def test_competing_observations_do_not_claim_one_history(self):
        """Two plausible observations cannot both update or clone a history."""
        state = StateHistory()
        state.update(1000 * MS, {'a': (0., 0.)})
        initial, = state.snapshot(1000 * MS)
        state.update(1200 * MS, {'a': (-.15, 0.), 'b': (.15, 0.)})
        held, = state.snapshot(1200 * MS)
        self.assertEqual(held.track_id, initial.track_id)
        self.assertFalse(held.position_valid)
        self.assertEqual(held.last_observation_stamp, 1000 * MS)

    def test_motion_support_requires_a_clear_distance_lead(self):
        """Use direction and speed only to support a closer in-gate history."""
        state = StateHistory()
        for ms, x in ((1000, -.6), (1200, -.4), (1400, -.2)):
            state.update(ms * MS, {'a': (x, 0.), 'b': (.22, 0.)})
        moving, parked = state.snapshot(1400 * MS)
        state.update(1600 * MS, {'b': (.03, 0.)})
        measured, held = state.snapshot(1600 * MS)
        self.assertEqual(measured.track_id, moving.track_id)
        self.assertEqual(measured.sort_id, 'b')
        self.assertTrue(measured.position_valid)
        self.assertEqual(held.track_id, parked.track_id)
        self.assertFalse(held.position_valid)

    def test_small_opposite_step_preserves_the_ols_window(self):
        """Centimeter-scale motion around rest cannot confirm a reversal."""
        state = StateHistory()
        for ms, x in ((1000, 0.), (1200, .02), (1400, .04), (1600, .026)):
            state.update(ms * MS, {'a': (x, 0.)})
        row, = state.snapshot(1600 * MS)
        self.assertTrue(row.velocity_valid)
        self.assertEqual(row.sample_stamps, (1200 * MS, 1400 * MS, 1600 * MS))

    def test_reversal_discards_the_old_direction(self):
        """A trusted reversal reports a negative difference and rebuilds OLS."""
        state = StateHistory()
        for ms, x in ((1000, 0.), (1200, .12), (1400, .24),
                      (1600, .12), (1800, 0.)):
            state.update(ms * MS, {'a': (x, 0.)})
            row, = state.snapshot(ms * MS)
            if ms == 1600:
                self.assertFalse(row.velocity_valid)
                self.assertTrue(row.direct_valid)
                self.assertAlmostEqual(row.direct[0], -.6)
        self.assertTrue(row.velocity_valid)
        self.assertEqual(row.sample_stamps, (1400 * MS, 1600 * MS, 1800 * MS))
        self.assertAlmostEqual(row.velocity[0], -.6)


if __name__ == '__main__':
    unittest.main()
