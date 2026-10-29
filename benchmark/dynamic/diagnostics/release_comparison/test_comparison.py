"""Build selection and runtime library identity regression tests."""
import importlib.util
from pathlib import Path
import os

import unittest


# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

class ComparisonTests(unittest.TestCase):
    def api(self):
        name='dynamic.diagnostics.release_comparison.settings'
        self.assertIsNotNone(importlib.util.find_spec(name),'variant selector must exist')
        from dynamic.diagnostics.release_comparison import settings
        return settings

    def test_selected_prefix_is_independent_and_matches_variant(self):
        api=self.api()
        for variant in ('unoptimized','release'):
            prefix,plugin=api.selection(variant)
            self.assertEqual(prefix.name,variant)
            self.assertEqual(plugin,prefix/'lib/libmy_dwa_controller.so')
            self.assertNotEqual(prefix,(ARCHIVE_WS / 'install/my_dwa_controller'))
        with self.assertRaises(ValueError):api.selection('debug')

    def test_mapping_requires_exact_selected_plugin_without_second_copy(self):
        api=self.api()
        _,release=api.selection('release')
        _,unopt=api.selection('unoptimized')
        def line(p):return f'0000-1000 r-xp 0 00:00 1 {p}\n'
        self.assertTrue(api.mapping_matches(line(release)+line(release),release))
        self.assertFalse(api.mapping_matches(line(unopt),release))
        self.assertFalse(api.mapping_matches(line(release)+line(unopt),release))
        self.assertFalse(api.mapping_matches(line(str(release)+' (deleted)'),release))
        self.assertFalse(api.mapping_matches('',release))

class ResultContractTests(unittest.TestCase):
    api=ComparisonTests.api
    def test_recorded_collision_exit_two_is_completed_observation(self):
        api=self.api()
        self.assertTrue(hasattr(api,'completed_trial'),'navigation exit contract must be recognized')
        summary=dict(valid=True,status='finished',action_accepted=True,action_result='CANCELED',action_error=None,error=None)
        self.assertTrue(api.completed_trial(2,summary))
        self.assertFalse(api.completed_trial(1,summary))
        self.assertFalse(api.completed_trial(2,dict(summary,valid=False)))
        self.assertFalse(api.completed_trial(2,dict(summary,action_error='client error')))
        self.assertFalse(api.completed_trial(2,dict(summary,action_accepted=False)))
        self.assertFalse(api.completed_trial(None,summary))

if __name__=='__main__':unittest.main()