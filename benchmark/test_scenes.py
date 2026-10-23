"""Check one geometric source drives generated physics obstacles and map cells."""
import importlib.util
from pathlib import Path
import tempfile
import unittest


class SceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).with_name('scenes.py')
        cls.m = None
        if path.exists():
            spec = importlib.util.spec_from_file_location('scenes', path)
            cls.m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.m)

    def test_generated_obstacles_cover_exactly_occupied_map_cells(self):
        self.assertIsNotNone(self.m, 'scene resources are not implemented')
        for name in ('S02', 'S03', 'S05'):
            with self.subTest(scene=name), tempfile.TemporaryDirectory() as directory:
                scene = self.m.load_scene(name)
                data = self.m.generate(scene, Path(directory))
                ox, oy = data['origin']
                for row, values in enumerate(data['occupied']):
                    for column, occupied in enumerate(values):
                        x, y = ox+(column+.5)*.05, oy+(row+.5)*.05
                        covered = any(a <= x <= c and b <= y <= d
                                      for a, b, c, d in data['obstacle_rectangles'])
                        self.assertEqual(covered, occupied)
                for point in (scene['start'], scene['goal']):
                    self.assertFalse(any(a <= point[0] <= c and b <= point[1] <= d
                                         for a, b, c, d in data['obstacle_rectangles']))

    def test_wall_interiors_are_unknown_not_likelihood_field_zero_distance(self):
        self.assertIsNotNone(self.m, 'scene resources are not implemented')
        with tempfile.TemporaryDirectory() as directory:
            self.m.generate(self.m.load_scene('S02'), Path(directory))
            pixels = (Path(directory) / 'map.pgm').read_bytes().split(b'\n', 3)[3]
            self.assertEqual(set(pixels), {0, 205, 254})


if __name__ == '__main__':
    unittest.main()
