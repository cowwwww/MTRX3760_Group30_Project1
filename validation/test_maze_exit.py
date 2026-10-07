import math
import unittest
from maze_exit import EastExitDetector


class MazeExitTests(unittest.TestCase):
    def test_can_turn_outside_after_crossing_opening(self):
        detector = EastExitDetector()
        for point in [(-2.0, -1.3), (2.4, 1.0), (2.5, 1.0)]:
            self.assertFalse(detector.update(*point))
        self.assertTrue(detector.update(2.61, 0.73))

    def test_crossing_wall_or_starting_outside_is_not_exit(self):
        for y in [-0.4, 1.8]:
            detector = EastExitDetector()
            self.assertFalse(detector.update(2.3, y))
            self.assertFalse(detector.update(2.7, y))
            self.assertFalse(detector.update(2.8, 1.0))
        self.assertFalse(EastExitDetector().update(3.0, 1.0))

    def test_coarse_sample_crossing_uses_portal_intersection(self):
        detector = EastExitDetector()
        self.assertFalse(detector.update(2.3, 0.9))
        self.assertTrue(detector.update(2.7, 0.7))

    def test_nonfinite_input_does_not_create_crossing(self):
        detector = EastExitDetector()
        self.assertFalse(detector.update(2.3, 1.0))
        self.assertFalse(detector.update(math.nan, 1.0))
        self.assertTrue(detector.update(2.7, 1.0))


if __name__ == '__main__':
    unittest.main(verbosity=2)
