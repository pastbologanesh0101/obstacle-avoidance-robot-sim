import math
import unittest

from obstacle_avoidance_robot.geometry import (
    normalize_angle,
    ray_circle_intersection,
    ray_wall_intersection,
)


class TestRayCircleIntersection(unittest.TestCase):
    def test_known_geometric_case_hits_circle(self):
        # Ray from origin along +x, circle centered at (10, 0) radius 2.
        # The ray should hit the near edge of the circle at distance 8.
        d = ray_circle_intersection(0, 0, 1, 0, 10, 0, 2)
        self.assertIsNotNone(d)
        self.assertAlmostEqual(d, 8.0, places=6)

    def test_offset_circle_known_distance(self):
        # Ray from origin along +x, circle centered at (5, 0) radius 1.5.
        d = ray_circle_intersection(0, 0, 1, 0, 5, 0, 1.5)
        self.assertAlmostEqual(d, 3.5, places=6)

    def test_ray_misses_circle_returns_none(self):
        # Circle well off to the side of the ray's path.
        d = ray_circle_intersection(0, 0, 1, 0, 10, 10, 1.0)
        self.assertIsNone(d)

    def test_circle_behind_ray_origin_returns_none(self):
        # Circle is entirely behind the ray's origin (negative t only).
        d = ray_circle_intersection(0, 0, 1, 0, -10, 0, 2)
        self.assertIsNone(d)


class TestRayWallIntersection(unittest.TestCase):
    def test_boundary_distance_along_positive_x(self):
        d = ray_wall_intersection(0, 0, 1, 0, xmin=-10, xmax=10, ymin=-10, ymax=10)
        self.assertAlmostEqual(d, 10.0, places=6)

    def test_boundary_distance_along_negative_y(self):
        d = ray_wall_intersection(0, 0, 0, -1, xmin=-10, xmax=10, ymin=-10, ymax=10)
        self.assertAlmostEqual(d, 10.0, places=6)

    def test_boundary_distance_from_off_center_origin(self):
        # Origin near one wall, ray pointing toward the far wall.
        d = ray_wall_intersection(8, 0, 1, 0, xmin=0, xmax=20, ymin=0, ymax=20)
        self.assertAlmostEqual(d, 12.0, places=6)

    def test_diagonal_ray_hits_nearest_wall(self):
        # 45-degree ray from center of a 20x20 box centered at origin should
        # exit through the corner region, distance = 10*sqrt(2).
        inv_sqrt2 = 1.0 / math.sqrt(2)
        d = ray_wall_intersection(0, 0, inv_sqrt2, inv_sqrt2, xmin=-10, xmax=10, ymin=-10, ymax=10)
        self.assertAlmostEqual(d, 10 * math.sqrt(2), places=5)


class TestNormalizeAngle(unittest.TestCase):
    def test_wraps_large_positive_angle(self):
        self.assertAlmostEqual(normalize_angle(3 * math.pi), math.pi, places=6)

    def test_wraps_large_negative_angle(self):
        self.assertAlmostEqual(normalize_angle(-3 * math.pi), math.pi, places=6)


if __name__ == "__main__":
    unittest.main()
