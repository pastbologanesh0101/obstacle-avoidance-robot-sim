import unittest

from obstacle_avoidance_robot.arena import Arena, Obstacle


class TestArenaCollision(unittest.TestCase):
    def setUp(self):
        self.arena = Arena(
            xmin=0.0,
            xmax=20.0,
            ymin=0.0,
            ymax=20.0,
            obstacles=[Obstacle(x=10.0, y=10.0, radius=2.0)],
        )

    def test_flags_collision_when_path_intersects_obstacle(self):
        # Robot center right on top of the obstacle's edge region.
        self.assertTrue(self.arena.collides(x=11.0, y=10.0, robot_radius=0.5))

    def test_no_collision_when_clear_of_obstacle(self):
        self.assertFalse(self.arena.collides(x=2.0, y=2.0, robot_radius=0.5))

    def test_collision_at_exact_touching_distance(self):
        # Distance between centers == radius + robot_radius should NOT
        # count as a collision (touching, not overlapping); just inside
        # that should.
        touching_x = 10.0 + 2.0 + 0.5  # exactly center-to-center = r_obs + r_robot
        self.assertFalse(self.arena.collides(x=touching_x, y=10.0, robot_radius=0.5))
        just_inside_x = touching_x - 0.01
        self.assertTrue(self.arena.collides(x=just_inside_x, y=10.0, robot_radius=0.5))

    def test_wall_collision_detected(self):
        self.assertTrue(self.arena.collides(x=0.1, y=10.0, robot_radius=0.5))

    def test_reached_goal_within_tolerance(self):
        self.arena.goal = (15.0, 15.0)
        self.assertTrue(self.arena.reached_goal(x=15.4, y=15.0, tolerance=0.5))
        self.assertFalse(self.arena.reached_goal(x=17.0, y=15.0, tolerance=0.5))


if __name__ == "__main__":
    unittest.main()
