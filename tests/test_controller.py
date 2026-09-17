import unittest

from obstacle_avoidance_robot.controller import ReactiveController
from obstacle_avoidance_robot.robot import Robot


class TestReactiveController(unittest.TestCase):
    def setUp(self):
        self.controller = ReactiveController()
        self.robot = Robot(x=0.0, y=0.0, heading=0.0)
        self.goal = (20.0, 0.0)

    def test_no_turn_when_path_is_clear_and_goal_is_ahead(self):
        readings = {"front": 6.0, "front_left": 6.0, "front_right": 6.0, "left": 6.0, "right": 6.0}
        turn, speed = self.controller.compute_from_readings(self.robot, self.goal, readings)
        self.assertAlmostEqual(turn, 0.0, places=6)
        self.assertGreater(speed, 0.0)

    def test_steers_left_away_from_obstacle_detected_on_the_right(self):
        # Obstacle close on the front-right ray only -> controller should
        # turn left (positive turn, CCW) to steer away from it.
        readings = {"front": 6.0, "front_left": 6.0, "front_right": 1.2, "left": 6.0, "right": 6.0}
        turn, _ = self.controller.compute_from_readings(self.robot, self.goal, readings)
        self.assertGreater(turn, 0.0)

    def test_steers_right_away_from_obstacle_detected_on_the_left(self):
        # Mirror image of the above: obstacle close on the front-left ray
        # only -> controller should turn right (negative turn).
        readings = {"front": 6.0, "front_left": 1.2, "front_right": 6.0, "left": 6.0, "right": 6.0}
        turn, _ = self.controller.compute_from_readings(self.robot, self.goal, readings)
        self.assertLess(turn, 0.0)

    def test_slows_down_when_obstacle_close_ahead(self):
        clear_readings = {"front": 6.0, "front_left": 6.0, "front_right": 6.0, "left": 6.0, "right": 6.0}
        close_readings = {"front": 0.5, "front_left": 6.0, "front_right": 6.0, "left": 6.0, "right": 6.0}
        _, speed_clear = self.controller.compute_from_readings(self.robot, self.goal, clear_readings)
        _, speed_close = self.controller.compute_from_readings(self.robot, self.goal, close_readings)
        self.assertLess(speed_close, speed_clear)


if __name__ == "__main__":
    unittest.main()
