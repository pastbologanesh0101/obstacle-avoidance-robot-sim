import math
import unittest

from obstacle_avoidance_robot.arena import Arena, Obstacle
from obstacle_avoidance_robot.robot import Robot
from obstacle_avoidance_robot.sensors import Sensor, sense


class TestSensors(unittest.TestCase):
    def setUp(self):
        self.arena = Arena(
            xmin=-50.0, xmax=50.0, ymin=-50.0, ymax=50.0, obstacles=[]
        )
        self.front_sensor = Sensor("front", 0.0, max_range=6.0)

    def test_clear_path_reports_max_range(self):
        # Huge open arena, no obstacles anywhere near the sensor's max range.
        robot = Robot(x=0.0, y=0.0, heading=0.0)
        reading = sense(self.front_sensor, robot, self.arena)
        self.assertAlmostEqual(reading, self.front_sensor.max_range, places=6)

    def test_sensor_pointed_directly_at_nearby_obstacle(self):
        self.arena.obstacles = [Obstacle(x=3.0, y=0.0, radius=1.0)]
        robot = Robot(x=0.0, y=0.0, heading=0.0)
        reading = sense(self.front_sensor, robot, self.arena)
        # Nearest edge of the obstacle is at distance 3 - 1 = 2.
        self.assertAlmostEqual(reading, 2.0, places=6)

    def test_sensor_ignores_obstacle_outside_its_ray(self):
        # Obstacle is directly behind the robot (heading 0, obstacle at -x);
        # the forward-pointing sensor should not see it at all.
        self.arena.obstacles = [Obstacle(x=-3.0, y=0.0, radius=1.0)]
        robot = Robot(x=0.0, y=0.0, heading=0.0)
        reading = sense(self.front_sensor, robot, self.arena)
        self.assertAlmostEqual(reading, self.front_sensor.max_range, places=6)

    def test_sensor_capped_at_max_range_even_if_wall_further(self):
        big_arena = Arena(xmin=-1000, xmax=1000, ymin=-1000, ymax=1000, obstacles=[])
        robot = Robot(x=0.0, y=0.0, heading=0.0)
        reading = sense(self.front_sensor, robot, big_arena)
        self.assertEqual(reading, self.front_sensor.max_range)

    def test_sensor_respects_robot_heading_offset(self):
        # Obstacle to the "left" of a robot facing +y should be seen by a
        # sensor angled -90 degrees relative to heading (i.e. pointing +x
        # when heading is +y... construct concretely below).
        self.arena.obstacles = [Obstacle(x=0.0, y=3.0, radius=1.0)]
        robot = Robot(x=0.0, y=0.0, heading=math.radians(90))  # facing +y
        forward_sensor = Sensor("front", 0.0, max_range=6.0)
        reading = sense(forward_sensor, robot, self.arena)
        self.assertAlmostEqual(reading, 2.0, places=6)


if __name__ == "__main__":
    unittest.main()
