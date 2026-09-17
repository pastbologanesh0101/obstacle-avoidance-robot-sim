"""Reactive obstacle-avoidance controller.

Uses a classic artificial-potential-fields approach:

- An *attractive* vector pulls the robot toward the goal.
- A *repulsive* vector is built from each range sensor reading that falls
  within an "influence range" -- the closer the reading, the stronger the
  push directly away from the ray direction that produced it.
- The two vectors are summed; the robot steers (turns) toward the
  resulting heading, clipped to its maximum turn rate, and slows down the
  closer the front sensor gets to an obstacle.

This is intentionally simple/stdlib-only so it is easy to read, test, and
(if someone wanted to) port to a microcontroller loop driving a real
differential-drive chassis.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from .arena import Arena
from .geometry import normalize_angle
from .robot import Robot
from .sensors import DEFAULT_SENSOR_ARRAY, Sensor, read_all


@dataclass
class ControllerParams:
    influence_range: float = 4.0  # sensor readings beyond this are ignored for avoidance
    k_attractive: float = 1.0
    k_repulsive: float = 6.0
    max_turn_rate: float = math.radians(25)
    max_speed: float = 1.0
    min_speed: float = 0.15
    slow_down_distance: float = 3.0  # front distance at which the robot starts braking


class ReactiveController:
    """Potential-fields based reactive controller."""

    def __init__(
        self,
        sensors: List[Sensor] = None,
        params: ControllerParams = None,
    ) -> None:
        self.sensors = sensors if sensors is not None else DEFAULT_SENSOR_ARRAY
        self.params = params if params is not None else ControllerParams()

    def compute_from_readings(
        self,
        robot: Robot,
        goal: Tuple[float, float],
        readings: Dict[str, float],
    ) -> Tuple[float, float]:
        """Core control law, given sensor readings. Split out from
        `compute()` so it's easy to unit test with hand-crafted readings.

        Returns (turn, speed) -- turn is a heading delta (radians) for this
        step, speed is the forward speed (units/step), both already meant
        to be passed straight into Robot.step().
        """
        p = self.params

        # Attractive component: unit vector toward the goal, scaled by k_att.
        gx, gy = goal
        to_goal_x, to_goal_y = gx - robot.x, gy - robot.y
        dist_to_goal = math.hypot(to_goal_x, to_goal_y)
        if dist_to_goal < 1e-9:
            attractive_x, attractive_y = 0.0, 0.0
        else:
            attractive_x = p.k_attractive * to_goal_x / dist_to_goal
            attractive_y = p.k_attractive * to_goal_y / dist_to_goal

        # Repulsive component: one term per sensor whose reading is inside
        # the influence range, pointing directly away from that ray.
        repulsive_x, repulsive_y = 0.0, 0.0
        for sensor in self.sensors:
            d = readings.get(sensor.name, sensor.max_range)
            if d >= p.influence_range:
                continue
            d_eff = max(d, 1e-3)
            angle = robot.heading + sensor.angle_offset
            ray_x, ray_y = math.cos(angle), math.sin(angle)
            # Standard potential-field repulsion magnitude, growing sharply
            # as d -> 0 and vanishing at d == influence_range.
            strength = p.k_repulsive * (1.0 / d_eff - 1.0 / p.influence_range) / (d_eff * d_eff)
            repulsive_x += -ray_x * strength
            repulsive_y += -ray_y * strength

        net_x = attractive_x + repulsive_x
        net_y = attractive_y + repulsive_y

        if abs(net_x) < 1e-9 and abs(net_y) < 1e-9:
            turn = 0.0
        else:
            desired_heading = math.atan2(net_y, net_x)
            heading_error = normalize_angle(desired_heading - robot.heading)
            turn = max(-p.max_turn_rate, min(p.max_turn_rate, heading_error))

        # Brake as the front sensor closes in on an obstacle.
        front_distance = readings.get("front", self.sensors[0].max_range)
        if front_distance >= p.slow_down_distance:
            speed = p.max_speed
        else:
            fraction = max(0.0, front_distance / p.slow_down_distance)
            speed = p.min_speed + (p.max_speed - p.min_speed) * fraction

        return turn, speed

    def compute(self, robot: Robot, arena: Arena) -> Tuple[float, float]:
        readings = read_all(robot, arena, self.sensors)
        return self.compute_from_readings(robot, arena.goal, readings)
