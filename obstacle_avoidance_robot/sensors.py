"""Simulated ultrasonic-style range sensors.

Each sensor is a ray cast from the robot's current position, at a fixed
angular offset from the robot's heading, that reports the true geometric
distance to the nearest obstacle or arena wall, capped at `max_range`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

from .arena import Arena
from .geometry import ray_circle_intersection, ray_wall_intersection
from .robot import Robot


@dataclass(frozen=True)
class Sensor:
    name: str
    angle_offset: float  # radians, relative to robot heading
    max_range: float = 6.0


# A standard 5-sensor array: front, front-left/right diagonals, and side sensors.
DEFAULT_SENSOR_ARRAY: List[Sensor] = [
    Sensor("front", 0.0),
    Sensor("front_left", math.radians(35)),
    Sensor("front_right", math.radians(-35)),
    Sensor("left", math.radians(90)),
    Sensor("right", math.radians(-90)),
]


def sense(sensor: Sensor, robot: Robot, arena: Arena) -> float:
    """Return the distance this sensor reads at the robot's current pose."""
    angle = robot.heading + sensor.angle_offset
    dx, dy = math.cos(angle), math.sin(angle)

    best = sensor.max_range

    for obs in arena.obstacles:
        d = ray_circle_intersection(robot.x, robot.y, dx, dy, obs.x, obs.y, obs.radius)
        if d is not None and d < best:
            best = d

    wall_d = ray_wall_intersection(
        robot.x, robot.y, dx, dy, arena.xmin, arena.xmax, arena.ymin, arena.ymax
    )
    if wall_d is not None and wall_d < best:
        best = wall_d

    return min(best, sensor.max_range)


def read_all(
    robot: Robot, arena: Arena, sensors: List[Sensor] = None
) -> Dict[str, float]:
    """Return a {sensor_name: distance} dict for the given sensor array
    (defaults to DEFAULT_SENSOR_ARRAY).
    """
    if sensors is None:
        sensors = DEFAULT_SENSOR_ARRAY
    return {s.name: sense(s, robot, arena) for s in sensors}
