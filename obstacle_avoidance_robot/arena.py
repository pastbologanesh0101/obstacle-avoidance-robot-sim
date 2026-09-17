"""The 2D world: bounded rectangular arena with circular obstacles."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

from .geometry import distance


@dataclass
class Obstacle:
    x: float
    y: float
    radius: float


@dataclass
class Arena:
    """A bounded rectangular arena. Walls are the arena boundary itself,
    obstacles are circular objects placed inside it.
    """

    xmin: float
    xmax: float
    ymin: float
    ymax: float
    obstacles: List[Obstacle] = field(default_factory=list)
    start: Tuple[float, float] = (0.0, 0.0)
    start_heading: float = 0.0
    goal: Tuple[float, float] = (0.0, 0.0)
    name: str = "arena"

    def contains_point(self, x: float, y: float) -> bool:
        return self.xmin <= x <= self.xmax and self.ymin <= y <= self.ymax

    def distance_to_nearest_wall(self, x: float, y: float) -> float:
        return min(x - self.xmin, self.xmax - x, y - self.ymin, self.ymax - y)

    def collides(self, x: float, y: float, robot_radius: float) -> bool:
        """True if a circular robot centered at (x, y) with the given
        radius overlaps any obstacle or has gone past/through a wall.
        """
        for obs in self.obstacles:
            if distance(x, y, obs.x, obs.y) < robot_radius + obs.radius:
                return True
        if self.distance_to_nearest_wall(x, y) < robot_radius:
            return True
        return False

    def reached_goal(self, x: float, y: float, tolerance: float) -> bool:
        return distance(x, y, self.goal[0], self.goal[1]) <= tolerance
