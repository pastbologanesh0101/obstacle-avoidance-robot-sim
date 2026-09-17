"""Differential-drive robot state and kinematics (2D simulation only)."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .geometry import clamp, normalize_angle


@dataclass
class Robot:
    x: float
    y: float
    heading: float  # radians, 0 = pointing along +x axis, CCW positive
    radius: float = 0.5

    @property
    def position(self):
        return (self.x, self.y)

    def step(
        self,
        turn: float,
        speed: float,
        dt: float = 1.0,
        max_turn_rate: float = math.radians(25),
        max_speed: float = 1.0,
    ) -> None:
        """Advance the robot's simplified differential-drive kinematics by
        one timestep. `turn` is the desired heading change (rad) for this
        step, `speed` is the desired forward speed (units/step); both are
        clamped to the robot's physical limits before being applied.
        """
        turn = clamp(turn, -max_turn_rate, max_turn_rate)
        speed = clamp(speed, 0.0, max_speed)

        self.heading = normalize_angle(self.heading + turn * dt)
        self.x += speed * math.cos(self.heading) * dt
        self.y += speed * math.sin(self.heading) * dt
