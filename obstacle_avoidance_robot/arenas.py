"""Bundled test arenas of varying obstacle density/layout.

Each function returns a *fresh* Arena instance (so running the demo or the
test suite repeatedly never mutates shared state).
"""

from __future__ import annotations

import math

from .arena import Arena, Obstacle


def make_easy_arena() -> Arena:
    """A large, mostly-open arena with a single obstacle well off the
    direct start->goal line. Should be an easy, near-collision-free run.
    """
    return Arena(
        xmin=0.0,
        xmax=30.0,
        ymin=0.0,
        ymax=20.0,
        obstacles=[
            Obstacle(x=15.0, y=15.0, radius=2.0),
        ],
        start=(2.0, 2.0),
        start_heading=math.radians(20),
        goal=(28.0, 3.0),
        name="easy_open_field",
    )


def make_medium_arena() -> Arena:
    """A handful of obstacles roughly along the straight-line path from
    start to goal, forcing genuine avoidance maneuvers.
    """
    return Arena(
        xmin=0.0,
        xmax=30.0,
        ymin=0.0,
        ymax=20.0,
        obstacles=[
            Obstacle(x=10.0, y=10.0, radius=2.0),
            Obstacle(x=18.0, y=8.0, radius=2.0),
            Obstacle(x=22.0, y=13.0, radius=1.5),
            Obstacle(x=14.0, y=15.0, radius=1.5),
        ],
        start=(2.0, 10.0),
        start_heading=0.0,
        goal=(28.0, 10.0),
        name="medium_scattered",
    )


def make_hard_arena() -> Arena:
    """A dense obstacle field with a deliberately narrow "gate" (a pair of
    obstacles with only a 1.5-unit gap between them -- the robot is 1.0
    units in diameter) directly on the start->goal line.

    This is intentionally hard enough that the bundled reactive controller
    is NOT guaranteed to get through: it genuinely collides here more
    often than not. That is reported honestly by the demo/README rather
    than tuned away, since a real reactive (no global planning) controller
    has exactly this kind of limitation.
    """
    gap = 1.5
    gate_radius = 2.5
    gate_half_sep = (gap + 2 * gate_radius) / 2.0
    center_y = 12.0

    return Arena(
        xmin=0.0,
        xmax=30.0,
        ymin=0.0,
        ymax=24.0,
        obstacles=[
            Obstacle(x=8.0, y=6.0, radius=1.5),
            Obstacle(x=8.0, y=18.0, radius=1.5),
            Obstacle(x=14.0, y=center_y - gate_half_sep, radius=gate_radius),
            Obstacle(x=14.0, y=center_y + gate_half_sep, radius=gate_radius),
            Obstacle(x=22.0, y=7.0, radius=1.5),
            Obstacle(x=22.0, y=17.0, radius=1.5),
        ],
        start=(2.0, center_y),
        start_heading=0.0,
        goal=(28.0, center_y),
        name="hard_gauntlet",
    )


BUNDLED_ARENAS = {
    "easy": make_easy_arena,
    "medium": make_medium_arena,
    "hard": make_hard_arena,
}
