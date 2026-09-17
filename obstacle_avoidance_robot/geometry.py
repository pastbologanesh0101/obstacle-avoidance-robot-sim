"""Ray-casting geometry helpers used by the simulated distance sensors.

All functions work in a plain 2D Cartesian plane (no external dependencies).
Rays are defined by an origin point ``(ox, oy)`` and a *unit* direction
vector ``(dx, dy)``.
"""

from __future__ import annotations

import math
from typing import Optional

EPSILON = 1e-9


def normalize_angle(angle: float) -> float:
    """Wrap an angle (radians) into the range (-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def ray_circle_intersection(
    ox: float, oy: float, dx: float, dy: float, cx: float, cy: float, r: float
) -> Optional[float]:
    """Return the distance from (ox, oy) to the nearest intersection of the
    ray (origin + t * direction, t >= 0) with the circle centered at
    (cx, cy) with radius r, or None if the ray misses the circle entirely.

    (dx, dy) is assumed to be a unit vector, so the quadratic's `a` term is 1.
    """
    ocx = ox - cx
    ocy = oy - cy

    b = 2.0 * (dx * ocx + dy * ocy)
    c = ocx * ocx + ocy * ocy - r * r

    discriminant = b * b - 4.0 * c
    if discriminant < 0:
        return None

    sqrt_disc = math.sqrt(discriminant)
    t1 = (-b - sqrt_disc) / 2.0
    t2 = (-b + sqrt_disc) / 2.0

    # Smallest non-negative root is the visible intersection point.
    if t1 >= EPSILON:
        return t1
    if t2 >= EPSILON:
        return t2
    return None


def ray_wall_intersection(
    ox: float,
    oy: float,
    dx: float,
    dy: float,
    xmin: float,
    xmax: float,
    ymin: float,
    ymax: float,
) -> Optional[float]:
    """Return the distance from (ox, oy), assumed to be inside the
    axis-aligned rectangle [xmin, xmax] x [ymin, ymax], to the point where
    the ray exits the rectangle (i.e. hits an arena boundary wall).

    Returns None only in the degenerate case where the origin is already
    outside the slab on some parallel axis.
    """
    t_near = -math.inf
    t_far = math.inf

    for o, d, lo, hi in ((ox, dx, xmin, xmax), (oy, dy, ymin, ymax)):
        if abs(d) < EPSILON:
            if o < lo or o > hi:
                return None
            continue
        t1 = (lo - o) / d
        t2 = (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        t_near = max(t_near, t1)
        t_far = min(t_far, t2)

    if t_near > t_far:
        return None
    if t_far < 0:
        return None
    return t_far


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(x2 - x1, y2 - y1)
