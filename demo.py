#!/usr/bin/env python3
"""Demo CLI: run the obstacle-avoidance simulator on the bundled arenas and
print the outcome + basic trajectory stats for each.

This is a 2D SOFTWARE SIMULATION only -- see README.md.
"""

from __future__ import annotations

import sys

from obstacle_avoidance_robot.arenas import BUNDLED_ARENAS
from obstacle_avoidance_robot.controller import ReactiveController
from obstacle_avoidance_robot.simulation import run_simulation


def run_and_report(arena_key: str) -> str:
    arena_factory = BUNDLED_ARENAS[arena_key]
    arena = arena_factory()
    controller = ReactiveController()

    result = run_simulation(arena, controller=controller, max_steps=600, goal_tolerance=1.0)

    lines = []
    lines.append(f"Arena: {arena.name} ({arena_key})")
    lines.append(f"  Obstacles: {len(arena.obstacles)}")
    lines.append(f"  Start: {arena.start}  Goal: {arena.goal}")
    lines.append(f"  Outcome: {result.status.upper()}")
    lines.append(f"  Steps taken: {result.steps}")
    lines.append(f"  Path length: {result.path_length:.2f} units")
    lines.append(f"  Final distance to goal: {result.final_distance_to_goal:.2f} units")
    return "\n".join(lines)


def main() -> int:
    keys = sys.argv[1:] if len(sys.argv) > 1 else list(BUNDLED_ARENAS.keys())

    unknown = [k for k in keys if k not in BUNDLED_ARENAS]
    if unknown:
        print(f"Unknown arena(s): {unknown}. Available: {list(BUNDLED_ARENAS.keys())}")
        return 1

    print("Obstacle Avoidance Robot -- 2D software simulation demo")
    print("=" * 56)
    for key in keys:
        print()
        print(run_and_report(key))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
