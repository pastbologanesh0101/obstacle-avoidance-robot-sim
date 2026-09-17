"""The main simulation loop: drive a robot from start to goal, timestep by
timestep, until it succeeds, collides, or times out.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

from .arena import Arena
from .controller import ReactiveController
from .geometry import distance
from .robot import Robot


@dataclass
class SimulationResult:
    status: str  # "success" | "collision" | "timeout"
    steps: int
    trajectory: List[Tuple[float, float]]
    final_distance_to_goal: float

    @property
    def path_length(self) -> float:
        total = 0.0
        for (x1, y1), (x2, y2) in zip(self.trajectory, self.trajectory[1:]):
            total += distance(x1, y1, x2, y2)
        return total


def run_simulation(
    arena: Arena,
    controller: ReactiveController = None,
    max_steps: int = 500,
    goal_tolerance: float = 1.0,
    dt: float = 1.0,
    max_turn_rate: float = None,
    max_speed: float = None,
) -> SimulationResult:
    """Run the robot from `arena.start`/`arena.start_heading` toward
    `arena.goal`, for at most `max_steps` timesteps.

    Guaranteed to terminate: it returns as soon as the robot reaches the
    goal ("success"), collides with an obstacle or wall ("collision"), or
    exhausts `max_steps` without either happening ("timeout").
    """
    if controller is None:
        controller = ReactiveController()

    robot = Robot(x=arena.start[0], y=arena.start[1], heading=arena.start_heading)
    trajectory: List[Tuple[float, float]] = [(robot.x, robot.y)]

    turn_kwargs = {}
    if max_turn_rate is not None:
        turn_kwargs["max_turn_rate"] = max_turn_rate
    if max_speed is not None:
        turn_kwargs["max_speed"] = max_speed

    steps_taken = 0
    status = "timeout"

    for step_index in range(max_steps):
        steps_taken = step_index
        if arena.collides(robot.x, robot.y, robot.radius):
            status = "collision"
            break
        if arena.reached_goal(robot.x, robot.y, goal_tolerance):
            status = "success"
            break

        turn, speed = controller.compute(robot, arena)
        robot.step(turn, speed, dt=dt, **turn_kwargs)
        trajectory.append((robot.x, robot.y))
    else:
        # Loop completed all max_steps without break: re-check final pose.
        if arena.collides(robot.x, robot.y, robot.radius):
            status = "collision"
        elif arena.reached_goal(robot.x, robot.y, goal_tolerance):
            status = "success"
        else:
            status = "timeout"
        steps_taken = max_steps

    final_distance = distance(robot.x, robot.y, arena.goal[0], arena.goal[1])
    return SimulationResult(
        status=status,
        steps=steps_taken,
        trajectory=trajectory,
        final_distance_to_goal=final_distance,
    )
