import unittest

from obstacle_avoidance_robot.arena import Arena, Obstacle
from obstacle_avoidance_robot.arenas import make_easy_arena, make_hard_arena
from obstacle_avoidance_robot.controller import ReactiveController
from obstacle_avoidance_robot.simulation import run_simulation


class TestSimulation(unittest.TestCase):
    def test_reaches_goal_on_simple_obstacle_free_arena(self):
        arena = Arena(
            xmin=0.0,
            xmax=20.0,
            ymin=0.0,
            ymax=20.0,
            obstacles=[],
            start=(1.0, 1.0),
            goal=(18.0, 1.0),
        )
        result = run_simulation(arena, controller=ReactiveController(), max_steps=200)
        self.assertEqual(result.status, "success")
        self.assertLess(result.final_distance_to_goal, 1.0)
        self.assertGreater(len(result.trajectory), 1)

    def test_bundled_easy_arena_succeeds(self):
        arena = make_easy_arena()
        result = run_simulation(arena, controller=ReactiveController(), max_steps=600)
        self.assertEqual(result.status, "success")

    def test_bundled_hard_arena_reports_collision(self):
        # This arena has a deliberately-too-narrow gate; the reactive
        # controller is expected to genuinely collide here (see README).
        arena = make_hard_arena()
        result = run_simulation(arena, controller=ReactiveController(), max_steps=600)
        self.assertEqual(result.status, "collision")

    def test_terminates_with_timeout_when_goal_is_unreachable_in_time(self):
        # Goal is technically reachable but max_steps is far too small, so
        # the loop must terminate via the timeout path, not run forever.
        arena = Arena(
            xmin=0.0,
            xmax=1000.0,
            ymin=0.0,
            ymax=1000.0,
            obstacles=[],
            start=(500.0, 500.0),
            goal=(999.0, 999.0),
        )
        result = run_simulation(arena, controller=ReactiveController(), max_steps=5)
        self.assertEqual(result.status, "timeout")
        self.assertEqual(result.steps, 5)

    def test_simulation_never_exceeds_max_steps(self):
        arena = Arena(
            xmin=0.0,
            xmax=1000.0,
            ymin=0.0,
            ymax=1000.0,
            obstacles=[],
            start=(500.0, 500.0),
            goal=(999.0, 999.0),
        )
        result = run_simulation(arena, controller=ReactiveController(), max_steps=10)
        self.assertLessEqual(result.steps, 10)
        self.assertLessEqual(len(result.trajectory), 11)

    def test_collision_terminates_loop_immediately(self):
        # Obstacle sitting right on top of the start point -> should be
        # flagged as a collision on step 0, without ever moving.
        arena = Arena(
            xmin=0.0,
            xmax=20.0,
            ymin=0.0,
            ymax=20.0,
            obstacles=[Obstacle(x=1.0, y=1.0, radius=1.0)],
            start=(1.0, 1.0),
            goal=(18.0, 18.0),
        )
        result = run_simulation(arena, controller=ReactiveController(), max_steps=200)
        self.assertEqual(result.status, "collision")
        self.assertEqual(result.steps, 0)


if __name__ == "__main__":
    unittest.main()
