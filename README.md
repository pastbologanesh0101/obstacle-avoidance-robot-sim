# Obstacle Avoidance Robot (2D Software Simulation)

**This is a 2D software simulation. There is no real robot, no real
hardware, and no physical sensors involved anywhere in this repository.**
Everything -- the arena, the obstacles, the ultrasonic-style range
sensors, and the differential-drive robot -- is pure Python math running
in a simulation loop on your computer. Nothing here talks to a serial
port, a microcontroller, or a physical actuator.

It's a companion piece to [`line-following-robot-sim`](https://github.com/pastbologanesh0101/line-following-robot-sim)
(PID line-following along a track) but a genuinely different problem:
here the robot has no line to follow at all. It starts at an arbitrary
point in an open 2D arena scattered with circular obstacles, and has to
reactively steer itself toward a goal point using only live simulated
distance-sensor readings -- much closer to how a real ultrasonic/IR
obstacle-avoidance robot (e.g. an Arduino + HC-SR04 rig) actually
behaves.

## What's actually being simulated

- **Arena**: a bounded rectangular 2D world (`Arena` in `arena.py`) with
  circular obstacles at fixed positions, plus the arena's own boundary
  walls.
- **Sensors**: a 5-sensor array (`front`, `front_left`, `front_right`,
  `left`, `right`, see `sensors.py`) modeled as rays cast from the
  robot's current position at fixed angular offsets from its heading.
  Each sensor computes the *true* geometric distance to the nearest
  obstacle or wall along its ray, using closed-form ray-circle and
  ray-box (slab method) intersection math (`geometry.py`), capped at a
  maximum sensor range. This is the same math a real ultrasonic sensor
  is approximating physically (time-of-flight to the nearest reflecting
  surface).
- **Robot**: a simplified differential-drive kinematic model
  (`robot.py`) -- each timestep it turns by a clamped heading delta and
  moves forward by a clamped speed.
- **Simulation loop**: (`simulation.py`) advances the robot step by step
  until it reaches the goal (success), collides with an obstacle/wall
  (collision), or exhausts a step budget (timeout). It is guaranteed to
  terminate -- there is no unbounded loop.

## Control algorithm: reactive potential fields

The controller (`controller.py`) is a classic **artificial potential
fields** approach, entirely reactive (no map, no path planning, no
memory of past obstacles):

1. **Attractive force**: a unit vector pointing from the robot straight
   at the goal, pulling it forward.
2. **Repulsive force**: for every sensor whose current reading is inside
   an "influence range", a force is added pointing directly *away* from
   that sensor's ray direction, with magnitude growing sharply as the
   reading approaches zero (and vanishing at the influence range).
3. The two are summed into a net vector. The robot turns toward that
   net vector's heading, clamped to its maximum turn rate per step, and
   brakes (reduces forward speed) the closer its front sensor gets to
   something.

In plain terms: *drive toward the goal, but the closer any sensor gets to
an obstacle, the harder the robot gets pushed away from that direction* --
so if an obstacle is detected on the front-right, the robot turns left
(away from it), and vice versa. This is intentionally simple and
explainable rather than optimal; like any purely reactive controller it
has known failure modes (e.g. it can be driven into a collision by a gap
that's simply too narrow -- see the "hard" arena below).

### Hardware portability note

Nothing in `controller.py`, `robot.py`, or `sensors.py` assumes a
simulation. The controller only consumes `{sensor_name: distance}`
readings and produces `(turn, speed)`. To run this on a real
differential-drive robot you would swap:

- `sensors.read_all()` → real ultrasonic/IR/ToF sensor reads (e.g. HC-SR04
  echo timing), keeping the same sensor names/angles.
- `Robot.step()` → real motor PWM/duty-cycle commands for a two-wheel
  differential drive.

The control law itself (`ReactiveController.compute_from_readings`)
would not need to change at all.

## Running the demo

Requires only the Python 3 standard library (Python 3.11+; no
third-party dependencies, numpy is not required).

```bash
python3 demo.py            # runs all 3 bundled arenas
python3 demo.py easy hard   # or pick specific ones
```

### Example output -- a success case

```
Arena: easy_open_field (easy)
  Obstacles: 1
  Start: (2.0, 2.0)  Goal: (28.0, 3.0)
  Outcome: SUCCESS
  Steps taken: 26
  Path length: 26.00 units
  Final distance to goal: 0.12 units

Arena: medium_scattered (medium)
  Obstacles: 4
  Start: (2.0, 10.0)  Goal: (28.0, 10.0)
  Outcome: SUCCESS
  Steps taken: 36
  Path length: 33.76 units
  Final distance to goal: 0.15 units
```

### Example output -- an honest failure case

The bundled "hard" arena includes a deliberately narrow gate (a 1.5-unit
gap between two obstacles, for a robot that is 1.0 units in diameter)
directly on the shortest path to the goal. This is *not* tuned to always
succeed -- it's included to be honest about the real limitation of a
purely reactive controller: with no global path planning, it can steer
itself into a gap that's simply too tight and collide.

```
Arena: hard_gauntlet (hard)
  Obstacles: 6
  Start: (2.0, 12.0)  Goal: (28.0, 12.0)
  Outcome: COLLISION
  Steps taken: 12
  Path length: 11.39 units
  Final distance to goal: 14.71 units
```

(Widen `gap` in `arenas.py::make_hard_arena` to `2.0` or more and it
reliably squeezes through instead -- that narrow margin is exactly the
"near miss" boundary between the two outcomes.)

## Project layout

```
obstacle_avoidance_robot/
  geometry.py     # ray-circle / ray-wall intersection math, angle utils
  arena.py        # Arena + Obstacle, collision/goal checks
  robot.py        # differential-drive kinematics
  sensors.py      # simulated 5-sensor ultrasonic-style array
  controller.py   # reactive potential-fields controller
  simulation.py   # the run-to-completion simulation loop
  arenas.py       # bundled easy / medium / hard test arenas
demo.py           # CLI entry point
tests/            # unit tests (unittest, stdlib only)
```

## Running the tests

```bash
python3 -m unittest discover -s tests -v
```

30 unit tests cover the ray-circle and ray-wall intersection geometry
against known cases, sensor readings (clear path → max range, obstacle
directly ahead → correct close distance), the controller's steering
direction and braking behavior, arena collision/goal detection, and the
simulation loop's three termination paths (success, collision, timeout)
including a guarantee that it never runs past `max_steps`. CI
(`.github/workflows/tests.yml`) runs the full suite on Python 3.11 and
3.12 on every push and pull request.
