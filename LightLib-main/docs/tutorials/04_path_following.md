# Path Following: RAMSETE & Jerryio

This tutorial covers smooth trajectory following — driving along a curved
path with continuous velocity instead of stop-and-pivot motion. LightLib
implements RAMSETE (a nonlinear pose tracker) on top of a quintic Hermite
spline + trapezoidal velocity profile.

## Theory

### Why not just chain pid_odom_set calls?

`pid_odom_set` with a vector of waypoints chains boomerang motions, but
each segment is a separate "go to that point" with its own acceleration
profile. The robot momentarily slows at every waypoint. For a smooth
serpentine across the field, you want **one continuous trajectory** that
the robot tracks at a controlled velocity throughout.

### What RAMSETE does

A trajectory is a function `(x(t), y(t), θ(t), v(t), ω(t))` defined every
10 ms — the robot's *desired* pose and velocity at every moment. RAMSETE
("Rapidly converging Asymptotically stable Method for SE(2) Tracking") is
a controller that takes:

- Desired pose `(x_d, y_d, θ_d)` from the trajectory
- Desired forward velocity `v_d` and angular velocity `ω_d`
- Actual pose `(x, y, θ)` from odometry

…and produces an actual `(v, ω)` command that drives the robot back onto
the trajectory. The controller has two parameters:

- **`b`** (aggressiveness) — how hard it corrects pose error. Larger →
  faster correction but jerkier. Typical: `1.0`–`2.5`.
- **`zeta`** (damping) — how much it damps velocity error. `0` = pure
  proportional, `1` = critically damped. Typical: `0.7`.

Underneath, each tank-side wheel velocity is mapped from `(v, ω)`:

```
v_left  = v - ω·trackWidth/2
v_right = v + ω·trackWidth/2
```

Then a per-wheel feedforward + P controller turns those into voltages:

```
V = kS·sgn(v_wheel) + kV·v_wheel + kA·a_wheel + kP·(v_wheel - measured_v)
```

`kS`, `kV`, `kA` are the motor constants you identify with
`characterize_kV_kA_kS` (next tutorial). `kP` is small — a few centivolts
per (in/s) error. RAMSETE handles the geometry; the feedforward handles
the motor dynamics.

### Trajectory generation

LightLib generates trajectories from waypoints in two passes:

1. **Spline pass** — fit a quintic Hermite spline through the waypoints.
   "Quintic" means C² continuous (continuous position, velocity, AND
   acceleration). This is what gives the path smooth curvature instead of
   sharp corners.
2. **Velocity pass** — walk the spline and assign a velocity at every 10
   ms tick, respecting four constraints:
   - `vMax` — max linear speed (in/s)
   - `aMax` — max forward accel (in/s²)
   - `aDecMax` — max deceleration (in/s²)
   - `aLatMax` — max centripetal accel (`v² / R` ≤ aLatMax → forces a
     slowdown in tight curves)

   The velocity pass is a **forward-backward sweep**: forward to enforce
   accel limits from the start, backward to enforce decel limits before
   each waypoint and the end, with the lateral cap clamped throughout.
   Result: the fastest profile that respects every constraint.

That's the entire pipeline. The result is a `Trajectory` object — a dense
table of `TrajState{x, y, θ, v, ω, a, κ, t}` sampled every 10 ms.

## Configuration

Path following needs three structs configured *once* in `initialize()`,
before any auton runs:

```cpp
#include "LightLib/path/ramsete.hpp"

void initialize() {
  // ... chassis init, default_constants(), etc ...

  light::ramsete_configure(
      // RamseteConfig: controller gains + chassis geometry
      { /*b=*/2.0f, /*zeta=*/0.7f,
        /*trackWidthIn=*/11.5f,
        /*wheelDiamIn=*/3.25f,
        /*gearRatio=*/0.6f },           // wheel_rpm / motor_rpm; 0.6 = 36:60 blue→wheels

      // DriveFF: per-wheel feedforward + velocity-loop P
      { /*kS=*/0.60f,                    // V to break static friction
        /*kV=*/0.18f,                    // V per (in/s) steady-state
        /*kA=*/0.03f,                    // V per (in/s²)
        /*kP=*/0.02f },                  // V per (in/s) of velocity error

      // TrajConstraints: kinematic limits
      { /*vMax=*/48.0f,                  // in/s
        /*aMax=*/60.0f,                  // in/s² forward
        /*aDecMax=*/60.0f,               // in/s² braking
        /*aLatMax=*/40.0f }              // in/s² centripetal
  );
}
```

The numbers above are reasonable starting points for a blue-cart 11.5"
tank chassis on 3.25" wheels. The `kS / kV / kA` values come from running
`characterize_kV_kA_kS` (next tutorial). The kinematic limits start
conservative — once the feedforward is right, you can push `vMax` up
toward 60 in/s.

## Following a trajectory

### Inline waypoint list

```cpp
#include "LightLib/path/ramsete.hpp"

void my_auton() {
  light::setPose(Pose(0, 0, 0));   // always reset pose at the top

  std::vector<light::Waypoint> path = {
      {.x = 0,  .y = 0,  .headingRad = 0},
      {.x = 24, .y = 24, .headingRad = M_PI/2},
      {.x = 48, .y = 0,  .headingRad = 0},
  };

  light::TrajConstraints cons{
      .vMax = 48.0f, .aMax = 60.0f, .aDecMax = 60.0f, .aLatMax = 40.0f
  };

  bool ok = light::followTrajectory(path, cons);
  if (!ok) printf("path bailed!\n");
}
```

`followTrajectory` blocks until the robot reaches the end (or bails on
timeout / pose error). The return value tells you whether it completed
cleanly.

### Waypoint anatomy

```cpp
struct Waypoint {
  float x = 0.0f;                   // inches
  float y = 0.0f;                   // inches
  std::optional<float> headingRad;  // radians; if absent, tangent from chord
  std::optional<float> speed;       // optional per-waypoint speed cap (unused yet)
};
```

If you omit `headingRad` on an interior waypoint, the spline picks a
heading that's a smooth blend of the chords on either side. Set
`headingRad` only when you specifically need a particular tangent
direction (start of path, end of path, sharp turn at a known point).

### Reverse paths

```cpp
light::followTrajectory(path, cons, /*reversed=*/true);
```

The robot drives the entire path backwards. Useful for backing into a
goal — define the path as if going forward, then flip the bit.

### Timeouts and bails

```cpp
//                                                  reversed  timeout  pose_err_bail
light::followTrajectory(path, cons, /*reversed=*/false, /*ms=*/8000, /*in=*/8.0f);
```

`poseErrBailIn` is the safety net: if the robot's pose error exceeds this
many inches at any point during the trajectory (e.g. someone is shoving
it, or odom desynced), the follower gives up and returns `false`. Tune
based on how aggressive your paths are.

## Mid-path triggers: PathEvent

Fire an action at a specific waypoint mid-path:

```cpp
std::vector<light::PathEvent> events = {
  { .atWaypoint = 1, .action = []{ Wings.set(true);  } },
  { .atWaypoint = 2, .action = []{ Loader.set(true); } },
};

light::followTrajectory(path, cons, events);
```

`atWaypoint` is the zero-based index in your `wps` vector. The action
fires on the 10 ms control tick where the trajectory's *time* first
crosses the time at which the trajectory passes that waypoint. Each event
fires at most once.

Callbacks may call `light::setPose()` to re-zero odometry mid-path — the
next tick will read the new pose and RAMSETE's error term will correct
against the unchanged trajectory. This is a clean way to handle "I rammed
the wall, my IMU drifted, now I'm fixing it" mid-auton.

## Jerryio paths: runJerryioPath

[path.jerryio.com](https://path.jerryio.com) is a browser-based path
designer. You drag waypoints around a field and it exports a CSV. LightLib
parses that CSV directly:

```cpp
// Export the path from path.jerryio.com as CSV, paste it into a header:
// include/paths/red_left.hpp
namespace light::paths {
inline constexpr const char* red_left = R"JERRYIO(
0.0, 0.0, 0.0
24.0, 24.0, 1.5708
48.0, 0.0, 0.0
)JERRYIO";
}

// Then in your auton:
#include "paths/red_left.hpp"

void red_left_auton() {
  light::setPose(Pose(0, 0, 0));
  light::runJerryioPath(light::paths::red_left);
}
```

The CSV format: each non-empty, non-`#`-commented line has 2 or 3 floats
(comma, tab, or whitespace separated). `x, y` or `x, y, heading_rad`.
Units are inches and radians (LightLib convention: +Y forward, theta=0
faces +Y, CW positive). Extra columns past 3 are ignored, so exports with
speed/lookahead fields still work.

### Reading from the SD card

For paths too big to compile in:

```cpp
light::runJerryioPathFromSD("/usd/paths/red_left.csv");
```

Same parser, just reads the file at runtime. Useful if you want to update
paths without re-flashing.

### The path registry: runPath

For multi-auton programs, register paths by name:

```cpp
// include/paths/all.hpp:
#include "paths/red_left.hpp"
#include "paths/red_right.hpp"
#include "paths/blue_left.hpp"

inline constexpr PathEntry kAll[] = {
    { "red_left",   red_left   },
    { "red_right",  red_right  },
    { "blue_left",  blue_left  },
};

// Then anywhere:
light::runPath("red_left");
```

It's just a thin lookup over `kAll[]`. Lookup is O(N) string compare —
fine for the small number of paths a single robot carries. Unknown names
print a warning and return `false` without invoking the follower.

## Tuning RAMSETE

If the robot tracks the path with persistent error in one direction:

- **Lateral error (drives parallel to the path, offset)** — `kV` is too
  low, `kS` is too low, or the chassis has uneven friction. Re-run
  `characterize_kV_kA_kS` and `characterize_friction`.
- **Lagging behind the desired velocity** — `kA` is too low, or `vMax` is
  optimistic. Drop `vMax` first, then re-tune `kA`.
- **Jerky / over-correcting** — `b` is too high. Drop from 2.0 to 1.5.
- **Sluggish corrections after a bump** — `b` is too low. Bump up.
- **Velocity oscillation on straight segments** — feedforward `kP` is
  too high. The default of `0.02` is conservative; if you've raised it,
  drop back down.

Start with the defaults, get characterized values, then tune `b` last.

## Common pitfalls

**Robot tracks well at low `vMax` but oscillates at high `vMax`.** Almost
always means `kA` is wrong — probably too low. Re-run
`characterize_kV_kA_kS` with a longer test distance (so the accel phase
gives the fitter more samples).

**First waypoint heading is "wrong" on launch.** You set `headingRad` on
the first waypoint to a value that doesn't match the actual robot start
heading. Either match it, or call `light::setPose()` to match the robot
to the path before running.

**Path completes but ends rotated 5° off.** End-of-path heading isn't
RAMSETE's responsibility — it tracks the trajectory until the trajectory
ends, and the final tangent of the spline determines the heading. If you
need a precise final heading, follow the path with `runJerryioPath` and
then queue a `pid_turn_set` to lock the heading:

```cpp
light::runJerryioPath(my_path);
chassis.pid_turn_set(180_deg, 90);  // snap to exactly 180°
chassis.pid_wait();
```
