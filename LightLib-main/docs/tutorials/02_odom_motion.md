# Odom Motion: Drive to a Field Point

Once odometry is initialized and tuned, the chassis stops thinking in
"24 inches forward" and starts thinking in field coordinates: "go to
`(48, 36)` heading north." This tutorial covers the odom-aware motion
primitives.

## Theory

Odometry fuses three sources to estimate the robot's pose `(x, y, θ)` in
the field frame, every 10 ms:

1. **Tracking wheels** — unpowered omni wheels on rotation sensors. Two
   verticals (forward axis) and two horizontals (lateral axis) give arc
   integration that is immune to scrub and wheel slip.
2. **IMU** — a chip-scale gyro that integrates yaw rate. Drifts at roughly
   1°/min, but is much faster-responding than encoder-based heading.
3. **(Optional) GPS / distance sensors** — absolute references that the
   EKF / MCL fuse in to bound the integration drift.

The arc-integration math is short:

```
ΔL_local = encoder_delta_forward
Δθ      = imu_delta_yaw
ΔW_local = encoder_delta_lateral

# Pose update — assumes constant curvature over the tick (good at 100 Hz)
if Δθ ≈ 0:
    Δx_local = ΔL_local
    Δy_local = ΔW_local
else:
    R = ΔL_local / Δθ            # arc radius
    Δx_local = R * sin(Δθ)
    Δy_local = R * (1 - cos(Δθ)) + ΔW_local

# Rotate into field frame
x += cos(θ_avg) * Δx_local - sin(θ_avg) * Δy_local
y += sin(θ_avg) * Δx_local + cos(θ_avg) * Δy_local
θ += Δθ
```

Because the IMU is ground truth for `θ`, the heading never drifts with
encoder slip. Because the tracking wheels are unpowered, `Δx_local` /
`Δy_local` don't drift with wheel slip either. The only error source is
sensor noise, which is what the EKF cleans up.

The **boomerang** controller is what actually drives the robot to a point.
Naïve "point at target → drive to it" fails when the target has a final
heading you care about — the robot would drive straight at the point and
stop facing the wrong way. Boomerang places a virtual "carrot" point a
configurable distance behind the target along the desired final heading,
drives toward the carrot, and the carrot recedes into the target as you
approach. Result: the robot curves into the target and arrives facing the
correct heading.

## Reading and writing the pose

```cpp
#include "LightLib/drive/odometry.hpp"

Pose p = light::getPose();
printf("X = %.2f in, Y = %.2f in, theta = %.2f deg\n", p.x, p.y, p.theta);

// Reset to a known starting pose at the start of an auton
light::setPose(Pose(36, 12, 0));   // X=36 in, Y=12 in, theta=0 deg

// Get pose with theta in radians (RAMSETE/trajectory code uses this)
Pose pr = light::getPoseRad();

// Field-frame velocity
Pose v = light::getSpeed();
printf("vx=%.2f in/s, vy=%.2f in/s, omega=%.2f deg/s\n", v.x, v.y, v.theta);
```

LightLib's pose convention:
- `+X` is right, `+Y` is forward (toward the opposite alliance wall).
- `θ` is in degrees by default, CW-positive (matching IMU output).
- Origin: wherever you last called `setPose()` — typically robot's
  starting corner.

**Always `setPose()` at the top of every auton.** Odometry from the
previous match is meaningless once you place the robot in a new starting
position.

## Driving to a point — `pid_odom_set`

The simplest odom motion targets a single pose:

```cpp
// Drive to (48, 36) facing 90 degrees, max speed 110
chassis.pid_odom_set({{48, 36, 90}, light::fwd, 110}, /*slew=*/true);
chassis.pid_wait();
```

The struct is `light::odom`:

```cpp
struct odom {
  pose target;                              // {x, y, theta} in inches/deg
  drive_directions drive_direction;         // light::fwd or light::rev
  int max_xy_speed;                         // 0..127
  e_angle_behavior turn_behavior = shortest;
};
```

If you don't care about the final heading, leave `theta = ANGLE_NOT_SET`
(omit it from the brace-init): the robot will arrive at the point facing
whichever direction was natural for the boomerang.

```cpp
// Drive to (48, 36), arrive facing whatever — boomerang picks the heading
pose target;
target.x = 48; target.y = 36;
chassis.pid_odom_set({target, light::fwd, 110});
chassis.pid_wait();
```

### Reverse driving

```cpp
// Drive backwards to a point — useful for backing into a goal
chassis.pid_odom_set({{12, 12, 0}, light::rev, 90});
chassis.pid_wait();
```

The robot will pivot so its back faces the target, then drive backwards.

## Multi-waypoint sequences

Pass a `std::vector<odom>` and the robot chains motions through them with
no stop in between:

```cpp
std::vector<light::odom> path = {
    {{24, 12, 0},   light::fwd, 110},
    {{48, 36, 90},  light::fwd, 110},
    {{36, 60, 180}, light::fwd, 110},
};
chassis.pid_odom_set(path, /*slew=*/true);
chassis.pid_wait();
```

This is *not* a smooth spline — each segment is its own boomerang motion,
just with chained exits. For a continuous curve, use the trajectory
follower (next tutorial).

## Mid-motion triggers

`pid_wait_until_point` fires when the robot reaches a specific waypoint
mid-sequence:

```cpp
chassis.pid_odom_set(path);
chassis.pid_wait_until_point({48, 36, 90});
Loader.set(true);                            // fire at the second waypoint
chassis.pid_wait();
```

`pid_wait_until_index(int)` does the same by index in the path vector.

## Tuning the boomerang

Three knobs in `default_constants()` control the curve shape:

```cpp
chassis.odom_look_ahead_set(15_in);
chassis.odom_boomerang_distance_set(16_in);
chassis.odom_boomerang_dlead_set(0.625);
```

- **`look_ahead`** — how far along the path the robot aims at any moment.
  Smaller = tighter tracking but more wiggly. Larger = smoother but cuts
  corners.
- **`boomerang_distance`** — max distance the carrot can be from the
  target. Larger = wider arc into the target. Smaller = tighter approach.
- **`boomerang_dlead`** — how aggressively the approach tightens at the
  end. `0` = arc smoothly into the target; `1` = sharp final correction.
  `0.625` is a reasonable starting value.

If the robot circles past the target instead of curving in, reduce
`boomerang_distance`. If it arrives at the right point but facing the
wrong way, raise `boomerang_dlead`.

## `odom_turn_bias_set`

```cpp
chassis.odom_turn_bias_set(1.0);
```

Ranges 0..1. Controls how much the robot prioritizes hitting the right
heading vs. the right position. `1.0` = full priority on heading
(recommended with tracking wheels). Values closer to `0` make the robot
accept some heading error in exchange for faster XY convergence.

## `pid_odom_behavior_set`

Sets the default angle-wrap rule for all subsequent odom turns:

```cpp
chassis.pid_odom_behavior_set(light::shortest);
```

Same enum as `pid_turn_set`'s third arg: `shortest`, `longest`, `cw`,
`ccw`, `raw`.

## `moveToPoint` — the legacy free function

For very simple auton logic (or as a fallback if you don't want to deal
with the odom struct):

```cpp
#include "LightLib/drive/odometry.hpp"

// (x, y, timeout_ms, max_speed, reversed)
moveToPoint(48, 36, 3000, 110, false);
```

It's a thin wrapper that builds a one-shot `pid_odom_set` and waits for
it. Identical behavior, less ceremony. Use it when you don't need the
chain / wait_until features.

## Common pitfalls

**Robot drifts off the path on the first move.** Almost always means
tracking wheels haven't been zeroed by `light::reset()` or starting pose
hasn't been set. Always call `setPose()` at the top of every auton.

**Robot arrives 6" off but heading is correct.** Tracking wheel offsets
are wrong. Re-measure the perpendicular distance from each wheel to the
robot center and update the `TrackingWheel` constructors in main.cpp.

**Robot curves smoothly to a point but stops short.** Drive exit
conditions are too loose. Tighten `pid_odom_drive_exit_condition_set` —
specifically the `small_error` (likely `1_in` is too generous; try
`0.5_in`).
