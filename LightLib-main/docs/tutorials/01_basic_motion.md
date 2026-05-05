# Basic Motion: Drive, Turn, Swing

This tutorial covers the bread-and-butter autonomous primitives: driving in
a straight line, turning in place, and swing turns. Every LightLib auton is
built from some combination of these three commands plus the `pid_wait`
family.

## Theory

A PID controller takes a setpoint (target) and a measurement (current
position) and produces a motor command that drives the error toward zero:

```
output = kP * error + kI * integral(error) + kD * d(error)/dt
```

**kP** (proportional) decides how hard the robot pushes against error. Too
high → oscillation; too low → sluggish, doesn't reach the target.

**kI** (integral) accumulates past error so the robot doesn't get stuck a
fraction of an inch short of the target. Almost always small (or zero).
Large kI causes overshoot and windup.

**kD** (derivative) damps the response — it pushes back when error is
*shrinking fast*, preventing overshoot. Tune kD up until the motion looks
critically damped (lands on the target without oscillating).

LightLib runs four PIDs simultaneously during a drive motion:

1. **Drive PID** — controls forward/back position via average wheel encoder.
2. **Heading PID** — corrects yaw drift using the IMU. Gives one side of the
   chassis slightly more voltage to keep the heading at the value it had
   when the move started.
3. **Turn PID** — controls heading during `pid_turn_set`.
4. **Swing PID** — controls heading when only one side of the chassis moves.

Slew is layered on top: the drive PID's output is clamped to ramp from a
small starting voltage up to its commanded max over the first few inches,
so the wheels don't slip on launch.

## Drive PID — `pid_drive_set`

Drives forward (or backward) by a distance in inches. The IMU heading PID
holds the starting heading throughout.

```cpp
// Drive 24 inches forward at speed 110 (out of 127)
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();

// Drive 12 inches backward
chassis.pid_drive_set(-12_in, 90);
chassis.pid_wait();

// Drive 36 inches with slew enabled and heading hold disabled
chassis.pid_drive_set(36_in, 127, true, false);
chassis.pid_wait();
```

**Signature** (one of several overloads — see [drive.hpp](../../include/LightLib/drive/drive.hpp)):

```cpp
void pid_drive_set(okapi::QLength p_target, int speed,
                   bool slew_on = false, bool toggle_heading = true);
```

- `p_target` — distance with units. `24_in`, `0.5_ft`, `300_mm` all work.
- `speed` — voltage scale, ±127. Sign of `p_target` decides direction; sign
  of `speed` is ignored.
- `slew_on` — ramp up from `slew_drive_constants_set` floor over the
  configured distance.
- `toggle_heading` — if `false`, heading correction is disabled (the IMU is
  ignored). Useful if the robot has just rammed a wall and the IMU
  measurement is unreliable for the next move.

### Common pitfall

Forgetting `pid_wait()`. Without it, the next line runs *immediately* —
your auton fires off six PID commands in a row and the chassis only
executes the last one. Every motion command needs a pair:

```cpp
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();           // ← block until done

chassis.pid_turn_set(90_deg, 90);
chassis.pid_wait();
```

## Turn PID — `pid_turn_set`

Turns to an **absolute** field heading (or a relative one — see below).
LightLib's IMU convention is `0°` = facing the alliance wall it was zeroed
at, CW positive.

```cpp
// Face 90° (a quarter turn clockwise from start)
chassis.pid_turn_set(90_deg, 90);
chassis.pid_wait();

// Face 270° via the shortest path (will go CCW because it's 90° CCW vs.
// 270° CW)
chassis.pid_turn_set(270_deg, 90, light::shortest);
chassis.pid_wait();

// Force a specific direction — useful if a wall is in the shortest path
chassis.pid_turn_set(180_deg, 90, light::cw);  // long way around
chassis.pid_wait();
```

**Signature**:

```cpp
void pid_turn_set(okapi::QAngle p_target, int speed,
                  e_angle_behavior behavior = light::shortest,
                  bool slew_on = false);
```

The `e_angle_behavior` enum values are: `light::shortest` (default),
`light::longest`, `light::cw`, `light::ccw`, `light::raw` (signed delta as-is).

### Relative turns — `pid_turn_relative_set`

```cpp
// Rotate 90° clockwise from current heading, regardless of where it is now
chassis.pid_turn_relative_set(90_deg, 90);
chassis.pid_wait();
```

Use this when the auton "doesn't care" about absolute field heading — most
of the time, honestly. Absolute turns make sense after `setPose()` resets
or near features keyed to the field.

## Swing PID — `pid_swing_set`

A swing turn locks one side of the drive and pivots around it. Half the
turn radius of an in-place turn, useful for sweeping into a target without
disturbing a mechanism on the locked side.

```cpp
// Right swing to face 45° (left side drives, right side held in place)
chassis.pid_swing_set(light::RIGHT_SWING, 45_deg, 90);
chassis.pid_wait();

// Left swing 90° relative to current heading
chassis.pid_swing_relative_set(light::LEFT_SWING, 90_deg, 90);
chassis.pid_wait();

// Swing where the locked side gets a small reverse boost (sharper pivot)
// 90 = main side speed, -30 = opposite side speed
chassis.pid_swing_set(light::RIGHT_SWING, 45_deg, 90, -30);
chassis.pid_wait();
```

The opposite-speed overload is unique to swing. Most teams ignore it, but
it can sharpen a sweeping intake into a stack — give the locked side a
small reverse voltage to tighten the radius.

## The `pid_wait` family

`pid_wait()` blocks until the active motion satisfies its exit conditions.
There are four variants:

```cpp
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait();              // wait fully — block until exit_conditions met

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait_quick();        // exit when error < small_error (no settle wait)

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait_quick_chain();  // exit early at chain_constant; queue next move

chassis.pid_drive_set(24_in, 110);
chassis.pid_wait_until(12_in);   // exit at the 12-inch mark, mid-motion
```

**`pid_wait_quick_chain`** is the magic one. It exits the current motion
when error drops below `pid_drive_chain_constant_set(...)` (typically a few
inches), so you can fire the next motion command without slowing the robot
to a stop. Use it for fluid multi-move sequences:

```cpp
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait_quick_chain();         // exit ~5 in early

chassis.pid_turn_set(45_deg, 90);
chassis.pid_wait_quick_chain();         // exit ~10° early

chassis.pid_drive_set(36_in, 110);
chassis.pid_wait();                     // last move waits fully
```

The chain constants are configured in `default_constants()`:

```cpp
chassis.pid_drive_chain_constant_set(5_in);
chassis.pid_turn_chain_constant_set(10_deg);
chassis.pid_swing_chain_constant_set(15_deg);
```

`pid_wait_until(distance)` is for triggering an action mid-motion — drop a
piston at the 12-inch mark, then keep driving:

```cpp
chassis.pid_drive_set(24_in, 110);
chassis.pid_wait_until(12_in);
Wings.set(true);                  // fire while still driving
chassis.pid_wait();                // now wait for the rest
```

## Putting it together: a real auton

```cpp
void red_left_rush() {
  // Reset to known starting pose (X, Y, theta_deg)
  light::setPose(Pose(36, 12, 0));

  // Slam forward into the goal
  chassis.pid_drive_set(36_in, 127, /*slew=*/true);
  chassis.pid_wait_until(20_in);
  Loader.set(true);                              // start scoring at 20 in
  chassis.pid_wait();

  // Back up and pivot toward the alliance ladder
  chassis.pid_drive_set(-18_in, 110);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90_deg, 90);
  chassis.pid_wait_quick_chain();

  // Final approach
  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait();
  Wings.set(true);
}
```

That's the entire vocabulary of "hand-crafted" autons in LightLib. Anything
more elaborate — driving to an arbitrary field point, following a Jerryio
path — uses the odom-based motion APIs covered in the next tutorial.
