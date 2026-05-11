# PID Tuning Workflow

This tutorial walks through tuning the four chassis PIDs (drive, turn,
swing, heading) from scratch. Plan to spend an evening on it. Bad PID
constants are the most common reason "the auton ran fine yesterday and
not today."

## Theory

Recall the PID equation:

```
u(t) = kP·e(t) + kI·∫e(τ)dτ + kD·de(t)/dt
```

When you tune, you are searching for the smallest gains that produce a
"critically damped" response: the robot reaches the target quickly,
without oscillating, and without sitting short of it.

There are three classic failure modes — recognize them by the response
shape:

```
target ────────────────────────────
       /                            ← target value
      / ⌒⌒⌒⌒⌒                       ← OSCILLATION: kP too high (or kD too low)
     /
    /
   /          ━━━━━━━━━━━━━━━━━━━   ← UNDERSHOOT: kP too low
  /                                    (steady state error → bump kI a tiny bit,
 /                                      or just raise kP)
/______________
                                       OVERSHOOT-AND-RING: kD too low
                                       (or kI windup)
```

Tuning rule of thumb: **kP first, kD second, kI last (often zero).**

1. Start with `kP = 1`, `kI = 0`, `kD = 0`.
2. Raise `kP` until the robot just barely overshoots the target.
3. Raise `kD` until the overshoot is gone but the robot still arrives
   quickly. Typical ratio: `kD ≈ 5–10× kP` for distance PIDs, `kD ≈ 5–15×
   kP` for angle PIDs.
4. Only add `kI` (small — `0.01`–`0.1`) if the robot consistently lands a
   small distance short of the target. Most well-tuned robots have
   `kI = 0`.

LightLib runs these PIDs at 100 Hz, so each motion has ~30 cycles to
settle in a one-second motion. That's plenty of headroom for clean tuning.

## Where the constants live

All four PIDs are configured in `default_constants()` in
[src/autons.cpp](../../src/autons.cpp). The shipped defaults are
reasonable starting points for a 4-motor blue-cart 11.5" tank chassis:

```cpp
void default_constants() {
  chassis.pid_drive_constants_set(10.0, 0.1, 30.0);
  chassis.pid_heading_constants_set(17.0, 0.0, 70.0);
  chassis.pid_turn_constants_set(5.0, 0.0, 45.0);
  chassis.pid_swing_constants_set(4.0, 0.0, 45.0);
  chassis.pid_odom_angular_constants_set(1.0, 0.0, 10.0);
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);
  // ... exit conditions, slew, etc.
}
```

If your robot is heavier, lighter, or geared differently, you'll need to
re-tune. The values above are starting points, not destinations.

## The tuning workflow

### Step 1 — set up test autons

The `autons.hpp` header declares stub functions like `drive_test(int)` and
`turn_test(int)` for tuning. **You implement these** — they aren't built
into LightLib. A reasonable implementation looks like:

```cpp
// In autons.cpp, below default_constants()
void drive_test(int inches) {
  chassis.pid_drive_set(inches * 1_in, 110);
  chassis.pid_wait();
}

void turn_test(int degrees) {
  chassis.pid_turn_set(degrees * 1_deg, 90);
  chassis.pid_wait();
}

void swing_test(int degrees) {
  chassis.pid_swing_set(light::RIGHT_SWING, degrees * 1_deg, 90);
  chassis.pid_wait();
}

void heading_test(int degrees) {
  // Drive 24 in while fighting an artificial heading offset
  chassis.pid_turn_set(degrees * 1_deg, 90);
  chassis.pid_wait();
  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait();
}
```

Then register them in `auton_config.cpp` so they show up on the brain
selector:

```cpp
light::auton_selector.add("Drive 24 in", "tune drive PID", []{ drive_test(24); });
light::auton_selector.add("Turn 90°",    "tune turn PID",  []{ turn_test(90); });
light::auton_selector.add("Swing 90°",   "tune swing PID", []{ swing_test(90); });
```

### Step 2 — tune drive PID

1. Pick "Drive 24 in" from the selector.
2. Run it. Watch the response:
   - **Overshoots and bounces back** → kP too high, or kD too low. Lower
     kP by 20% or raise kD by 50%.
   - **Stops short and creeps in** → kP too low. Raise by 20–50%.
   - **Oscillates around the target** → kP too high or kD too low.
   - **Smooth approach but undershoots by ~½ inch** → tighten exit
     conditions, OR add a tiny `kI` (start with `0.05`).
3. Edit `pid_drive_constants_set` in `autons.cpp`, rebuild, repeat.
4. Once 24" is clean, test 6", 12", 48", 72". A well-tuned PID handles all
   distances without retuning.

### Step 3 — tune heading PID

The heading PID is what keeps the robot pointed straight while the drive
PID is moving it. You tune it by deliberately starting a drive motion off-
heading:

1. Implement a heading test that turns to a non-zero heading and then
   commands a forward drive (see the `heading_test` example above).
2. Run it. The robot should drive a straight line at the *new* heading,
   not curve back to its original.
3. If the robot wobbles laterally as it drives → heading kD too low.
4. If the robot drifts to one side → heading kP too low.

Heading kP is typically 1.5–2× drive kP, and heading kD is typically
2–3× drive kD. The defaults (`17.0, 0.0, 70.0`) are tight; if your
robot is rangy, scale them down proportionally.

### Step 4 — tune turn PID

Same recipe as drive PID, but with `turn_test(90)`. Aim for clean 90°
turns first — they are the most common in autons. Then verify 45°, 180°,
270° all land within ±2°.

The turn PID's units are degrees, not inches, so its gains live in a
totally different range — typical values are `kP ≈ 5`, `kD ≈ 45`. Don't
expect drive constants to "look like" turn constants.

### Step 5 — tune swing PID

A swing turn pivots around one wheel, so it has roughly half the
rotational inertia of an in-place turn — its gains will be slightly
smaller than turn PID gains. Run `swing_test(90)` and tune with the same
method.

### Step 6 — tune the odom angular PID

Once all four basic PIDs are clean, tune the odom-specific ones. They
control the *angular* component of `pid_odom_set` motions. The
`pid_odom_angular_constants_set` PID is fundamentally different from the
turn PID because odom motion mixes turning and driving simultaneously, so
it tolerates a much smaller kP.

Run `pid_odom_set` to a point with a final heading (e.g. `(48, 24, 90)`)
and tune `pid_odom_angular_constants_set` until the robot arrives at the
right pose without spiraling or under-turning.

`pid_odom_boomerang_constants_set` is the angular control during the
*approach* portion of a boomerang motion. Tune it after the angular PID is
solid — same goal, but with the boomerang carrot active.

## Exit conditions

Exit conditions decide *when* a motion is finished. Loose conditions =
robot exits early and the next motion starts before settling. Tight
conditions = robot stalls forever trying to land within an unrealistic
tolerance.

```cpp
chassis.pid_drive_exit_condition_set(50_ms,  1_in,    // small_error
                                     250_ms, 3_in,    // big_error
                                     500_ms,          // velocity_exit
                                     500_ms);         // mech_timeout (mA)
```

Logic: the motion exits if **either**
- error stays within `small_error` for `small_exit_time`, **or**
- error stays within `big_error` for `big_exit_time`, **or**
- velocity stays at zero for `velocity_exit_time` (the robot stalled), **or**
- motors are over-current for `mech_timeout`.

Start with the shipped defaults. Only tighten `small_error` once the PID
is well-tuned; loose exit thresholds will mask poor PID gains.

## The live PID tuner: light::pid_tuner

LightLib ships an on-brain PID tuner that lets you adjust gains live
without rebuilding. To enable:

```cpp
// In opcontrol(), once at the start
light::pid_tuner.set_drive(&chassis);
light::pid_tuner.start_task();

// Bind a controller button to open it
if (master.get_digital_new_press(DIGITAL_X)) {
  light::pid_tuner.open();
}
```

When the tuner is open the brain screen shows tabs for DRIVE / TURN /
SWING / HEADING / EKF, each with editable kP/kI/kD/start_i values and a
live chart of left/right wheel output and error. Hit "Apply" to push
the values to the chassis; the changes take effect on the *next* PID
motion.

**Workflow with the tuner:**

1. Open the tuner.
2. Run an auton command (e.g. via a button-bound `drive_test(24)`).
3. Watch the chart. Adjust gains. Apply. Re-run the auton.
4. When you find values you like, transcribe them into
   `default_constants()` so they survive a program restart. **The tuner
   does not save to flash.**

The legacy EZ-Template tuner (`pid_tuner_enable()`, `pid_tuner_iterate()`)
is also still available on the chassis — but the LVGL tuner (`light::pid_tuner`)
is the supported one.

## Auto-tune (Z–N relay feedback)

LightLib includes an Åström–Hägglund relay-feedback auto-tuner. It
oscillates the chassis with a bang-bang voltage relay, measures the
sustained-oscillation amplitude `a` and period `Tu`, then applies the
classical Ziegler–Nichols formula:

```
Ku = 4·h / (π·a)
Pu = Tu
kP = 0.6 · Ku
kI = 2·kP / Pu
kD = kP·Pu / 8
```

The result is printf'd to the terminal AND pushed to the live chassis
PIDs immediately. You still have to transcribe the printed values into
`default_constants()` for them to survive a restart.

```cpp
// Run each as its own auton routine. The robot WILL move during these.
void run_autotune_drive() {
  light::autotune_drive_pid(/*reliefV=*/6.0f,
                            /*cycles=*/5,
                            /*timeoutMs=*/15000,
                            /*chunkCycles=*/2,
                            /*coolMs=*/5000);
}

void run_autotune_turn()  { light::autotune_turn_pid();  }
void run_autotune_swing() { light::autotune_swing_pid(); }

void run_autotune_heading() {
  light::autotune_heading_pid(/*forwardV=*/3.0f, /*reliefV=*/2.0f);
}
```

**Space requirements** — the routines are noted in
[ramsete.hpp](../../include/LightLib/path/ramsete.hpp):

- `turn` / `swing` — about 2 ft² (in-place oscillation).
- `drive` — at least 8 ft of clear straight space ahead.
- `heading` — at least 8 ft of clear lane; the robot drives forward at
  `forwardV` while the relay oscillates the heading around 0.

**`reliefV` must exceed `kS`** (the static-friction voltage). If the robot
never moves, the tune times out and aborts without overwriting the previous
PID values. Bumping `reliefV` solves this — but if you have to push it past
~7V the robot is too sticky for the algorithm and you should hand-tune.

The chunked-cooldown defaults (`chunkCycles=2, coolMs=5000`) protect the
motors from thermal damage during long tunes. Don't disable cooldowns
unless you're tuning indoors with a fan on the chassis.

**Use auto-tune as a starting point**, not a final answer. The Z–N
formulas favor robustness over performance; you can usually get 20–40%
faster motion by hand-trimming after an auto-tune.
