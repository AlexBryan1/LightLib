# Auton Config: Constants, Positions, and the Three-File Workflow

This tutorial covers the autonomous configuration surface — the three
files that, between them, decide *what* runs in autonomous, *how* the
robot moves while running it, and *what state* the mechanisms start in.
Once you understand the split, adding a new auton is mechanical.

## The three files

| File | Job |
|---|---|
| [`include/autons.hpp`](../../include/autons.hpp) | Declare every auton function — one line each. |
| [`src/autons.cpp`](../../src/autons.cpp) | Write the actual auton bodies, plus `default_constants()` and `default_positions()`. |
| [`src/auton_config.cpp`](../../src/auton_config.cpp) | Register each auton with the brain-screen picker. |

A new auton needs touches in all three. Skip any one and either it
won't compile, won't appear on the screen, or won't be callable from
other code.

## Workflow: adding `red_left_rush`

**1. Declare it in `autons.hpp`:**

```cpp
void red_left_rush();
```

**2. Write it in `autons.cpp`:**

```cpp
void red_left_rush() {
  light::setPose(Pose(36, 12, 0));

  chassis.pid_drive_set(36_in, 127, /*slew=*/true);
  chassis.pid_wait_until(20_in);
  Loader.set(true);
  chassis.pid_wait();

  chassis.pid_drive_set(-18_in, 110);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(90_deg, 90);
  chassis.pid_wait();
}
```

**3. Register it in `auton_config.cpp`:**

```cpp
light::auton_selector.add("Red Left",
                          "Rush goal, then ladder",
                          red_left_rush);
```

That's the whole loop. Recompile, flash, the button shows up on the
brain.

## `default_constants()` — the global tuning file

Called once from `initialize()`, before any auton runs.
[`default_constants()`](../../src/autons.cpp) is where every PID gain,
exit condition, slew rate, and odom knob lives. It's the *only* place
these should be set in code — anything you `pid_*_constants_set()`
elsewhere will silently override the global tune for the rest of the
match, which is almost never what you want.

The full set of knobs, with what each one actually does:

### PID gains

```cpp
chassis.pid_drive_constants_set(10.0, 0.1, 30.0);    // straight-line motion
chassis.pid_heading_constants_set(17.0, 0.0, 70.0);  // hold heading during drive
chassis.pid_turn_constants_set(5.0, 0.0, 45.0);      // in-place pivots
chassis.pid_swing_constants_set(4.0, 0.0, 45.0);     // single-side swings
chassis.pid_odom_angular_constants_set(1.0, 0.0, 10.0);    // odom drive-to-point heading
chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // boomerang curve
```

The format is `(kP, kI, kD)` for every one. See
[PID Tuning](03_pid_tuning.md) for the full tuning workflow — the short
version is "kP for response, kD for damping, kI almost always zero."

### Exit conditions

```cpp
//                                small_to  small_err  big_to  big_err  v_to  m_to
chassis.pid_turn_exit_condition_set(100_ms,  3_deg,    300_ms,  7_deg,  500_ms, 500_ms);
chassis.pid_drive_exit_condition_set(50_ms,  1_in,     250_ms,  3_in,   500_ms, 500_ms);
```

A motion exits when **either**:

- error stays within `small_error` for `small_timeout`, **or**
- error stays within `big_error` for `big_timeout`.

Plus two failsafes: `velocity_timeout` (motors stalled) and
`mech_timeout` (gave up entirely). Tighter exit conditions = more
accurate but slower. Loose ones = fast and chainable.

### Chain constants

```cpp
chassis.pid_drive_chain_constant_set(5_in);
chassis.pid_turn_chain_constant_set(10_deg);
chassis.pid_swing_chain_constant_set(15_deg);
```

How close the robot must be before `pid_wait_quick_chain()` exits and
queues the next motion. Larger = exit sooner, fluider transitions, less
final accuracy. These are the secret to autons that *look fast*.

### Slew

```cpp
chassis.slew_drive_constants_set(3_in, 30);
chassis.slew_turn_constants_set(10_deg, 55);
chassis.slew_swing_constants_set(3_in, 80);
```

Format: `(distance_to_ramp_over, starting_speed)`. The motor output is
clamped to start at `starting_speed` and ramp linearly up to whatever
the PID is asking for, over the given distance. Stops the wheels from
slipping on launch.

### Odom

```cpp
chassis.odom_turn_bias_set(1.0);              // 1.0 = full priority on turn
chassis.odom_look_ahead_set(15_in);           // pure-pursuit lookahead
chassis.odom_boomerang_distance_set(16_in);   // max carrot distance
chassis.odom_boomerang_dlead_set(0.625);      // 0=gentle, 1=sharp final
chassis.pid_angle_behavior_set(light::shortest);
```

See [Odom Motion](02_odom_motion.md) for what each of these knobs feels
like in practice.

### RAMSETE

```cpp
light::ramsete_configure(
    {/*b=*/2.0f, /*zeta=*/0.7f,
     /*trackWidthIn=*/11.5f, /*wheelDiamIn=*/3.25f, /*gearRatio=*/0.75f},
    {/*kS=*/0.60f, /*kV=*/0.18f, /*kA=*/0.03f, /*kP=*/0.02f},
    {/*vMax=*/48.0f, /*aMax=*/60.0f, /*aDecMax=*/60.0f, /*aLatMax=*/40.0f});
```

Three structs: geometry, feedforward + velocity P, default trajectory
constraints. The numbers come from the
[characterization routines](05_characterization.md). The defaults will
drive but won't track cleanly until you've replaced them with measured
values.

## `default_positions()` — pre-match piston state

```cpp
void default_positions() {
  Wings.set(true);
  Loader.set(true);
  MidGoal.set(false);
  Hood.set(true);
}
```

Called once from `initialize()`, before the match. Every piston the
robot has should be in here, in the state you want at the buzzer.
Forgetting one isn't a compile error — it's a piston that starts in
whatever state it happened to be in when you powered on, which on a
match field is "extended into another robot."

## `auton_config.cpp` — the registration file

```cpp
void register_autons() {
  light::auton_selector.add("Red Left",  "rush left side",  red_left_rush);
  light::auton_selector.add("Red Right", "safe right play", red_right_safe);
  light::auton_selector.add("Skills",    "60 s programming skills", skills_route);
}
```

This file does *one* thing: tell the brain-screen picker which buttons
exist. See [UI Config](06_ui_config.md) for what each button looks like
on screen and what the description shows up as.

**Order matters** — the first registered button is selected by default
on boot. Put your most-likely competition pick first.

### Lambdas for tuning autons

Tuning routines often take parameters or call into namespaces. Wrap
them in a lambda:

```cpp
light::auton_selector.add("Tune: Drive",
                          "Relay-tune drive PID (needs 8 ft)",
                          [] { light::autotune_drive_pid(); });

light::auton_selector.add("Drive 48",
                          "drive forward 48 in (PID test)",
                          [] { drive_test(48); });
```

The lambda is itself a `void()` — same signature as a regular auton, so
the selector doesn't care.

## A real example end-to-end

Goal: add a 15-point AWP auton called "Right AWP."

**`autons.hpp`:**
```cpp
void right_awp();
```

**`autons.cpp`:**
```cpp
void right_awp() {
  light::setPose(Pose(-36, 60, 270));
  allianceColor = RED;

  chassis.pid_drive_set(24_in, 110);
  chassis.pid_wait_until(8_in);
  Wings.set(true);             // pop wings to grab AWP ring
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180_deg, 90);
  chassis.pid_wait();

  Wings.set(false);
  chassis.pid_drive_set(-12_in, 90);
  chassis.pid_wait();
}
```

**`auton_config.cpp`:**
```cpp
void register_autons() {
  light::auton_selector.add("Right AWP", "15-pt AWP route", right_awp);
  // ...other autons below
}
```

Compile, flash, button appears on the brain. Done.

## Common gotchas

- **"My auton compiles but doesn't show on the brain."** You forgot
  step 3. `auton_config.cpp` is opt-in — the function existing isn't
  enough.
- **"My auton shows up but does nothing when I run it."** You
  registered the *declaration* address but forgot to define the body
  in `autons.cpp`. The linker will catch a missing body, so this is
  more often "the body has a `return;` at the top from earlier
  debugging."
- **"My PID values reset every reboot."** You tuned them in the
  brain-screen PID panel but never copied them into
  `default_constants()`. The panel is a scratchpad.
- **"Pistons start in random states."** You added a new piston to
  `subsystems.hpp` but forgot to add a line for it in
  `default_positions()`. Add every piston, every time.
- **"Routines drift differently on a fresh battery vs. a tired one."**
  This is what `slew_drive_constants_set` is for. Bumping the starting
  speed up too high makes a fresh battery slip; too low makes a tired
  battery feel sluggish. The default of 30 is a good compromise.
- **"My characterized RAMSETE numbers don't survive a reboot."** The
  characterization auton writes them to the live config, not to the
  source file. Transcribe the printed numbers into
  `ramsete_configure()` in `default_constants()` to make them
  permanent.
