# Subsystems: Motors, Pistons, and Mechanisms

This tutorial covers [`include/subsystems.hpp`](../../include/subsystems.hpp) —
the single header where every motor, sensor, and pneumatic on the robot
is declared. If a subsystem isn't in this file, no other file can see
it. If it *is* in this file, every `.cpp` automatically gets access to
it without extra includes.

## Why one file

Robot code has two natural levels: "what hardware exists" (subsystems)
and "what to do with it" (autons, opcontrol). LightLib keeps the first
level in *one* place using `inline` globals:

```cpp
// subsystems.hpp
inline pros::Motor Top(7);
inline pros::Motor Bottom(17);
inline pros::MotorGroup Score({17, 7});
inline light::Piston Wings('A');
```

Because they're `inline`, every translation unit that includes this
header sees the *same* object — no `extern`, no separate `.cpp`
definition. Add a motor here, use it in `autons.cpp` and `main.cpp`
without touching anything else.

> The drive chassis is the one exception: it's built in `main.cpp` from
> the `#define`s at the top of that file (so the motor-port macros are
> visible at construction time) and re-exposed here as
> `extern Drive chassis;`.

## Adding a motor

```cpp
inline pros::Motor Intake(11);          // forward
inline pros::Motor Conveyor(-12);       // negative port = reversed
inline pros::MotorGroup Flywheel({13, -14});   // group of two motors
```

Three things to know:

1. **Sign reverses direction.** `pros::Motor(-12)` is port 12, reversed.
   Don't hand-flip signs in your code — set it once at declaration and
   trust it everywhere else.
2. **`MotorGroup` runs multiple motors as one.** Calling
   `Score.move(127)` runs both motors in `{17, 7}` at full forward.
   You can still address them individually as `Top` and `Bottom` if
   needed (this is exactly what the template does for the scoring
   roller).
3. **Pick a cartridge that matches the mechanism.** A flywheel wants
   blue (600 RPM), a lift wants green (200 RPM) or red (100 RPM). The
   `pros::Motor` constructor doesn't know — you set the gearset later
   with `Intake.set_gearing(pros::E_MOTOR_GEAR_GREEN)` if it's not the
   default red.

### Common methods

```cpp
Intake.move(127);                    // -127 … 127 (signed PWM-ish)
Intake.move_voltage(12000);          // -12000 … 12000 mV
Intake.brake();                      // zero output, brake mode active
Intake.get_temperature();            // °C — > 55 is warm, > 70 is throttled
Intake.get_actual_velocity();        // measured RPM
Intake.set_brake_mode(MOTOR_BRAKE_HOLD);  // COAST / BRAKE / HOLD
```

`set_brake_mode` is the one most teams forget. A lift on `COAST` falls
under gravity; a lift on `HOLD` stays put. The template's `Lift` snaps
to a preset and *needs* HOLD — that's why `user_initialize()` in
`main.cpp` sets it explicitly.

## Adding a pneumatic

```cpp
inline light::Piston Wings('A');     // 3-wire port A
inline light::Piston Loader('C');    // 3-wire port C
```

`light::Piston` is a thin wrapper around `pros::adi::DigitalOut` that
remembers its toggle state. Two methods you'll use:

```cpp
Wings.set(true);                              // extend
Wings.set(false);                             // retract
Wings.button_toggle(master.get_digital(DIGITAL_B));  // flip on rising edge
```

`button_toggle` is the magic — it tracks "was this button pressed last
loop?" so a held button doesn't rapid-fire toggle. The whole opcontrol
loop in `main.cpp` is built around this:

```cpp
Wings.button_toggle(master.get_digital(DIGITAL_B));
Loader.button_toggle(master.get_digital(DIGITAL_Y));
```

Press B once → wings extend. Press B again → retract. Held B → no
oscillation.

### Setting safe starting positions

The pistons need to be in a known state at the start of every match.
That's `default_positions()` in [`src/autons.cpp`](../../src/autons.cpp):

```cpp
void default_positions() {
  Wings.set(true);
  Loader.set(true);
  MidGoal.set(false);
  Hood.set(true);
}
```

Called once from `initialize()`, before either `autonomous()` or
`opcontrol()`. Put every piston in here with the state you want at the
sound of the buzzer.

## The `RotationalSnap` lift

The template includes a higher-level subsystem — a lift that snaps to
preset angles when the operator releases the joystick. The declaration
shows the pattern for any custom mechanism:

```cpp
inline pros::Motor LiftMotor(0);    // 0 = disabled until you wire it up
inline pros::Rotation LiftRot(0);

inline light::RotationalSnap Lift(
    LiftMotor, LiftRot,
    /* snap_angles_deg = */ {0.0, 45.0, 90.0, 135.0, 180.0},
    /* kP             = */ 1.5,
    /* tolerance_deg  = */ 1.0,
    /* max_snap_speed = */ 80);
```

How it works:

- While the operator is actively driving it (joystick deflected, or
  `DIGITAL_UP`/`DIGITAL_DOWN` held), it runs the motor manually.
- The instant the operator stops, it picks the *closest* angle from
  `snap_angles_deg` and runs a P-controller until it's within
  `tolerance_deg`, then holds.

In `opcontrol()`, you feed it one signed input per loop:

```cpp
int liftInput = master.get_analog(ANALOG_RIGHT_Y);
if (master.get_digital(DIGITAL_UP))    liftInput = 100;
else if (master.get_digital(DIGITAL_DOWN)) liftInput = -100;
Lift.update(liftInput);
```

The `0`-port motor / sensor declarations are placeholders. Until you
swap them for real ports, the lift is harmless: PROS treats port 0 as
a no-op device.

### Tuning the snap

- **`kP`** — too low, the lift creeps to the snap angle. Too high, it
  overshoots and oscillates. Start at 1.0–2.0.
- **`tolerance_deg`** — how close is "close enough." 1° is tight; 3° is
  forgiving and won't hunt forever near the target.
- **`max_snap_speed`** — clamps the motor output during snapping (out
  of 127). 80 is gentle; 127 is a slap.

## Alliance color

```cpp
enum Colors { BLUE = 0, NEUTRAL = 1, RED = 2 };
extern Colors allianceColor;
```

A single global the color-sort logic reads to decide which rings to
reject. Set it at the top of your auton or in `user_autonomous()`:

```cpp
void user_autonomous() {
  allianceColor = RED;       // sort out blue rings
}
```

Not setting it leaves it as `NEUTRAL` (sort nothing), which is fine but
means your color-sort code is doing nothing useful.

## Holonomic / non-tank drives

The template assumes a tank drive (left side / right side). If you're
on a mecanum, X-drive, or H-drive instead, the bottom of
`subsystems.hpp` has commented-out skeletons:

```cpp
inline light::HoloDrive holoDrive(
    FL_PORT, FR_PORT, BL_PORT, BR_PORT,
    IMU_PORT,
    WHEEL_DIAMETER,
    GEAR_RATIO,
    light::HoloDrive::Type::MECANUM);   // or XDRIVE
```

```cpp
inline light::HDrive hDrive(
    {LEFT_PORTS}, {RIGHT_PORTS}, CENTER_PORT,
    IMU_PORT, WHEEL_DIAMETER, GEAR_RATIO);
```

Uncomment ONE block, fill in the ports, and set `DRIVE_TYPE` in
`main.cpp` to 4 (mecanum/X) or 5 (H-drive). Both expose a similar API —
`drive`, `strafe`, `turn_to`, plus per-axis PID setters and an
`opcontrol(throttle, strafe, turn)` call you put in your while loop.

> You can run a holonomic drive *alongside* the tank chassis (e.g. a
> dev-bot with a strafe pod), but only one set of motors should answer
> to the joystick at a time. Use `DRIVE_TYPE` to pick.

## A complete example: adding a flywheel

Suppose you want a two-motor flywheel on ports 13 and 14 (one
reversed), runnable from `R1` / `R2`.

**1. Declare in `subsystems.hpp`:**

```cpp
inline pros::Motor FlywheelTop(13);
inline pros::Motor FlywheelBot(-14);
inline pros::MotorGroup Flywheel({13, -14});
```

**2. Use in `main.cpp` opcontrol:**

```cpp
if (master.get_digital(DIGITAL_R1))      Flywheel.move(127);
else if (master.get_digital(DIGITAL_R2)) Flywheel.move(64);
else                                     Flywheel.move(0);
```

**3. Use in autons.cpp:**

```cpp
void corner_shot() {
  Flywheel.move(127);
  pros::delay(1500);                  // spin up
  Loader.set(true);
  pros::delay(500);
  Loader.set(false);
  Flywheel.move(0);
}
```

That's it — three files touched, two of them only on lines you'd be
writing anyway.

## Common gotchas

- **"My motor isn't moving."** Port 0. PROS silently no-ops port 0 so
  the placeholders compile; if you forgot to set the real port, that's
  why your motor sits there. (Hot tip: check the brain's port screen —
  port 0 motors don't appear at all.)
- **"My piston is on backwards."** Pneumatic plumbing decides which
  state is "extended" vs. "retracted." Either re-plumb or treat
  `set(false)` as your "go" call. Don't fight it in code by sprinkling
  `!`s; pick a convention and document it in the comment next to the
  declaration.
- **"My motor group is reversing the wrong motor."** The signs in
  `MotorGroup({13, -14})` are *independent* of the signs you used for
  `pros::Motor FlywheelBot(-14)` above — the group rebuilds its own
  motor objects from the ports you give it. Pick one place to declare
  the reversal and don't double up.
- **"Compile error: multiple definition of `Wings`."** Something else
  in the codebase defined `Wings` non-inline. Almost always a copy-paste
  of an old `extern` declaration; delete the duplicate.
