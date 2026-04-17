# LightLib — Session Context

This file documents changes made and context gathered across Claude Code sessions so future sessions can pick up where we left off.

---

## Project Overview

LightLib is a VEX V5 robotics library built on top of **EZ-Template** and **PROS 4**. It provides:

- **Auton selector UI** — touchscreen GUI on the V5 Brain for picking autonomous routines, with a built-in PID tuner and live odom display
- **Odometry** — custom odom system (`light::getPose()`, `light::setPose()`)
- **Custom movement** — `WallRide()` (wall-tracking with distance sensors), `light::moveToPoint()` (odom drive-to-point)
- **Holonomic drive support** — `HoloDrive` (X-Drive/Mecanum) and `HDrive` (tank + center strafe wheel) classes, independent of EZ-Template
- **Drive utilities** — `drive_until_distance()`, `distance_reset()`, `angle_reset()` for sensor-based positioning

### Key directories

- `src/` — main source files (`autons.cpp`, `main.cpp`, robot config)
- `src/LightLib/` — library source files (auton_selector, cstm_move, ez_extra, holo_drive, odom, pid_tuner, etc.)
- `include/LightLib/` — library headers
- `include/` — EZ-Template, PROS, and okapi headers

### Dependencies

- **PROS 4** — VEX V5 runtime
- **EZ-Template 3.2.2** — tank drive PID, auton selector framework
- **LVGL** — graphics library for the V5 Brain touchscreen
- **okapi units** — `QLength` (`_in`, `_mm`, etc.) and `QAngle` (`_deg`) unit types

---

## Changes Made

### 1. `drive_until_distance()` — accept `okapi::QLength` (drive_utils.hpp)

**What:** Changed `drive_until_distance()` from taking a raw `double` (inches) to taking `okapi::QLength`, so callers can use `20_in` instead of just `20`.

**How:** Replaced the original `double target_in` parameter with `okapi::QLength target`. The function converts internally using `target.convert(okapi::inch)`.

**Files changed:**
- `include/LightLib/drive_utils.hpp` — changed function signature and body
- `src/autons.cpp` line 173 — updated call from `drive_until_distance(20)` to `drive_until_distance(20_in)`

**Note:** `okapi::QLength` has an `explicit` constructor, so plain `20` won't compile — you must use `20_in` (or `50.8_cm`, etc.).

### 2. `drive_until_distance()` — support driving backwards (drive_utils.hpp)

**What:** The function always drove forward (`pid_drive_set(5000, speed)`). Now it checks the sign of `speed` — negative speed drives backward.

**How:** Added `int drive_dir = (speed >= 0) ? 5000 : -5000;` before `pid_drive_set()`, and passes `std::abs(speed)` as the speed argument.

**Usage:**
- `drive_until_distance(20_in)` — forward at speed 127 (default)
- `drive_until_distance(20_in, *frontDist, -127)` — backward at speed 127

### 3. Comprehensive comments added to source files

Added detailed comments explaining architecture, usage, and internal logic to these files for onboarding new team members:

#### `src/LightLib/auton_selector.cpp`
- File header with ASCII layout diagrams of both screens (selector + run)
- Usage example (`add()` → `init()` → `run()`)
- Public API summary (add, init, run, show)
- Every builder function explained (build_ui, build_right_preview, build_right_pid, build_right_odom)
- PID tuner sub-panel ASCII layout
- Panel switching logic
- Every LVGL callback documented (what triggers it, what it does)
- Run screen layout diagram with side-strip buttons
- Animation explanations (zoom-in/zoom-out)

#### `src/LightLib/cstm_move.cpp`
- File header with ASCII diagram showing robot, sensor positions, and wall
- Step-by-step explanation of WallRide algorithm
- Setup instructions
- Parameter docs for WallRide()
- Cleaned up the turret example code at the bottom (was messy, now properly formatted)

#### `src/LightLib/ez_extra.cpp`
- File header with table of contents for all 6 functions
- `ez_screen_task()` — how to start it, what it shows in practice vs competition
- `ez_template_extras()` — controller button mappings for PID tuner
- `checkMotorTemp()` — explains the rumble warning
- `turret_reset()` — step-by-step of re-homing sequence
- `light::moveToPoint()` — detailed 8-step algorithm explanation, cosine scaling trick, proportional clamping, tuning guidance

#### `src/LightLib/holo_drive.cpp`
- File header with ASCII diagrams for both drive types (HoloDrive 4-motor layout, HDrive tank+center layout)
- Setup instructions (construct → calibrate → set PIDs → use)
- PID controller roles explained (drive, strafe, turn, heading)
- Exit condition logic (settled counter)
- Motor mixing equation with diagram
- `drive_pos()` and `strafe_pos()` encoder math explained
- HDrive-specific notes (center wheel only strafes, tank sides provide heading correction)

---

## Things NOT changed (left as-is)

- `autons.cpp` — already had good comments from a previous session; only updated the `drive_until_distance(20)` → `drive_until_distance(20_in)` call
- Header files (`.hpp`) — already well-documented in the header; comments were added to the `.cpp` implementation files
- `odom.cpp`, `pid_tuner.cpp` — not touched this session

---

## Known TODOs / future work

- `cstm_move.cpp` — `WALL_KP` and `WALL_KD` are marked `// TODO: tune for your robot`
- `ez_extra.cpp` — `moveToPoint()` PID constants are marked `// TODO: tune these PID constants for your robot`
- The turret functions (`turret_reset`, `track_basket`) reference a global `turret` motor that must be declared in `subsystems.hpp`
