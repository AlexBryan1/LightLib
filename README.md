# LightLib

A PROS library for VEX V5: hybrid EKF/MCL localization, RAMSETE/Pure-Pursuit path following, and a high-level chassis API.

LightLib started as a fork of [EZ-Template](https://github.com/EZ-Robotics/EZ-Template) and grew into a full localization + motion-planning stack on top of it.

## Build

```
pros c fetch okapilib
pros c fetch liblvgl
pros make
```

`pros make` (default `quick` target) produces a hot-package binary. When library sources change, `make lightlib` rebuilds [firmware/liblightlib.a](firmware/liblightlib.a). `make clean` clears the build.

## Dependencies

PROS template versions (from [project.pros](project.pros)):

- **kernel** 4.1.1
- **okapilib** 4.8.0
- **liblvgl** 8.3.8

## Tutorials

Step-by-step guides for the things you actually do in practice:

- [Basic Motion](docs/tutorials/01_basic_motion.md) — drive, turn, swing
- [Odom Motion](docs/tutorials/02_odom_motion.md) — driving to a field point
- [PID Tuning](docs/tutorials/03_pid_tuning.md) — full tuning workflow
- [Path Following](docs/tutorials/04_path_following.md) — RAMSETE & Jerryio
- [Characterization](docs/tutorials/05_characterization.md) — `characterize_*` and `autotune_*`

## Quick start

[src/main.cpp](src/main.cpp) is the configuration entry point. Drivetrain ports, wheel size, drive style, and EKF/MCL tuning constants are all `#define`s at the top of the file. After those, `#include "LightLib/robot_impl.inl"` instantiates the chassis, IMU, trackers, distance sensors, EKF, and LightCast from those defines.

User code goes in three hooks:

- `user_initialize()` — extra subsystem setup; runs after the chassis and selector are ready.
- `user_autonomous()` — pre-match setup (e.g. set alliance color) before the selected routine runs.
- `opcontrol()` — main teleop loop; call `_run_drive()` for joystick control and map your buttons.

Autonomous routines live in [src/autons.cpp](src/autons.cpp) and are registered in [src/auton_config.cpp](src/auton_config.cpp).

## Features

**Control** — [include/LightLib/control/](include/LightLib/control/)
- PID controller with integral-start threshold and anti-windup
- Slew rate limiter for smooth acceleration
- On-brain LVGL tuner for drive / turn / swing / heading PID and EKF noise constants

**Drive** — [include/LightLib/drive/](include/LightLib/drive/)
- Tank chassis with PID for drive, turn, swing, and arcs
- Holonomic drive (mecanum / X-drive) with internal angle PID
- Arc-based odometry fusing tracking wheels, IMU(s), and GPS
- 6-state Extended Kalman Filter at 100 Hz (`x, y, θ, vx, vy, ω`)
- LightCast Monte-Carlo localization at 10 Hz — particle filter using distance-sensor ray-casts; the EKF snaps to it when divergence is detected
- Custom moves: wall-tracking primitives using paired distance sensors

**Path** — [include/LightLib/path/](include/LightLib/path/)
- Quintic Hermite spline generator over 2D waypoints
- Time-parameterized trajectories (pose, velocity, acceleration) sampled at 10 ms
- RAMSETE feedback follower with `kS / kV / kA` feedforward
- Named-path registry for Jerryio-exported paths
- Static VRC field map (12×12 ft perimeter) for the LightCast sensor model

**UI** — [include/LightLib/ui/](include/LightLib/ui/)
- LVGL auton selector with optional scrolling banner images per routine
- SD-card persistence

**Util** — [include/LightLib/util/](include/LightLib/util/)
- Rotational-snap subsystem (joystick control + auto-snap to nearest preset angle)
- Auton timer helpers (`auton_elapsed`, `wait_until_auton`)
- Piston wrapper with default-state initialization
- EZ-Template integration shim for routing odom through the fused pose

## Project layout

```
include/LightLib/   library headers (control, drive, path, ui, util)
src/LightLib/       library implementation
src/main.cpp        drivetrain config + user hooks
src/autons.cpp      autonomous routines
src/auton_config.cpp  selector wiring
firmware/           prebuilt static libs (libpros, okapilib, liblvgl, liblightlib)
```


Claude was used to make the Doxygen Commenting and Read Me, because I didn't want to reformat it all...


## License

[MPL-2.0](LICENSE.md), inherited from EZ-Template and the PROS kernel.
