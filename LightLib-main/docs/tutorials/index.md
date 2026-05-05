# Tutorials

Step-by-step guides for the things you actually do with LightLib in
practice. Read in order if new; pick what you need otherwise.

1. **[Basic Motion](01_basic_motion.md)** — drive, turn, swing. The three
   primitives that every hand-crafted auton is built from. Covers the
   full `pid_*_set` / `pid_wait*` family and how to chain motions
   smoothly.

2. **[Odom Motion](02_odom_motion.md)** — driving to a field point
   instead of "24 inches forward." Covers the boomerang controller, how
   odometry actually works under the hood, and tuning the look-ahead /
   boomerang knobs.

3. **[PID Tuning](03_pid_tuning.md)** — full workflow for tuning all four
   chassis PIDs (drive, turn, swing, heading) from scratch. Covers the
   theory, the live on-brain tuner, exit conditions, and the
   relay-feedback auto-tune routines.

4. **[Path Following](04_path_following.md)** — RAMSETE trajectory
   following. Smooth curved paths, Jerryio CSV imports, and the
   per-tick `kV / kA / kS` feedforward.

5. **[Characterization](05_characterization.md)** — `characterize_*` and
   `autotune_*` routines. Once-per-build measurements that produce the
   real-robot constants every other tutorial assumes you have.

6. **[UI Config](06_ui_config.md)** — the on-brain auton selector.
   What each panel does (preview, live PID, live odom), how registered
   autons end up as buttons, and the run-screen takeover during
   autonomous.

7. **[Subsystems](07_subsystems.md)** — `subsystems.hpp` end-to-end.
   Adding motors, motor groups, and pistons; the `RotationalSnap` lift;
   alliance color; and the holonomic / H-drive opt-ins.

8. **[Auton Config](08_auton_config.md)** — the three-file workflow for
   adding an auton (`autons.hpp` / `autons.cpp` / `auton_config.cpp`),
   plus everything `default_constants()` and `default_positions()`
   control.

## Suggested order on a fresh build

```
characterization day  →  PID tuning  →  basic motion autons  →
odom motion autons    →  path following
```

The first two are "configure the robot" — they make every later motion
work. The last three are "use the robot."
