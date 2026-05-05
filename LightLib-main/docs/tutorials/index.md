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

## Suggested order on a fresh build

```
characterization day  →  PID tuning  →  basic motion autons  →
odom motion autons    →  path following
```

The first two are "configure the robot" — they make every later motion
work. The last three are "use the robot."
