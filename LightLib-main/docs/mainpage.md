# LightLib {#mainpage}

A PROS library for VEX V5 — hybrid EKF/MCL localization, RAMSETE path
following, and a high-level chassis API.

## Tutorials

<div class="tutorial-grid">

<a class="tutorial-card" href="md_docs_2tutorials_201__basic__motion.html">
<div class="tutorial-num">01</div>
<div class="tutorial-title">Basic Motion</div>
<div class="tutorial-desc">Drive, turn, swing — the three primitives every hand-crafted auton is built from.</div>
</a>

<a class="tutorial-card" href="md_docs_2tutorials_202__odom__motion.html">
<div class="tutorial-num">02</div>
<div class="tutorial-title">Odom Motion</div>
<div class="tutorial-desc">Drive to field points instead of "24 inches forward." Boomerang controller and look-ahead tuning.</div>
</a>

<a class="tutorial-card" href="md_docs_2tutorials_203__pid__tuning.html">
<div class="tutorial-num">03</div>
<div class="tutorial-title">PID Tuning</div>
<div class="tutorial-desc">Full workflow for tuning all four chassis PIDs from scratch — live on-brain tuner included.</div>
</a>

<a class="tutorial-card" href="md_docs_2tutorials_204__path__following.html">
<div class="tutorial-num">04</div>
<div class="tutorial-title">Path Following</div>
<div class="tutorial-desc">RAMSETE trajectory following. Smooth curved paths, Jerryio CSV imports, kV/kA/kS feedforward.</div>
</a>

<a class="tutorial-card" href="md_docs_2tutorials_205__characterization.html">
<div class="tutorial-num">05</div>
<div class="tutorial-title">Characterization</div>
<div class="tutorial-desc">Once-per-build measurements that produce the real-robot constants every other tutorial assumes you have.</div>
</a>

</div>

**New here?** Read top-to-bottom. Otherwise pick what you need.

**Suggested order on a fresh build:** characterization → PID tuning →
basic motion → odom motion → path following.

## Quick build

```
pros c fetch okapilib
pros c fetch liblvgl
pros make
```

When library sources change, `make lightlib` rebuilds the static
library. `make clean` clears the build.

## Project at a glance

- **Control** — PID with anti-windup, slew limiter, on-brain LVGL tuner
- **Drive** — Tank + holonomic, arc odometry, 6-state EKF at 100 Hz, LightCast MCL at 10 Hz
- **Path** — Quintic Hermite splines, RAMSETE follower, named-path registry
- **UI** — LVGL auton selector with banner images, SD-card persistence
- **Util** — Rotational snap, auton timers, piston wrapper

For the full feature list and source layout, see the
[README on GitHub](https://github.com/AlexBryan1/LightLib).

## API reference

Browse the sidebar (or use the search box top-right) for the full API.
Useful entry points:

- **Classes** — every public class, indexed alphabetically
- **Files** — header-by-header view of `include/LightLib/`
- **Namespaces** — grouped by `LightLib::control`, `::drive`, `::path`, `::ui`, `::util`

## License

[MPL-2.0](https://github.com/AlexBryan1/LightLib/blob/main/LICENSE.md),
inherited from EZ-Template and the PROS kernel.
