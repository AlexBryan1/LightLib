# Characterization & State-Estimator Tuning

This tutorial covers the routines that *measure* your robot — its motors,
its geometry, and its sensor noise — and write the results into the
follower configs. Each is meant to be run **as its own selectable auton**
and produces a printout you transcribe into `default_constants()`.

Run these once per robot build. After a major hardware change (new
gearing, new wheels, different battery), re-run.

## 1. Drive feedforward — `characterize_kV_kA_kS`

### Theory

A real DC motor under load follows roughly:

```
V = kS·sgn(v) + kV·v + kA·a
```

- **kS** (volts) — the static-friction voltage. Anything below kS produces
  zero motion. Bigger robots, more drag, higher kS.
- **kV** (V per in/s) — the steady-state voltage per unit velocity. The
  reciprocal of "max velocity at 12 V."
- **kA** (V per in/s²) — voltage needed to accelerate. Reflects the
  effective rotational inertia.

Identifying these three constants is **system identification** — given
voltage in, velocity out, find the parameters that best fit. The classic
method is a quasi-static voltage ramp: command voltage that increases
slowly (so acceleration is approximately zero), measure velocity, fit a
line:

```
V = kS·sgn(v) + kV·v   (when a ≈ 0)
```

Then a separate trapezoidal velocity step gives kA.

LightLib's `characterize_kV_kA_kS` does both in one routine. It commands a
voltage ramp from 0 V to `maxVoltage`, samples (V, v, a) over the ramp,
and runs a least-squares fit on the model `V = kS·sgn(v) + kV·v + kA·a`.

### Running it

```cpp
// In autons.cpp, register as a selectable auton
void run_char_kVkAkS() {
  // maxVoltage = peak voltage during the ramp
  // rampVps    = how fast voltage rises (V per second)
  light::characterize_kV_kA_kS(/*maxVoltage=*/10.0f, /*rampVps=*/0.25f);
}
```

```cpp
// In auton_config.cpp
light::auton_selector.add("CHAR kV/kA/kS",
                          "characterize feedforward",
                          run_char_kVkAkS);
```

**Space requirement:** straight clear lane, **at least 8 ft long**. The
robot accelerates progressively and needs room to reach steady state at
each voltage. Too short and the steady-state portion of the fit will be
noisy.

### Reading the output

The routine printf's the fitted gains to the PROS terminal. Expect
something like:

```
[characterize_kV_kA_kS]  kS = 0.61 V   kV = 0.182 V/(in/s)   kA = 0.029 V/(in/s²)
[characterize_kV_kA_kS]  R² = 0.987     n_samples = 412
```

R² above 0.95 is good. Below 0.85 means the fit is suspect — usually
caused by wheel slip on launch (raise `rampVps` to slow the start) or by
running on too short a lane.

Transcribe into `initialize()`:

```cpp
light::ramsete_configure(
    {2.0f, 0.7f, 11.5f, 3.25f, 0.6f},
    {/*kS=*/0.61f, /*kV=*/0.182f, /*kA=*/0.029f, /*kP=*/0.02f},
    {/*vMax=*/48.0f, /*aMax=*/60.0f, /*aDecMax=*/60.0f, /*aLatMax=*/40.0f}
);
```

## 2. Track width — `characterize_track_width`

### Theory

The follower needs to know the *effective* distance between left and right
wheels — not the physical track width, but the value that makes the
kinematic equation `ω = (v_right − v_left) / trackWidth` true. Wheel scrub
on omni / mecanum tracking wheels means the effective value is usually
1–4% larger than the geometric measurement.

The routine spins the chassis in place for `rotations` complete turns and
measures:

- Left wheel travel `L_left` (signed, inches)
- Right wheel travel `L_right` (signed, inches)
- Total IMU yaw change `Δθ` (radians)

Then solves:

```
trackWidth = (L_right − L_left) / Δθ
```

Spinning multiple revolutions averages out the IMU drift and any
single-wheel scrub events, giving a clean measurement.

### Running it

```cpp
void run_char_trackwidth() {
  float w = light::characterize_track_width(/*rotations=*/10);
  printf("Effective track width = %.4f in\n", w);
}
```

**Space requirement:** about 2 ft² — the robot stays in place. Make sure
the wheels aren't on a seam in the foam or a slick spot; both will skew
the result.

Update the `RamseteConfig.trackWidthIn` in `ramsete_configure()` with the
printed value.

## 3. Lateral acceleration — `characterize_a_lat_max`

### Theory

`aLatMax` is the centripetal acceleration the chassis can sustain in a
turn before sliding. It governs how aggressively the trajectory generator
slows down through curves: `v² / R ≤ aLatMax`.

Too high → robot scrubs out of corners and the trajectory follower bails
on pose error. Too low → robot crawls through curves.

The routine drives a tightening circle, increasing centripetal acceleration
until the robot starts losing traction (detected as wheel-encoder velocity
exceeding the IMU-derived expected velocity, i.e. wheels spinning faster
than the body is moving).

### Running it

```cpp
void run_char_alat() {
  float alat = light::characterize_a_lat_max();
  printf("Max lateral accel = %.2f in/s^2\n", alat);
}
```

**Space requirement:** open area at least 6 ft × 6 ft. The routine drives
in circles of varying radius.

Update `TrajConstraints.aLatMax` in `ramsete_configure()`.

## 4. Friction model — `characterize_friction`

### Theory

The basic kS+kV feedforward fits a *linear* friction model. Real motors
have a quadratic component too, especially at high speed where windage
becomes significant:

```
V = kS·sgn(v) + kV·v + kQ·v·|v|
```

`characterize_friction` sweeps voltage open-loop, fits all three
parameters, and applies the result to the **non-trajectory PIDs** (drive,
turn, swing) via `chassis.friction_constants_set(kS, kV, kQ)`. This is a
separate friction model from the RAMSETE feedforward — RAMSETE uses
`kS / kV / kA` from `characterize_kV_kA_kS`; the regular drive PIDs use
`kS / kV / kQ` from `characterize_friction`.

Once applied, the regular PID drive motions feed-forward the predicted
voltage and only use kP/kI/kD to correct residual error. Result: less
overshoot, better tracking on long drives.

### Running it

```cpp
void run_char_friction() {
  // maxVoltage = peak voltage in the sweep
  // stepV      = voltage increment between sweep points
  light::characterize_friction(/*maxVoltage=*/10.0f, /*stepV=*/0.5f);
}
```

**Space requirement:** straight 8+ ft clear lane.

If an SD card is present, raw samples are written to
`/usd/friction_id.csv` so you can re-fit offline if the on-brain fit looks
wrong.

The fitted constants are pushed into the chassis live; they survive until
the program restarts. To make them permanent, transcribe into
`default_constants()`:

```cpp
chassis.friction_constants_set(0.61, 0.182, 0.0014);
```

## 5. EKF process noise — `autotune_ekf_noise`

### Theory

The EKF needs to know how noisy the *motion model* is — how much the
predicted pose can drift between sensor updates. This is the `Q` matrix.
Too low → filter trusts the model too much, ignores sensor corrections,
locks onto wrong pose. Too high → filter trusts every noisy sensor sample,
output is jittery.

The routine measures `Q` directly: park the robot, sample odometry for a
few seconds, compute the per-tick variance of `(x, y, θ)`. Anything that
changes while the robot is stationary is, by definition, noise.

### Running it

```cpp
// Park the robot on the field. Don't bump it.
void run_autotune_ekf() {
  light::autotune_ekf_noise(/*sampleMs=*/10,
                            /*durationMs=*/5000,
                            /*warmupMs=*/500);
}
```

**Critical:** the robot must be **completely still** during the sample
window. Don't run this while the field is being set up around it. The
`warmupMs` parameter discards an initial transient — useful because the
IMU outputs a small bias spike for ~500 ms after starting.

Pushed into the live EKF via `setConfig()` immediately. To persist, copy
the printed `Q` values into your `MCLConfig` definition (in
`main.cpp`'s `EKF_*` macros).

## 6. MCL sensor noise — `autotune_mcl_noise`

### Theory

LightCast's particle filter weights particles by how well their predicted
distance-sensor readings match the actual ones. The weighting model
assumes Gaussian noise with standard deviation `sensorSigmaIn`. If you set
this too small, even small sensor noise causes the wrong particles to
dominate. Too large, and the filter ignores good measurements.

The routine samples each registered distance sensor for a few seconds
while the robot is parked, computes the std-dev of raw readings (in
inches), and averages across sensors. It then sets `sensorSigmaIn` to that
value and `outlierGapIn` to ~3·sigma (the threshold below which a reading
is treated as an outlier — a game element in front of the wall).

### Running it

```cpp
// Park the robot with every distance sensor pointed at a wall in range.
void run_autotune_mcl() {
  light::autotune_mcl_noise(/*sampleMs=*/20,
                            /*durationMs=*/4000,
                            /*warmupMs=*/300);
}
```

Same stillness requirement as `autotune_ekf_noise`. Result is pushed into
the live MCL config; transcribe printed values into your `MCL_*` macros
in main.cpp to persist.

## Putting it all together — characterization day

When you build a new chassis or change drivetrain hardware, run these in
order. About 15 minutes total, not counting cooldowns.

```
1. Park robot on a clean section of the field.
2. Run "CHAR kV/kA/kS"            → kS, kV, kA              (2 min, 8 ft lane)
3. Run "CHAR track width"          → trackWidthIn            (1 min, 2 ft²)
4. Run "CHAR a_lat_max"            → aLatMax                 (3 min, 6×6 ft)
5. Run "CHAR friction"             → kS, kV, kQ for PIDs     (2 min, 8 ft lane)
6. Park robot. Run "CHAR EKF noise"→ ekfQ*                   (5 sec)
7. Pose distance sensors at walls. Run "CHAR MCL noise"      (4 sec)
8. Transcribe all printed values into initialize() / default_constants().
9. Rebuild, flash, retest one of your existing autons.
```

## Common pitfalls

**`kV` looks too high (robot is slow at full voltage).** Check that
`gearRatio` in `RamseteConfig` is right. `gearRatio = wheel_rpm /
motor_rpm` — for a 36:60 blue cart driving 36-tooth wheels, that's
`60/100 × 36/36 = 0.6`. Wrong gear ratio scales kV by the same factor.

**Track width measurement comes out impossibly large (>2× the geometric
width).** IMU not zeroed, or IMU port is wrong. Verify with
`pros::Imu::get_heading()` after a manual 360° push.

**Friction characterization printout has crazy huge kQ.** The fit picked
up wheel slip as quadratic friction. Re-run with lower `maxVoltage`.

**EKF noise comes back as zero.** The warmup window swallowed all your
samples. Check `durationMs > warmupMs + 100` and that the IMU is alive.

**MCL noise wildly different across sensors.** One of your distance
sensors is partly occluded or pointing at a curve where small angles
produce big distance changes. Re-aim before re-running.
