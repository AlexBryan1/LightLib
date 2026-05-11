# UI Config: The Brain-Screen Auton Selector

This tutorial covers the on-brain UI — the LVGL screen that lets you pick
an auton, watch live odom, and tweak PID gains without rebuilding. You
won't touch its internals often; what matters is knowing what each panel
shows and how the selector you populate in `auton_config.cpp` ends up on
the screen.

## What the UI does

The `light::AutonSelector` singleton is the default LightLib brain
screen. It's built once during `initialize()` and stays alive for the
entire match. It has three jobs:

1. **Pick an auton.** Every routine you register with
   `light::auton_selector.add(...)` becomes a button on the left side of
   the screen.
2. **Show live odometry.** The right-hand panel can be flipped to display
   `x`, `y`, and `theta` updating in real time — useful for sanity-checking
   pose resets and tracking drift before a match.
3. **Edit PID live.** The same right-hand panel has a tab for the four
   chassis PIDs (drive, turn, swing, heading). You can bump gains up or
   down with on-screen +/- buttons and apply them without re-flashing.

A fourth view — the **run screen** — takes over the whole display while
the selected auton runs, then drops back to the picker when it finishes.

## Layout at a glance

```
┌──────────────────────────┬───────────────────────────────┐
│  Auton 1                 │  [Preview / PID / Odom]       │
│  Auton 2  ← selected     │                               │
│  Auton 3                 │   X:    0.00 in               │
│  Auton 4                 │   Y:    0.00 in               │
│  ...                     │   θ:    0.00°                 │
│                          │                               │
│                          │   [toggle panel button]       │
└──────────────────────────┴───────────────────────────────┘
```

- **Left column** — scrollable list of registered autons. Tap to select.
  The selected button is highlighted.
- **Right column** — one of three modes:
  - *Preview* — description text and an optional banner image you passed
    to `add(...)`.
  - *PID* — four tabs (Drive / Turn / Swing / Heading), each with kP, kI,
    kD, and a target field, plus +/- buttons and an Apply button.
  - *Odom* — live `x`, `y`, `theta` readout, refreshed by an LVGL timer.
- **Toggle button** — bottom of the right column, cycles
  Preview → PID → Odom → Preview.

## Registering autons (the only file you'll edit)

Every brain-screen button comes from one `add(...)` call in
[`src/auton_config.cpp`](../../src/auton_config.cpp):

```cpp
void register_autons() {
  light::auton_selector.add("Red Left",
                            "Rush goal, then alliance ladder",
                            red_left_rush);

  light::auton_selector.add("Skills",
                            "60 s programming skills route",
                            skills_route);

  // Lambda is fine when you want to wrap a function with arguments
  light::auton_selector.add("Tune: Drive",
                            "Relay-tune drive PID (needs 8 ft)",
                            [] { light::autotune_drive_pid(); });
}
```

**Three arguments:**

1. **Button label** — shown on the picker. Keep it short — long labels
   wrap and look bad. "Red Left Rush" is fine; a sentence isn't.
2. **Description** — shown in the *Preview* panel when that button is
   selected. Use it for reminders ("expect 4 rings, watch the wall").
3. **Function** — any `void()` function. Must be declared in
   `autons.hpp` first (or be a lambda).

There's a four-arg overload that takes an `lv_img_dsc_t*` banner
image — the picker scrolls it inside the button. Use it sparingly;
images live in flash and bloat your build.

### Order matters

Buttons appear in registration order. The first `add(...)` call ends up
selected by default on boot. Put your most likely competition auton
first — if you forget to pick before the match starts, that's what runs.

### Don't register tuning autons in competition

Each `add()` is one button on the screen, and the selector cycles by
*tap*, not by some safety mechanism. If you leave `Tune: Drive`
registered and accidentally tap it before a match, your robot ramps full
forward at the wall. Wrap tuning entries in `#ifdef TUNING` or
just comment them out before the event:

```cpp
#ifdef TUNING
  light::auton_selector.add("Tune: Drive", "...", [] { light::autotune_drive_pid(); });
#endif
```

## The PID panel — what the +/- buttons actually do

The PID tab reads the *current* gains from the chassis at boot (so
whatever `default_constants()` set is what you see), then lets you nudge
each value with the +/- buttons. Pressing **Apply** writes the displayed
values back into the chassis via the same `pid_*_constants_set()` calls
you'd use in code.

This is the fastest way to tune by hand:

1. Run a tuning routine (e.g. `drive_test(48)`).
2. Watch the robot. Oscillating? Bump kD. Sluggish? Bump kP.
3. Apply, run again. No rebuild, no re-flash.

When you've found values you like, **transcribe them into
`default_constants()`** in [`src/autons.cpp`](../../src/autons.cpp). The
panel is a scratchpad — its values reset on the next reboot.

> The auto-tune routines (`Tune: Drive`, etc.) write their results
> straight into the live chassis the same way the panel does — same
> caveat: copy them into `default_constants()` to make them permanent.

## The odom panel — what to watch for

`x`, `y`, `theta` come straight from `chassis.odom_x_get()` and friends,
sampled by an LVGL timer (~100 ms). Two things to check before a match:

- **Numbers move when you push the robot.** If `x` and `y` stay at zero
  while you shove it across the field, your tracking wheels are dead or
  miswired.
- **Numbers settle when you stop.** Drift while parked usually means an
  un-calibrated IMU or a slipping tracking-wheel encoder.

The display caches its own last-rendered text and skips
`lv_label_set_text` when nothing changed — so if a value is *visibly*
frozen, the underlying number is genuinely not moving (it's not a UI
glitch).

## The run screen

When `light::auton_selector.run()` is called from `autonomous()`, the UI
swaps to a full-screen view that:

- Hides the picker (so you can't accidentally tap during the match).
- Shows the banner image (if any) or the live odom readout.
- Drops back to the picker automatically when the routine returns.

You don't configure this screen; it inherits whatever description and
banner you passed to `add(...)`. The toggle button on the run screen
flips between the image view and the live odom view, mid-match — handy
if you want to see where the robot *thinks* it is while it's driving.

## Tying it all together

The full UI lifecycle, end to end:

```cpp
// main.cpp — already wired by the LightLib boot code
void initialize() {
  default_constants();
  default_positions();
  register_autons();              // ← your buttons get added here
  light::auton_selector.init();   // build the LVGL screen
  light::auton_selector.show();   // make it the active screen
  user_initialize();
}

void autonomous() {
  user_autonomous();
  light::auton_selector.run();    // invoke whichever button was tapped
}
```

So the only file *you* touch for UI config is
[`src/auton_config.cpp`](../../src/auton_config.cpp). Everything else is
plumbing that's already correct.

## Common gotchas

- **"My new auton doesn't show up."** You declared it in `autons.hpp`
  and wrote it in `autons.cpp`, but forgot to `add(...)` it in
  `auton_config.cpp`. The selector is opt-in.
- **"PID changes don't stick."** They reset every reboot. Copy the
  values from the panel into `default_constants()`.
- **"The selector is blank."** `register_autons()` was never called, or
  the function body is empty. Confirm `initialize()` calls it.
- **"My banner image crashes the brain."** `lv_img_dsc_t` arrays are
  unforgiving — wrong size or stride will hard-fault LVGL. Generate them
  with the LVGL image converter; don't hand-author.
