#pragma once

/**
 * \file pure_pursuit.hpp
 *
 * Pure Pursuit follower configuration.
 *
 * The Pure Pursuit follower itself is invoked through the same
 * `light::followTrajectory` / `runJerryioPath` / `runPath` entry points as
 * RAMSETE — pass `light::Follower::PURE_PURSUIT` (the default) as the
 * trailing argument. See `ramsete.hpp` for those signatures.
 *
 * This header only declares the controller-specific tuning knobs. PP shares
 * the chassis geometry from `RamseteConfig` and the wheel feedforward from
 * `DriveFF` — both registered via `ramsete_configure()` — so this is the
 * only PP-specific call you need in `default_constants()`.
 */

namespace light {

/** Pure Pursuit controller config — only knob is the carrot lookahead. */
struct PurePursuitConfig {
  /// Carrot distance, inches. Larger = smoother but cuts corners; smaller =
  /// tighter tracking but oscillates near tight curvatures. Typical: 8–18 in.
  float lookaheadIn = 12.0f;
};

/**
 * Configure the Pure Pursuit follower. Call once in `initialize()` (after
 * `ramsete_configure()`, since PP reuses RamseteConfig + DriveFF).
 */
void pure_pursuit_configure(PurePursuitConfig cfg);

}  // namespace light
