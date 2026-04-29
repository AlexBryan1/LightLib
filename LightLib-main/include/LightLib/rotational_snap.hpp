#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

#include <vector>

namespace light {

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                        RotationalSnap                                   │
// │                                                                         │
// │  Drives a motor manually while the operator is pressing a button or     │
// │  deflecting a joystick, and automatically snaps to the nearest preset   │
// │  angle the instant the operator stops.                                  │
// │                                                                         │
// │  Pair any pros::Motor with a pros::Rotation sensor on the same axle.    │
// │  Call update(input) once per opcontrol tick where `input` is your       │
// │  blended button + joystick value in the range -127..127.                │
// │                                                                         │
// │  NOTE: nearest-snap is computed in linear angle distance — fine for a   │
// │  lift or arm that doesn't wrap. For a turret, add shortest-arc math.    │
// └─────────────────────────────────────────────────────────────────────────┘
class RotationalSnap {
public:
    RotationalSnap(pros::AbstractMotor& motor,
                   pros::Rotation& sensor,
                   std::vector<double> snap_angles_deg,
                   double kP             = 1.5,
                   double tolerance_deg  = 1.0,
                   int    max_snap_speed = 80,
                   int    input_deadband = 5);

    // Call once per opcontrol loop. `manual_input` is -127..127.
    // |input| > deadband → manual control. Going from non-zero to zero
    // latches the nearest snap angle and runs a P-controller to it.
    void update(int manual_input);

    void   set_snap_angles(std::vector<double> angles_deg);
    void   set_enabled(bool on);          // false → pure passthrough
    double get_position_deg() const;      // sensor reading in degrees
    double get_target_deg()   const;      // current snap target, NaN if none
    bool   is_snapping()      const { return has_target_; }

private:
    double nearest_snap_(double pos_deg) const;

    pros::AbstractMotor& motor_;
    pros::Rotation& sensor_;
    std::vector<double> snaps_;

    double kP_;
    double tol_;
    double target_     = 0.0;
    int    max_speed_;
    int    deadband_;
    bool   enabled_    = true;
    bool   was_active_ = false;
    bool   has_target_ = false;
};

} // namespace light
