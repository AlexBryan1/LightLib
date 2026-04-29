// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       DRIVETRAIN CONFIGURATION                          │
// │  Set your motor ports, wheel size, and sensor ports below.              │
// │  Use a negative port number to reverse that motor.                      │
// └─────────────────────────────────────────────────────────────────────────┘

#define LEFT_PORTS  {-1, -2, -3, -4, -5}   // left drive motors
#define RIGHT_PORTS { 6, 7, 8, 9, 10}   // right drive motors

#define IMU_PORT    0   // primary inertial sensor
#define IMU2_PORT    0   // second IMU — set to 0 if you only have one

#define WHEEL_DIAMETER  2.6   // inches  (4" wheels are actually ~4.125")
#define WHEEL_RPM       600    // motor cartridge RPM × (motor sprocket / wheel sprocket)
#define TRACK_HALF_W    8.0f   // half of robot track width in inches (used for odometry)

#define JOYSTICK_CURVE    0.2f   // expo curve strength (0 = linear, higher = more curve)
#define JOYSTICK_DEADZONE 10     // joystick values ±this are treated as 0 (0–127)

// ── Thermal buzz ──────────────────────────────────────────────────────────────
#define HEAT_BUZZ_ENABLED  true  // set to false to disable the temperature warning
#define HEAT_BUZZ_TEMP     55    // °C threshold that triggers the controller buzz

// ── Drive style ───────────────────────────────────────────────────────────────
//  1  Arcade        left stick = throttle,  right stick = turn
//  2  Tank          left stick = left side, right stick = right side
//  3  Single stick  left stick = throttle + turn (one hand)
//  4  X/Mecanum     left stick = move,      right stick = turn  (needs holoDrive in subsystems.hpp)
//  5  H-Drive       tank sides + strafe,    right stick = turn  (needs hDrive   in subsystems.hpp)
#define DRIVE_TYPE  1

// ── MCL distance sensors — one per side (set port to 0 if not installed) ─────
//  PORT:  VEX port number
//  ALONG: position along that face in robot frame, inches (0 = centered)
//  DEPTH: distance from robot center to sensor, inches

#define MCL_FRONT_PORT   0
#define MCL_FRONT_ALONG  0.0f
#define MCL_FRONT_DEPTH  6.0f

#define MCL_BACK_PORT    0
#define MCL_BACK_ALONG   0.0f
#define MCL_BACK_DEPTH   6.0f

#define MCL_LEFT_PORT    0
#define MCL_LEFT_ALONG   0.0f
#define MCL_LEFT_DEPTH   6.0f

#define MCL_RIGHT_PORT   0
#define MCL_RIGHT_ALONG  0.0f
#define MCL_RIGHT_DEPTH  6.0f

// ── MCL tuning ────────────────────────────────────────────────────────────────
#define MCL_PARTICLES    200     // particle count (reduce if CPU load is high)
#define MCL_SENSOR_SIGMA 2.5f   // distance sensor noise std dev (inches)
#define MCL_OUTLIER_GAP  6.0f   // readings shorter than expected by this → ignored
#define MCL_MAX_RANGE    144.0f // max ray-cast distance (12 ft for VRC)

#define EKF_Q_POS        0.02f    // position process noise  (in²/sec)
#define EKF_Q_THETA      0.0005f  // heading process noise   (rad²/sec)
#define EKF_Q_VEL        1.0f     // velocity process noise

#define MCL_SNAP_DIVERGE  9.0f   // EKF uncertainty (in²) before MCL can snap
#define MCL_SNAP_CONVERGE 3.0f   // MCL std dev (in) required before snapping

// ── Legacy subsystem distance sensors (for WallRide / collision detect) ───────
#define DIST_LEFT_FRONT_PORT  0
#define DIST_LEFT_BACK_PORT   0
#define DIST_FRONT_PORT       0


#include "LightLib/robot_impl.inl"

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       INITIALIZATION HOOK                               │
// │  Called at the end of initialize() — add any extra setup here.          │
// │  The chassis, IMU, and auton selector are already ready by this point.  │
// └─────────────────────────────────────────────────────────────────────────┘
void user_initialize() {
    // Stiff hold for the snapping lift, and define boot pose as 0°.
    LiftMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    LiftRot.reset_position();
}

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       AUTONOMOUS HOOK                                   │
// │  Called at the start of autonomous(), before the selected routine runs. │
// │  Use this to set allianceColor or do any pre-auton setup.               │
// └─────────────────────────────────────────────────────────────────────────┘
void user_autonomous() {

}

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       OPERATOR CONTROL                                  │
// │  Map your buttons and subsystems here.                                  │
// │  Motors and pistons are declared in subsystems.hpp.                     │
// └─────────────────────────────────────────────────────────────────────────┘
void opcontrol() {
    chassis.drive_brake_set(MOTOR_BRAKE_COAST);

    while (true) {
        auton_toggle();

        if (!auton_running) {
            _run_drive();   // style set by DRIVE_TYPE at the top of this file

            // ── Held buttons ──────────────────────────────────────────────
            if (master.get_digital(DIGITAL_R2))
                MidGoal.set(false), Score.move(-127), Hood.set(false);
            else if (master.get_digital(DIGITAL_R1))
                Score.move(127);
            else if (master.get_digital(DIGITAL_L2))
                MidGoal.set(true), Score.move(-127);
            else if (master.get_digital(DIGITAL_L1))
                Score.move(-127), Hood.set(true), MidGoal.set(false);
            else
                Score.move(0);

            // ── Lift (snaps to nearest preset on release) ──────────────────
            // Joystick provides analog control; UP/DOWN buttons override it.
            int liftInput = master.get_analog(ANALOG_RIGHT_Y);
            if      (master.get_digital(DIGITAL_UP))   liftInput =  100;
            else if (master.get_digital(DIGITAL_DOWN)) liftInput = -100;
            Lift.update(liftInput);
        }

        // ── Toggle buttons ────────────────────────────────────────────────
        Wings.button_toggle     (master.get_digital(DIGITAL_B));
        Loader.button_toggle    (master.get_digital(DIGITAL_Y));

        pros::delay(10);
    }
}
