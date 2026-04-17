// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       DRIVETRAIN CONFIGURATION                           │
// │  Set your motor ports, wheel size, and sensor ports below.              │
// │  Use a negative port number to reverse that motor.                      │
// └─────────────────────────────────────────────────────────────────────────┘

#define LEFT_PORTS  {-10, -9, -8}   // left drive motors
#define RIGHT_PORTS { 20, 19, 18}   // right drive motors

#define IMU_PORT    21   // primary inertial sensor
#define IMU2_PORT    0   // second IMU — set to 0 if you only have one

#define WHEEL_DIAMETER  3.25   // inches  (4" screw-hole wheels are actually ~4.125)
#define WHEEL_RPM       450    // motor cartridge RPM × (motor sprocket / wheel sprocket)
#define TRACK_HALF_W    3.0f   // half of robot track width in inches (used for odometry)

#define JOYSTICK_CURVE    0.2f   // expo curve strength (0 = linear, higher = more curve)
#define JOYSTICK_DEADZONE 10     // joystick values ±this are treated as 0 (0–127)

// ── Thermal buzz ──────────────────────────────────────────────────────────────
#define HEAT_BUZZ_ENABLED  true  // set to false to disable the temperature warning
#define HEAT_BUZZ_TEMP     55    // °C threshold that triggers the controller buzz

// ── Drive style ───────────────────────────────────────────────────────────────
//  1  Arcade        left stick = throttle,  right stick = turn
//  2  Tank          left stick = left side, right stick = right side
//  3  Single stick  left stick = throttle + turn (one hand)
//  4  X/Mecanum  left stick = move,      right stick = turn  (needs holoDrive in subsystems.hpp)
//  5  H-Drive       tank sides + strafe,    right stick = turn  (needs hDrive   in subsystems.hpp)
#define DRIVE_TYPE  1

// ── Distance sensor ports (set to 0 if not installed) ────────────────────────
#define DIST_LEFT_FRONT_PORT  0
#define DIST_LEFT_BACK_PORT   0
#define DIST_FRONT_PORT       15


#include "LightLib/robot_impl.inl"

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       INITIALIZATION HOOK                                │
// │  Called at the end of initialize() — add any extra setup here.          │
// │  The chassis, IMU, and auton selector are already ready by this point.  │
// └─────────────────────────────────────────────────────────────────────────┘
void user_initialize() {

}

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       AUTONOMOUS HOOK                                    │
// │  Called at the start of autonomous(), before the selected routine runs. │
// │  Use this to set allianceColor or do any pre-auton setup.               │
// └─────────────────────────────────────────────────────────────────────────┘
void user_autonomous() {

}

// ┌─────────────────────────────────────────────────────────────────────────┐
// │                       OPERATOR CONTROL                                   │
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
        }

        // ── Toggle buttons ────────────────────────────────────────────────
        Wings.button_toggle     (master.get_digital(DIGITAL_B));
        Loader.button_toggle    (master.get_digital(DIGITAL_Y));

        pros::delay(10);
    }
}
