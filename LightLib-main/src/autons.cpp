#include "autons.hpp"
#include <cmath>
#include "EZ-Template/util.hpp"
#include "cstm_move.hpp"
#include "main.h"
#include "pros/motors.h"
#include "subsystems.hpp"
#include "drive_utils.hpp"
#include "odom.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 127;
const int SWING_SPEED = 127;
///
// This is for turret function, ignore it if you aren't using it
///
void track_basket(){
    ///
    // Ignore these 2 I am just declaring variables
    ///
    double err_basket = 0.0;
    double p_turret = 0.0;
    ///
    // Set x and y to the position of the "basket" or where the goal you score in is,
    // Tune turret_p as a p controller of how fast/accurately it snaps to the position
    ///
    double basket_x = 0.0;
    double basket_y = 0.0;

    double turret_p = 0.0;

    while(true){
    double current_x = chassis.odom_x_get();
    double current_y = chassis.odom_y_get();
    double current_angle = chassis.odom_theta_get();

    double x_pos = basket_x - current_x;
    double y_pos = basket_y - current_y;

    double rads = std::atan2(y_pos, x_pos);
    double basket_pos = rads * (180.0 / M_PI);
    double err_basket = basket_pos - current_angle;
    double p_turret = err_basket * turret_p;
    pros::delay(10);
    }
}
///
// Set the position you want your pistons to start in
///
void default_positions(){
  Wings.set(true);
  Loader.set(true);
  MidGoal.set(true);
  Hood.set(true);
  IntakeLift.set(true);
}
///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(7.0, 0.10, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(5.0, 0.0, 120.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(2.2, 0.02, 15.0, 5.0 );     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 50.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(1.0, 0.0, 10.0);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(100_ms, 3_deg, 300_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);


  chassis.pid_turn_chain_constant_set(5_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(2.5_in);

  // Slew constants
  chassis.slew_turn_constants_set(10_deg, 55);
  chassis.slew_drive_constants_set(3_in, 30);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(1.0);

  chassis.odom_look_ahead_set(10_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
  
}

void skills(){
  
}

void rush_right(){

}

void rush_left(){

}
void split_right(){

}

void split_left(){

}

void sevenball_right(){

}

void sevenball_left(){

}

void con_test() {
  Hood.set(false);
  MidGoal.set(false);
  Score.move(-127);
  chassis.pid_drive_set(-10, 127);
  pros::delay(4000);
  light::moveToPoint(15.0,10.0,5000,127,0);
  chassis.pid_turn_set(0, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-20, 127);
  chassis.pid_wait();
}




void drive_test(int inches) {
    chassis.pid_drive_set(inches, DRIVE_SPEED);
    chassis.pid_wait();
}

void turn_test(int degrees) {
    chassis.pid_turn_set(degrees, TURN_SPEED, raw);
    chassis.pid_wait();
}

void swing_test(int degrees) {
    chassis.pid_swing_set(LEFT_SWING, degrees, SWING_SPEED, raw);
    chassis.pid_wait();
}

void heading_test(int degrees) {
    chassis.pid_drive_set(12, DRIVE_SPEED);
    chassis.headingPID.target_set(chassis.drive_imu_get() + degrees);
}

void odom_test(int degrees) {
    chassis.odom_xyt_set(0, 0, degrees);
    chassis.pid_odom_set({{0_in, 24_in}, fwd, DRIVE_SPEED});
}

void antijam(){
  Score.move(127);
  pros::delay(150);
  Score.move(-127);

}
