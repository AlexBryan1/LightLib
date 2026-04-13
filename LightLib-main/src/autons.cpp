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
  MidGoal.set(false);
  Hood.set(true);
  IntakeLift.set(true);
}
///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(10.0, 0.1, 30.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(17.0, 0.0, 50.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(5.0, 0.0, 35.0 );     // Turn in place constants
  chassis.pid_swing_constants_set(4.0, 0.0, 45.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(1.0, 0.0, 10.0);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(100_ms, 3_deg, 300_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);


  chassis.pid_turn_chain_constant_set(10_deg);
  chassis.pid_swing_chain_constant_set(15_deg);
  chassis.pid_drive_chain_constant_set(5_in);

  // Slew constants
  chassis.slew_turn_constants_set(10_deg, 55);
  chassis.slew_drive_constants_set(3_in, 30);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(1.0);

  chassis.odom_look_ahead_set(15_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
  
}

void skills(){
  
}
void rush_mid_left(){

}
void rush_mid_right(){
  Score.move(-127);
  chassis.pid_swing_set(ez::RIGHT_SWING, 285_deg, 127, 50, ez::ccw);
  chassis.pid_wait_quick_chain();
  Loader.set(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 300_deg, 127, 0 , ez::cw);
  chassis.pid_wait_quick_chain();
  Loader.set(true);
  chassis.pid_drive_set(20_in, 127);
  chassis.pid_wait_quick_chain();
  Loader.set(false);
  pros::delay(200);
}
void rush_right(){
  
}
void rush_left(){
  Score.move(-127);
  Wings.set(false);
  chassis.pid_swing_set(ez::RIGHT_SWING, 210_deg, 127, 50, ez::ccw);
  chassis.pid_wait_until(220);
  Loader.set(false);
  chassis.pid_drive_set(35_in, 127);
  chassis.pid_wait_until(25_in);
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 40 , ez::ccw);
  pros::delay(300);
  chassis.pid_drive_set(50_in, 80);
  pros::delay(1000);
  chassis.pid_drive_set(-30_in, 127);
  chassis.pid_wait_until(-15_in);
  Hood.set(false);
  pros::delay(1650);
  chassis.pid_swing_set(ez::RIGHT_SWING, 100_deg, 127, 5 , ez::ccw);
  chassis.pid_wait_until(110_deg);
  Hood.set(true);
  chassis.pid_swing_set(ez::LEFT_SWING, 185_deg, 127, 4 , ez::cw);
  chassis.pid_wait_until(177_deg);
  Wings.set(true);
  // chassis.pid_drive_exit_condition_set(50000_ms, 1_in, 25000_ms, 3_in, 50000_ms, 50000_ms);
  chassis.pid_drive_set(-25_in, 127);
  chassis.pid_wait();
}
void split_right(){
  Score.move(-127);
  chassis.pid_swing_set(ez::RIGHT_SWING, 285_deg, 127, 50, ez::ccw);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 340_deg, 90, 40, ez::cw);
  chassis.pid_wait_quick_chain();
  Loader.set(false);
  pros::delay(200);
  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, 90, 60, ez::cw);
  chassis.pid_wait_quick_chain();
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 127, 10, ez::cw);
  chassis.pid_wait_quick_chain();
}


void split_left(){
  Score.move(-127);
  chassis.pid_swing_set(ez::RIGHT_SWING, 285_deg, 127, 45, ez::ccw);
  chassis.pid_wait_quick_chain();
  Loader.set(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 300_deg, 127, 0 , ez::cw);
  chassis.pid_wait_quick_chain();
  Loader.set(true);
  chassis.pid_drive_set(20_in, 127);
  chassis.pid_wait_quick_chain();
  Loader.set(false);
  pros::delay(200);
  chassis.pid_swing_set(ez::RIGHT_SWING, 80_deg, 127, 30, ez::cw);
  chassis.pid_wait_until(45_deg);
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 0, ez::cw);
  chassis.pid_wait_until(160_deg);
  chassis.pid_drive_set(-30_in, 127);
  Hood.set(false);
  pros::delay(700);
  chassis.pid_turn_set(175_deg, 127);
  pros::delay(800);
  Hood.set(true);
  chassis.pid_drive_set(30_in, 127);
  chassis.pid_wait_until(15_in);
  chassis.pid_speed_max_set(40);
  pros::delay(800);
  chassis.pid_speed_max_set(127);
  chassis.pid_swing_set(ez::RIGHT_SWING, 217_deg, 127, 20, ez::cw);
  chassis.pid_wait_until(212_deg);
  chassis.pid_drive_set(-45_in, 127);
  Score.move(127);
  pros::delay(100);
  Score.move(0);
  chassis.pid_wait_until(-25_in);
  chassis.pid_speed_max_set(80);
  MidGoal.set(true);
  Score.move(-127);
  pros::delay(1000);
  chassis.pid_speed_max_set(127);
  Score.move(0);
  chassis.pid_drive_set(20_in, 127);
  chassis.pid_wait_until(15_in);
  chassis.pid_swing_set(ez::RIGHT_SWING, 150_deg, 127, 5, ez::ccw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-25_in, 127);
  chassis.pid_wait_until(-10_in);
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 0, ez::cw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_exit_condition_set(5000_ms, 1_in, 25000_ms, 3_in, 50000_ms, 50000_ms);
  MidGoal.set(false);
  Loader.set(true);
  chassis.pid_drive_set(-3_in, 127);
  chassis.pid_wait();
}

void sevenball_right(){

}

void sevenball_left(){

}
void sixball_right(){

}
void sixball_left(){
  Score.move(-127);
  chassis.pid_swing_set(ez::RIGHT_SWING, 285_deg, 127, 45, ez::ccw);
  chassis.pid_wait_quick_chain();
  Loader.set(false);
  chassis.pid_swing_set(ez::LEFT_SWING, 300_deg, 127, 0 , ez::cw);
  chassis.pid_wait_quick_chain();
  Loader.set(true);
  chassis.pid_drive_set(20_in, 127);
  chassis.pid_wait_quick_chain();
  Loader.set(false);
  pros::delay(200);
  chassis.pid_swing_set(ez::RIGHT_SWING, 80_deg, 127, 30, ez::cw);
  chassis.pid_wait_until(45_deg);
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 0, ez::cw);
  chassis.pid_wait_until(160_deg);
  chassis.pid_drive_set(-30_in, 127);
  Hood.set(false);
  pros::delay(1200);
  chassis.pid_swing_set(ez::RIGHT_SWING, 100_deg, 127, 5 , ez::ccw);
  chassis.pid_wait_until(110_deg);
  Hood.set(true);
  chassis.pid_swing_set(ez::LEFT_SWING, 180_deg, 127, 4 , ez::cw);
  chassis.pid_wait_until(170_deg);
  Wings.set(true);
  chassis.pid_drive_exit_condition_set(50000_ms, 1_in, 25000_ms, 3_in, 50000_ms, 50000_ms);
  chassis.pid_drive_set(-25_in, 127);
  chassis.pid_wait();
}
void con_test() {
  Hood.set(false);
  MidGoal.set(true);
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
