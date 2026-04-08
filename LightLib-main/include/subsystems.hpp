    #pragma once

    #include "EZ-Template/api.hpp"
    #include "api.h"
    #include "pros/motors.hpp"

    extern Drive chassis;

    // Your motors, sensors, etc. should go here.  Below are examples

    // inline pros::Motor intake(1);
    // inline pros::adi::DigitalIn limit_switch('A');

    inline pros::Motor Top(7);
    inline pros::Motor Bottom(17);
    inline pros::MotorGroup Score({17, 7});


    inline ez::Piston Wings('A');
    inline ez::Piston IntakeLift('B');
    inline ez::Piston Loader('C');
    inline ez::Piston MidGoal('D');
    inline ez::Piston Hood('E');

    inline pros::Motor turret(0);
    inline pros::Optical optical(15); 

    inline pros::Distance left_front_sensor(11);  
    inline pros::Distance left_back_sensor(16);
    inline pros::Distance leftDist(16);
    inline pros::Distance frontDist(5);


    enum Colors { BLUE = 0, NEUTRAL = 1, RED = 2 };

    extern Colors allianceColor;
