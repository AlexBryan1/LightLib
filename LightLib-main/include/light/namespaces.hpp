#pragma once

#ifdef __cplusplus

#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/link.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "pros/vision.hpp"

#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QTime.hpp"

namespace light {

using ::pros::Motor;
using ::pros::MotorGroup;
using ::pros::AbstractMotor;
using ::pros::Controller;
using ::pros::Imu;
using ::pros::Rotation;
using ::pros::Distance;
using ::pros::Optical;
using ::pros::Gps;
using ::pros::Vision;
using ::pros::Mutex;
using ::pros::Task;

using ::pros::motor_brake_mode_e_t;
using ::pros::controller_digital_e_t;
using ::pros::controller_analog_e_t;
using ::pros::controller_id_e_t;

using ::pros::E_CONTROLLER_MASTER;
using ::pros::E_CONTROLLER_PARTNER;
using ::pros::E_CONTROLLER_ANALOG_LEFT_X;
using ::pros::E_CONTROLLER_ANALOG_LEFT_Y;
using ::pros::E_CONTROLLER_ANALOG_RIGHT_X;
using ::pros::E_CONTROLLER_ANALOG_RIGHT_Y;
using ::pros::E_CONTROLLER_DIGITAL_L1;
using ::pros::E_CONTROLLER_DIGITAL_L2;
using ::pros::E_CONTROLLER_DIGITAL_R1;
using ::pros::E_CONTROLLER_DIGITAL_R2;
using ::pros::E_CONTROLLER_DIGITAL_UP;
using ::pros::E_CONTROLLER_DIGITAL_DOWN;
using ::pros::E_CONTROLLER_DIGITAL_LEFT;
using ::pros::E_CONTROLLER_DIGITAL_RIGHT;
using ::pros::E_CONTROLLER_DIGITAL_X;
using ::pros::E_CONTROLLER_DIGITAL_B;
using ::pros::E_CONTROLLER_DIGITAL_Y;
using ::pros::E_CONTROLLER_DIGITAL_A;
using ::pros::E_MOTOR_BRAKE_COAST;
using ::pros::E_MOTOR_BRAKE_BRAKE;
using ::pros::E_MOTOR_BRAKE_HOLD;

using ::pros::delay;
using ::pros::millis;
using ::pros::micros;

namespace adi         { using namespace ::pros::adi; }
namespace lcd         { using namespace ::pros::lcd; }
namespace c           { using namespace ::pros::c; }
namespace usd         { using namespace ::pros::usd; }
namespace battery     { using namespace ::pros::battery; }
namespace competition { using namespace ::pros::competition; }

using ::okapi::QLength;
using ::okapi::QAngle;
using ::okapi::QTime;

using ::okapi::inch;
using ::okapi::degree;
using ::okapi::millisecond;

inline namespace literals {
  using namespace ::okapi::literals;
}

}  // namespace light

#endif  // __cplusplus
