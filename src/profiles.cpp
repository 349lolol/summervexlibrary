#include "main.h"

okapi::Controller controller(okapi::ControllerId::master);
//buttons on controller
okapi::ControllerAnalog leftY(okapi::ControllerAnalog::leftY);
okapi::ControllerAnalog rightX(okapi::ControllerAnalog::rightX);
okapi::ControllerAnalog rightY(okapi::ControllerAnalog::rightY);
okapi::ControllerButton l1(okapi::ControllerDigital::L1);
okapi::ControllerButton l2(okapi::ControllerDigital::L2);
okapi::ControllerButton r1(okapi::ControllerDigital::R1);
okapi::ControllerButton r2(okapi::ControllerDigital::R2);
okapi::ControllerButton rightButton(okapi::ControllerDigital::right);
okapi::ControllerButton downButton(okapi::ControllerDigital::down);
okapi::ControllerButton leftButton(okapi::ControllerDigital::left);
okapi::ControllerButton upButton(okapi::ControllerDigital::up); //no current bind atm
okapi::ControllerButton AButton(okapi::ControllerDigital::A);
okapi::ControllerButton BButton(okapi::ControllerDigital::B);
okapi::ControllerButton XButton(okapi::ControllerDigital::X);
okapi::ControllerButton YButton(okapi::ControllerDigital::Y);

//drive motors 
/*left front*/ okapi::Motor lf(10, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
/*left bottom*/ okapi::Motor lb(5, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
/* Left top */ okapi::Motor lt(9, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);

/*right front*/ okapi::Motor rf(2, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
/*right bottom*/ okapi::Motor rb(3, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
/* Right top */ okapi::Motor rt(4, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);

okapi::Motor intake(8, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);

okapi::Motor arm(6, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
/*drive motor groups*/ 
okapi::MotorGroup driveLeftGroup = {lf, lb, lt};
okapi::MotorGroup driveRightGroup = {rf, rb, rt};
okapi::MotorGroup driveGroup = {lf, lb, lt, rf, rb, rt};

//odom subsystem
pros::adi::Encoder verticalEncoder('C','D',false);
pros::adi::Encoder horizontalEncoder({19, 'A','B'},true);
pros::Task* odomTask;

//gyro
pros::Imu gyro(16);
