#pragma once

#include "keejLib/lib.hpp"
#include "main.h"

#define MAX_LIN 0.647953484803
#define MAX_ANG 0.647953484803

namespace glb {
// pros
pros::Imu imu(6);
pros::ADIDigitalOut clamp('D');
pros::ADIDigitalOut tilter('C');
pros::ADIDigitalOut arm('E');

pros::ADIButton limit(-1);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Rotation rot(3);
lib::scheduler async;
} // namespace glb

// https://www.desmos.com/calculator/zwl4noapxl
// math for robot constants
namespace robot {
    lib::diffy chassMtrs({-5, 9, -10, 2, 3, -4});
    lib::mtrs intake({8});
    lib::mtrs arm({6});
    lib::mtrs tsukasa({-1});


/*left front*/  okapi::Motor lf(10, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
/*left bottom*/ okapi::Motor lb(5, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
/* Left top */  okapi::Motor lt(9, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);

/*right front*/  okapi::Motor rf(2, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
/*right bottom*/ okapi::Motor rb(3, false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
/* Right top */  okapi::Motor rt(4, true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);

/* left side*/   okapi::MotorGroup driveLeftGroup = {lf, lb, lt};
/* right side*/  okapi::MotorGroup driveRightGroup = {rf, rb, rt};
/*combined mtrs*/okapi::MotorGroup driveGroup = {lf, lb, lt, rf, rb, rt};


    lib::pis clamp({glb::clamp}, true);
    lib::pis tilter({glb::tilter}, true);
    lib::pis rightArm({glb::arm}, false);
    
    lib::controller controller(glb::controller);
    lib::chassis chass(chassMtrs, glb::imu, {1, 1},
        {
            .horizTrack = 5.53532 + 5.25712,
            .vertTrack = (5.53532 + 5.25712) / 2,
            .trackDia = 0,
        },
        {
            .maxSpeed = MAX_LIN,
            .fwdAccel = MAX_LIN / 26,
            .fwdDecel = MAX_LIN / 18,
            .revAccel = MAX_LIN / 18,
            .revDecel = MAX_LIN / 32,
            .velToVolt = 196.001723856},
        {.maxSpeed = MAX_ANG,
            .fwdAccel = MAX_ANG / 40,
            .fwdDecel = MAX_ANG / 40,
            .revAccel = MAX_ANG / 20,
            .revDecel = MAX_ANG / 20,
            .velToVolt = 127 / MAX_ANG});
} // namespace robot
