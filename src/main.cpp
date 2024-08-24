#include "main.h"
#include "pros/misc.h"
#include "robot.hpp"
#include "controls.hpp"
#include "cata.hpp"
#include "autons/autons.hpp"
#include "keejLib/lib.hpp"

void (*auton)();

void initialize() {
	glb::imu.reset();
	robot::chassMtrs.setBrake(okapi::AbstractMotor::brakeMode::brake);
	robot::intake.setBrake(okapi::AbstractMotor::brakeMode::brake);
	robot::arm.setBrake(okapi::AbstractMotor::brakeMode::brake);
}

void autonomous() {
	auton();
}

void opcontrol() {
	robot::controller.setCurves(1, 1);
	while (true) {
		driver();
		robot::chassMtrs.spinDiffy(robot::controller.drive(1, lib::controller::arcade));
		pros::delay(20);
	}
}