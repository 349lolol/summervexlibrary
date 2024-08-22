#include "main.h"


void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}


void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	driveLeftGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	driveRightGroup.setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
}


void disabled() {

}


void competition_initialize() {

}

void autonomous() {
	/**
	 * 
	 * 
	 * -------------------->|
	 *  ^                   |
	 *  |                   |
	 *  |                   |
	 *  |                   |
	 * 	|                   |
	 *  |                   |
	 *  |                   |
	 *  <--------------------
	 * 
	 * travels in a box lmao
	 * start robot in this orientation from your position <-
	 */
	// moveDistance(24, 0, 12000, {0.1, 0.0, 0.1});
	// rotationTurn(90, 4000, 1, 1000, {0.1, 0.0, 0.1});
	// moveDistance(24, 90, 12000, {0.1, 0.0, 0.1});
	// rotationTurn(180, 4000, 1, 1000, {0.1, 0.0, 0.1});
	// moveDistance(24, 180, 12000, {0.1, 0.0, 0.1});
	// rotationTurn(270, 4000, 1, 1000, {0.1, 0.0, 0.1});
	// moveDistance(24, 270, 12000, {0.1, 0.0, 0.1});
	// rotationTurn(360, 4000, 1, 1000, {0.1, 0.0, 0.1});
}

void opcontrol() {
	while (true) {
	{
		double fwd = controller.getAnalog(leftY);
		double turn = controller.getAnalog(rightX); 
		if(l1.isPressed()) {
			intake.moveVoltage(12000);
		}
		else if(l2.isPressed()) {
			intake.moveVoltage(-12000);
		}
		else {
			intake.moveVoltage(0);
		}

		if(r1.isPressed()) {
			arm.moveVoltage(12000);
		}
		else if(r2.isPressed()) {
			arm.moveVoltage(-12000);
		}
		else {
			arm.moveVoltage(0);
		}
		
		driveLeftGroup.moveVoltage(std::clamp(fwd + turn, -1.0, 1.0) * 12000);
		driveRightGroup.moveVoltage(std::clamp(fwd - turn, -1.0, 1.0) * 12000);
	} //drive train code
	pros::delay(10);
	}
}