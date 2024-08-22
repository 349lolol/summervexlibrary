#include "main.h"
#include "PID_Controller.hpp"

#define DRIVE_CIRCUMFERENCE 3.25 * 3.14159
#define GEAR_RATIO 1/1
/**
 * @brief rotationTurn
 * turns on the spot using both wheels
 * 
 * @param angle angle to be turned to
 * @param turnVoltage how fast the motors should spin, in millivolts
 * @param momentum tolerance (in degrees) for the robot to stop turning
 * @param timeout longest time the function can run for, in milliseconds
 * @param values pid values
 */
void rotationTurn(double angle, double turnVoltage, double momentum, uint32_t timeout, PIDvalues values){
    uint32_t startTime = pros::millis();
    PID_Controller turnController(values);
    turnController.SetTarget(angle);
    double startingAngle = gyro.get_rotation();

    while(abs(angle - gyro.get_rotation()) > momentum && timeout > pros::millis() - startTime){
        double outputValue = turnController.Calculate(gyro.get_rotation());
        driveLeftGroup.moveVoltage(turnVoltage * outputValue);
        driveRightGroup.moveVoltage(turnVoltage * outputValue * (-1));

        pros::lcd::print(1, "Current angle: %f", gyro.get_rotation());
        pros::lcd::print(2, "target angle: %f", angle);
        pros::delay(10);
    }
    driveLeftGroup.moveVoltage(0);
    driveRightGroup.moveVoltage(0);
}

/**
 * @brief moveDistance
 * moves the robot a certain distance
 * 
 * @param distance distance to move, in inches
 * @param heading angle to move at, in degrees
 * @param voltage voltage to move at, in millivolts
 * @param values pid values
 */
void moveDistance(double distance, double heading, int voltage, PIDvalues values) {
    PID_Controller angleController(values);
    angleController.SetTarget(heading);

    double leftTarget = driveLeftGroup.getPosition() / 360 * DRIVE_CIRCUMFERENCE * GEAR_RATIO + distance;
    double rightTarget = driveRightGroup.getPosition() / 360 * DRIVE_CIRCUMFERENCE * GEAR_RATIO + distance;

    while((driveLeftGroup.getPosition() / 360 * DRIVE_CIRCUMFERENCE * GEAR_RATIO < leftTarget) && (driveRightGroup.getPosition() / 360 * DRIVE_CIRCUMFERENCE * GEAR_RATIO < rightTarget)) {
        double angleOutput = angleController.Calculate(gyro.get_rotation()) / 2; //reduce correctional scale to -0.5 to 0.5
        driveLeftGroup.moveVoltage((squiggles::sgn(distance) * 1 + angleOutput) * voltage);
        driveRightGroup.moveVoltage((squiggles::sgn(distance) * 1 - angleOutput) * voltage);
        pros::delay(10);

        pros::lcd::print(1, "Current angle: %f", gyro.get_rotation());
        pros::lcd::print(2, "Distance remaining: %f", leftTarget + rightTarget - driveLeftGroup.getPosition() / 360 * DRIVE_CIRCUMFERENCE * GEAR_RATIO - driveRightGroup.getPosition() / 360 * DRIVE_CIRCUMFERENCE * GEAR_RATIO);
    }
    driveLeftGroup.moveVoltage(0);
    driveRightGroup.moveVoltage(0);
}