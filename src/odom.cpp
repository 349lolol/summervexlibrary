#include "main.h"
#include "odom.hpp"

#define TRACKER_CIRCUMFERENCE (2.75 * 3.14159)
#define VERTICAL_X_OFFSET 0
#define VERTICAL_Y_OFFSET 0
#define HORIZONTAL_X_OFFSET 0
#define HORIZONTAL_Y_OFFSET 0
#define ENCODER_TICKS 360.0



double toRadian(double angle) {
    return angle / 180.0 * 3.14159;
}



void initializePosition() {
    InternalRobotPos.x = 0;
    InternalRobotPos.y = 0;
    InternalRobotPos.theta = 0;
    InternalRobotPos.lastVerticalEncoderReading = verticalEncoder.get_value();
    InternalRobotPos.lastHorizontalEncoderReading = horizontalEncoder.get_value();

    pos.x = InternalRobotPos.x;
    pos.y = InternalRobotPos.y;
}

void resetPosition() {
    verticalEncoder.reset();
    horizontalEncoder.reset();
    pros::delay(20);
    InternalRobotPos.lastVerticalEncoderReading = verticalEncoder.get_value();
    InternalRobotPos.lastHorizontalEncoderReading = horizontalEncoder.get_value();
    InternalRobotPos.x = 0;
    InternalRobotPos.y = 0;
    InternalRobotPos.theta = toRadian(gyro.get_rotation());

    pos.x = InternalRobotPos.x;
    pos.y = InternalRobotPos.y;
}

void updatePosition() {
    double currentVerticalEncoderReading = verticalEncoder.get_value();
    double currentHorizontalEncoderReading = horizontalEncoder.get_value();
    double deltaVerticalEncoderReading = currentVerticalEncoderReading - InternalRobotPos.lastVerticalEncoderReading;
    double deltaHorizontalEncoderReading = currentHorizontalEncoderReading - InternalRobotPos.lastHorizontalEncoderReading;

    // Convert encoder readings to distance
    double deltaVerticalDistance = (deltaVerticalEncoderReading / ENCODER_TICKS) * TRACKER_CIRCUMFERENCE;
    double deltaHorizontalDistance = (deltaHorizontalEncoderReading / ENCODER_TICKS) * TRACKER_CIRCUMFERENCE;

    double deltaTheta = (toRadian(gyro.get_rotation()) - InternalRobotPos.theta);

    // Rotational displacement
    double deltaHorizontalRotation = -HORIZONTAL_X_OFFSET * deltaTheta;
    double deltaVerticalRotation = VERTICAL_Y_OFFSET * deltaTheta;

    // Translational displacement
    double deltaHorizontalTranslation = deltaHorizontalDistance + deltaHorizontalRotation;
    double deltaVerticalTranslation = deltaVerticalDistance + deltaVerticalRotation;

    // Local movement
    double deltaXlocal = deltaVerticalTranslation;
    double deltaYlocal = deltaHorizontalTranslation;

    // Rotate local movement to global frame
    double deltaX = deltaXlocal * cos(InternalRobotPos.theta) - deltaYlocal * sin(InternalRobotPos.theta);
    double deltaY = deltaXlocal * sin(InternalRobotPos.theta) + deltaYlocal * cos(InternalRobotPos.theta);

    // Update global position
    InternalRobotPos.x += deltaX;
    InternalRobotPos.y += deltaY;
    InternalRobotPos.theta += deltaTheta;

    pos.x = InternalRobotPos.x;
    pos.y = InternalRobotPos.y;

    // Update last encoder readings
    InternalRobotPos.lastVerticalEncoderReading = currentVerticalEncoderReading;
    InternalRobotPos.lastHorizontalEncoderReading = currentHorizontalEncoderReading;
}

void initTracking() {
    initializePosition();
    odomTask = new pros::Task{[=] 
    {
        while (true) 
        {
        updatePosition();
        pros::delay(10);
        }
    }};
}