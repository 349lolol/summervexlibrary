#include "main.h"

#ifndef ODOM_HPP
#define ODOM_HPP
struct {
    double x;
    double y;
    double theta;
    double lastVerticalEncoderReading;
    double lastHorizontalEncoderReading;
} InternalRobotPos;

struct point{
    double x;
    double y;
    double theta;
};

point pos;

extern double toRadian(double angle);
extern void initializePosition();
extern void resetPosition();
extern void updatePosition();
extern void initTracking();

#endif