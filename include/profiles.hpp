#include "main.h"

// Master controller
extern okapi::Controller controller;

extern okapi::ControllerAnalog leftY;
extern okapi::ControllerAnalog rightX;
extern okapi::ControllerAnalog rightY;
extern okapi::ControllerButton l1;
extern okapi::ControllerButton l2;
extern okapi::ControllerButton r1;
extern okapi::ControllerButton r2;
extern okapi::ControllerButton leftButton;
extern okapi::ControllerButton downButton;
extern okapi::ControllerButton rightButton;
extern okapi::ControllerButton upButton;
extern okapi::ControllerButton YButton;
extern okapi::ControllerButton AButton;
extern okapi::ControllerButton BButton;
extern okapi::ControllerButton XButton;

// Drives
/*Left front*/ extern okapi::Motor lf;
/*Left middle*/  extern okapi::Motor lPTO;
/*Left back*/  extern okapi::Motor lb;
/*Right front*/extern okapi::Motor rf;
/*Right middle*/ extern okapi::Motor rPTO;
/*Right back*/ extern okapi::Motor rb;

extern okapi::Motor intake;
extern okapi::Motor arm;


//motor groups
extern okapi::MotorGroup driveLeftGroup;
extern okapi::MotorGroup driveRightGroup;
extern okapi::MotorGroup driveGroup;

extern pros::adi::Encoder verticalEncoder;
extern pros::adi::Encoder horizontalEncoder;
extern pros::Task* odomTask;

extern pros::IMU gyro;