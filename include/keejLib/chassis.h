#pragma once
#include "util.h"
#include "okapi/api.hpp"

namespace lib
{
    class chassis
    {
        private:
            lib::diffy* chass;
            pros::Imu* imu;
            pros::ADIEncoder* horizTracker = nullptr;
            pros::ADIEncoder* vertTracker = nullptr;
            point pos = {0,0};
            lib::robotConstants constants;
            lib::accelConstants linear;
            lib::accelConstants angular;
            pros::Task* odomTask = nullptr;
            double prevRotation = 0;

        public:
            lib::odomType odom;

            okapi::Motor lf{2}; // Replace '2' with the appropriate port number
            okapi::Motor lb{3}; // Replace '3' with the appropriate port number
            okapi::Motor lt{4}; // Replace '4' with the appropriate port number
            okapi::Motor rf{5}; // Replace '5' with the appropriate port number
            okapi::Motor rb{6}; // Replace '6' with the appropriate port number
            okapi::Motor rt{7}; // Replace '7' with the appropriate port number
            okapi::MotorGroup driveLeftGroup{lf, lb, lt};
            okapi::MotorGroup driveRightGroup{rf, rb, rt};
            okapi::MotorGroup driveGroup{rf, rb, rt, lf, lb, lt};

            chassis(lib::diffy& mtrs, pros::Imu& imu) 
                : chass(&mtrs), imu(&imu), 
                  driveLeftGroup{lf, lb, lt}, 
                  driveRightGroup{rf, rb, rt}, 
                  driveGroup{rf, rb, rt, lf, lb, lt} {}

            chassis(lib::diffy& mtrs, pros::Imu& imu, lib::robotConstants constants, lib::accelConstants linear, lib::accelConstants angular) 
                : chass(&mtrs), imu(&imu), constants(constants), linear(linear), angular(angular),
                  driveLeftGroup{lf, lb, lt}, 
                  driveRightGroup{rf, rb, rt}, 
                  driveGroup{rf, rb, rt, lf, lb, lt} {}

            chassis(lib::diffy& mtrs, pros::Imu& imu, std::vector<int> encoderPorts, lib::robotConstants constants, lib::accelConstants linear, lib::accelConstants angular)
                : chass(&mtrs), imu(&imu), constants(constants), linear(linear), angular(angular),
                  driveLeftGroup{lf, lb, lt}, 
                  driveRightGroup{rf, rb, rt}, 
                  driveGroup{rf, rb, rt, lf, lb, lt} {}
        
            void updatePos();
            void initTracking();

            //1dpid
            lib::pid pidDrive(double target, double timeout, lib::pidConstants constants, char brake);
            lib::pid pidTurn(double target, double timeout, lib::pidConstants constants, double slew, char brake);
            lib::pid pidDrive(double target, double timeout, char brake, lib::pid cont);
            lib::pid pidTurn(double target, double timeout, char brake, lib::pid cont);

            void arcTurn(double target, double radius, double timeout, int dir, lib::pidConstants constants, double min, int endTime, char brake); 
            void eulerTurn(double theta, double rate, double timeout, int dir, lib::pidConstants constants);

            //1dmp
            std::vector<double> asymTrapezoidalProfile(double dist, double maxSpeed, double accel, double decel, double start, double end);
            void profiledDrive(double target, int endDelay, double start, double end);
            void profiledTurn(double target, int dir, int endDelay);
            void timedDrive(int time, int speed);

            //2dpid
            void driveAngle(double target, double heading, double timeout, lib::pidConstants lCons, lib::pidConstants acons, bool reset, double rushError, double slew);
            std::vector<double> pidMTPVel(const point& target, double rotationBias, lib::pid* lCont, lib::pid* rCont);
            void pidMoveTo(const point& target, double timeout, lib::pidConstants lConstants, lib::pidConstants rConstants, double rotationBias);
            void boomerang(const point& target, double timeout, double dLead, double thetaEnd, double rotationBias, lib::pidConstants lConstants, lib::pidConstants rConstants);

            //pp
            lib::point targetPoint(lib::linearPath path, int lookAhead, int lineLookAhead, int lineIndex);
            void moveToPurePursuit(lib::linearPath path, double lookAhead, int lineLookAhead, int finalTimeout, lib::pidConstants lConstants, lib::pidConstants rConstants, double rotationBias);
    };
}