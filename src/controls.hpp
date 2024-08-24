#include "main.h"
#include "robot.hpp"
#include "autons/constants.hpp"

using namespace lib;

bool isClamped = false;
bool aLastState = false;
lib::timer globalTimer;
int lastActuated = -1;
void driver() {
    std::vector<bool> state = robot::controller.getAll(ALLBUTTONS);

    //tilter control
    {
        if(state[A] != aLastState && state[A] == true) {
            aLastState = true;
            globalTimer.reset();
            if(isClamped) {
                robot::tilter.setState(true);
                isClamped = false;
            }
            else {
                robot::clamp.setState(false);
                isClamped = true;
            }
        }
        else {
            aLastState = false;
        }

        if(globalTimer.time() > 200) {
            if(isClamped) {
                robot::tilter.setState(false);
            }
            else {
                robot::clamp.setState(true);
            }
        }
    }

    //extend arm control
    {
        if(state[B]) {
            robot::rightArm.setState(true);
        }
        else {
            robot::rightArm.setState(false);
        }
    }


    //intake control
    if(state[L1]) {
        robot::intake.spin(12000);
    }
    else if (state[L2]) {
        robot::intake.spin(-12000);
    }
    else {
        robot::intake.spin(0);
    }

    //arm control
    if(state[R1]) {
        robot::arm.spin(12000);
    }
    else if (state[R2]) {
        robot::arm.spin(-12000);
    }
    else {
        robot::arm.spin(0);
    }


}