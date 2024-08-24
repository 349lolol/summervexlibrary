#include "robot.hpp"

using namespace glb;
using namespace robot;


namespace cata {
    constexpr double halfPos = 25681;
    constexpr double load = 25681;
    pros::Task* cataTask = nullptr;

    enum cataState {
        firing,
        reloading,
        half,
        idle,
        toggeled,
        delayed,
        off
    };
    
    cataState state = idle;
    lib::pid pid({
        .p = 2.2,
        .i = 0.4, 
        .d = 0, 
        .tolerance = 0.05, 
        .integralThreshold = 15, 
        .maxIntegral = 20
    }, 0);
    lib::timer delay;
    lib::timer inRange; 
    double target = load;

    void cataControl() {    
        
        
    }

    void halfway() {

    }

    void fire() {

    }

    void toggle() {

    }

    void cut() {

    }

    void init() {

    }
}