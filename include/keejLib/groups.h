#pragma once
#include "main.h"

namespace lib
{
    class mtrs
    {
        private:
            pros::motor_brake_mode_e returnBrakeType(char brakeMode);

        protected:
            std::vector<okapi::Motor> motors;
            int size;

        public:
            mtrs(const std::vector<int> & ports);

            void spin(double volts);
            void spin(int index, int volts);
            void stop(okapi::AbstractMotor::brakeMode brakeMode) ;
            void setBrake(okapi::AbstractMotor::brakeMode brakeMode);
            void reset();
            int getPort(int index);
            double getSpeed(int index);
            double getSpeed();
            double getRotation();
            double getEfficiency();
    };


    class pis
    {
        private:
            std::vector<pros::ADIDigitalOut> pistons;
            bool state;
        
        public:
            pis(std::vector<pros::ADIDigitalOut> p, bool s);

            void toggle();
            void setState(bool iState);
            bool getState();
    };
}

    class diffy : public mtrs
    {
        public:
            using mtrs::mtrs;
            /**
             * @brief spin motors at different voltages
             * 
             * @param rvolt right side 
             * @param lvolt left side
             */

            void spinDiffy(double rvolt, double lvolt);

/**
 * @brief just drives forward lmao
 * 
 * @param voltages all motor power 
 */

            void spinDiffy(std::vector<double> voltages);
            std::vector<double> getDiffy();
    };
