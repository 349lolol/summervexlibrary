#pragma once
#include "main.h"
#include "lib.hpp"
#include <cstdint>
#include "okapi/api.hpp"

int absoluteValue(int i) {
    return i < 0 ? -i : i;
}

int8_t convertToInt8(int value) {
    return static_cast<int8_t>(value);
}


lib::mtrs::mtrs(const std::vector<int> & ports)
{
    for (int i : ports)
    {
        const int8_t k = convertToInt8((i));
        if(i < 0) {
            okapi::Motor temp(std::abs(k), true, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
            motors.push_back(temp);
        } 
        else {
            okapi::Motor temp(std::abs(k), false, okapi::AbstractMotor::gearset::blue, okapi::AbstractMotor::encoderUnits::degrees);
            motors.push_back(temp);
        }
        
    }
    size = ports.size();
}

void lib::mtrs::spin(double volts) 
{
    for (int i=0; i < size; i++)
    {
        motors[i].moveVoltage(volts);
    }
}

void lib::mtrs::spin(int index, int volts)
{
    motors[index].moveVoltage(volts);
}


void lib::mtrs::stop(okapi::AbstractMotor::brakeMode brakeMode) 
{

    for (int i=0; i < size; i++)
    {
        motors[i].setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    }
}

void lib::mtrs::setBrake(okapi::AbstractMotor::brakeMode brakeMode) 
{
    for (int i=0; i < size; i++)
    {
        motors[i].setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
    }
}

int lib::mtrs::getPort(int index) {
    return motors[index].getPort();
}

double lib::mtrs::getSpeed(int index) {
    return motors[index].getActualVelocity();
}

double lib::mtrs::getSpeed() 
{
    double vel = 0;

    for (int i=0; i < size; i++)
    {
        vel += motors[i].getActualVelocity();
    }
    
    return(vel/size);
}


double lib::mtrs::getRotation() 
{
    double rotation = 0;

    for (int i=0; i < size; i++)
    {
        rotation += motors[i].getPosition();
    }
    
    return(rotation/size);
}

double lib::mtrs::getEfficiency() 
{
    double result = 0;
    for (auto motor: motors)
    {
        result += motor.getEfficiency();
    }
    return result / size;
}

void lib::mtrs::reset() 
{
    for (int i=0; i < size; i++)
    {
        motors[i].tarePosition();
    }
}

void lib::diffy::spinDiffy(double rvolt, double lvolt) 
{
    int half = size/2;

    for (int i=0; i < half; i++)
    {
        motors[i].moveVoltage(rvolt * 12000);
        motors[i + half].moveVoltage(lvolt * 12000);
    }
}

void lib::diffy::spinDiffy(std::vector<double> voltages) 
{
    int half = size/2;

    for (int i=0; i < half; i++)
    {
        motors[i].moveVoltage(voltages[0]);
        motors[i + half].moveVoltage(voltages[1]);
    }
}

std::vector<double> lib::diffy::getDiffy() 
{
    double dl = 0;
    double dr = 0;
    int half = size/2;
    
    for (int i=0; i < half; i++)
    {
        // std::cout << motors[0].get_encoder_units() << std::endl;
        dl += motors[i].getPosition();
        dr += motors[i + half].getPosition();
    }
    
    return(std::vector<double> {dr/half, dl/half});
}

lib::pis::pis(std::vector<pros::ADIDigitalOut> p, bool s) : pistons(p), state(s)
{
    setState(s);
}

void lib::pis::toggle() 
{
    state = !state;

    for(int i = 0; i < pistons.size(); i++)
    {
        pistons[i].set_value(state);
    }
}

void lib::pis::setState(bool iState) 
{
    state = iState;

    for(int i = 0; i < pistons.size(); i++)
    {
        pistons[i].set_value(state);
    }
}

bool lib::pis::getState()
{
    return(state);
}