#include "main.h"
#include "PID_Controller.hpp"
#include <cmath> // for abs()

using namespace std;

/**
 * @brief Construct a new pid controller::pid controller object 
 * 
 */
PID_Controller::PID_Controller() {
    _error = _lastError = _sumOfError = _maxSumOfError = _sumErrorRange = 0;
    _outputRange = 1.0;
    _target = 0;
    _outputValue = 0;
}

/**
 * @brief Destroy the pid controller::pid controller object
 * 
 */
PID_Controller::~PID_Controller() {}

/**
 * @brief Construct a new pid controller::pid controller object
 * 
 * @param values PID values assigned by user (kp, ki, kd)
 */
PID_Controller::PID_Controller(PIDvalues values) {
    _values = values;
    _error = _lastError = _sumOfError = _maxSumOfError = _sumErrorRange = 0;
    _outputRange = 1.0;
    _target = 0;
    _outputValue = 0;
}

/**
 * @brief reassign values to the PID controller
 * 
 * @param values the new values being assigned (kp, ki, kd)
 */
void PID_Controller::Set_Parameters(PIDvalues values) {
    _values = values;
}

/**
 * @brief set target angle for PID_Controller to model
 * 
 * @param target target angle for IMU/gyro
 */
void PID_Controller::SetTarget(double target) {
    _target = target;
}

/**
 * @brief setMaxSumofError (limit the sum of error)
 * 
 * @param maxSumError 
 */
void PID_Controller::SetMaxSumOfError(double maxSumError) {
    _maxSumOfError = abs(maxSumError);
}

/**
 * @brief calculates delta between target and current reading
 * not sure why you need it but if you do here it is!
 * @return double 
 */
double PID_Controller::GetError() {
    return _error;
}

/**
 * @brief calculates output on a scale of -1 to 1 on correction scale of system using PID Controller
 * 
 * @param currentReading current reading of gyro/IMU
 * @return double correction on scale of -1 to 1
 */
double PID_Controller::Calculate(double currentReading) {
    _error = _target - currentReading;

    if (abs(_error) < _sumErrorRange) {
        _sumOfError += _error * _values.ki;
    }

    if (squiggles::sgn(_error) != squiggles::sgn(_lastError)) {
        _sumOfError = 0;
    }

    _sumOfError = abs(_sumOfError) > _maxSumOfError ? squiggles::sgn(_sumOfError) * _maxSumOfError : _sumOfError;

    _outputValue = _values.kp * _error + _values.kd * (_error - _lastError) + _sumOfError;
    _lastError = _error;

    return _outputValue;
}