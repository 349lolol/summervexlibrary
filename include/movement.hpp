#include "PID_Controller.hpp"
#include <cstdint>

void rotationTurn(double angle, double turnVoltage, double momentum, uint32_t timeout, PIDvalues values);

void moveDistance(double distance, double heading, int voltage, PIDvalues values);