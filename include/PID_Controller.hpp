#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

struct PIDvalues {
    double kp;
    double ki;
    double kd;
};

class PID_Controller {
private:
    PIDvalues _values;
    double _target; 
    double _error;
    double _lastError;
    double _sumOfError;
    double _maxSumOfError;
    double _sumErrorRange;
    double _outputRange;
    double _outputValue;

public:
    PID_Controller();
    ~PID_Controller();
    PID_Controller(PIDvalues values);

    void Set_Parameters(PIDvalues values);
    void SetTarget(double target);
    void SetMaxSumOfError(double maxSumError);
    double GetError();
    double Calculate(double currentReading);
    void turnToAngle(double angle, double turnVoltage, PIDvalues values);
};

#endif 