#include "PID.h"

PID::PID(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;

    control = 0;
}

PID::~PID()
{
    ;
}

void PID::SetPID(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

void PID::SetControl(float _control)
{
    control = _control;
}

void PID::SetRange(float min, float max)
{
    min_control = min;
    max_control = max;
}

void PID::SetEnv(float _expect, float _real)
{
    real = _real;
    expect = _expect;
}

float PID::Compute(uint8_t type)
{
    error = expect - real;

    if (INCREMENTAL == type)
    {
        //  incremental digital PID
        float adjust = kp * (error - error_next)
                       + ki * error 
                       + kd * (error - 2 * error_next + error_last);
        control += adjust;
    }
    else
    {
        float sum = error + error_next + error_last;
        control = kp * error + ki * sum + kd * (error - error_next);
    }

    if (control > max_control)
    {
        control = max_control;
    }
    if (control < min_control)
    {
        control = min_control;
    }

    error_last = error_next;
    error_next = error;

    return control;
}
