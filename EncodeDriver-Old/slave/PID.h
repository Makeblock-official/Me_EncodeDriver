#ifndef _PID_H_
#define _PID_H_

#include "stdint.h"

#define INCREMENTAL     0x01
#define ABSOLUTE        0x02

class PID
{
public:
    PID(float p, float i, float d);
    ~PID();

    void SetControl(float _control);
    void SetPID(float p, float i, float d);
    void SetRange(float min, float max);
    void SetEnv(float _expect, float _real);
    float Compute(uint8_t type);

// private:
    float kp;
    float ki;
    float kd;

    float error;
    float error_next;
    float error_last;

    float expect;
    float real;
    float control;
    float max_control;
    float min_control;
};

#endif
