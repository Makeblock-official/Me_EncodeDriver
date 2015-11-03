#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stdint.h"
#include "Encoder.h"
#include "MotorParam.h"
#include "PID.h"

//  time step of timer (second)
#define TIME_STEP 0.03

#define SPEED_MODE      0x01
#define POSITION_MODE   0x02

class EncoderMotor
{
public:
    EncoderMotor(uint8_t _pinDir1, uint8_t _pinDir2, uint8_t _pinPWM, 
                 uint8_t _pinEncoder1, uint8_t _pinEncoder2, 
                 uint32_t _baseAddr);
    ~EncoderMotor();

    void Reset();
    void Run();
    //  scram the motor (for emergency situation)
    void Scram();
    //  get the real speed (rpm)
    float GetRealSpeed();
    //  set target speed
    void SetSpeed(float _targetSpeed);
    //  set running time
    void SetRunTime(float * _runTime);
    //  set max speed
    void SetMaxSpeed(float _maxSpeed);
    //  set the work mode
    void SetMode(uint8_t _mode);

    //  get current position
    float GetCurrentPos();
    //  set target position
    void SetPosition(float _targetPos);
    //  get the difference between the target and current position
    float GetDiffPos();

    void SetSpeedPID(float _p, float _i, float _d);
    void SetPosPID(float _p, float _i, float _d);

    //  parameters of motor
    MotorParam param;

private:
    //  encoder of motor
    Encoder encoder;
    //  PINs to control motor
    uint8_t pinDir1;
    uint8_t pinDir2;
    uint8_t pinPWM;

    //  mode: speed/position
    uint8_t mode;

    //  target speed
    float targetSpeed;
    //  real speed
    float realSpeed;
    //  control speed
    float controlSpeed;
    //  max speed
    float maxSpeed;
    //  run time
    float runTime;
    //  PID controller of speed
    PID speed_pid;

    //  target position
    float targetPos;
    //  current position
    float currentPos;
    long absoluteCounter;
    //  PID controller of position
    PID position_pid;

// public:
    //  Run motor in open-loop
    void RunMotor(float speed);
};

#endif
