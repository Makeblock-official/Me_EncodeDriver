#include "EncoderMotor.h"
#include "Arduino.h"
#include "MeHost_Packer.h"

#define MAX_SPEED 16000

EncoderMotor::EncoderMotor(uint8_t _pinDir1, uint8_t _pinDir2, uint8_t _pinPWM, 
                           uint8_t _pinEncoder1, uint8_t _pinEncoder2, 
                           uint32_t _baseAddr)
                :encoder(_pinEncoder1, _pinEncoder2), 
                param(_baseAddr), 
                speed_pid(0, 0, 0), 
                position_pid(0, 0, 0)
{
    pinDir1 = _pinDir1;
    pinDir2 = _pinDir2;
    pinPWM  = _pinPWM;

    mode = SPEED_MODE;

    targetSpeed = 0;
    realSpeed = 0;
    maxSpeed = MAX_SPEED;
    speed_pid.SetRange(-300, 300);

    memset(&runTime, 0xFF, 4);

    targetPos = 0;
    currentPos = 0;
    absoluteCounter = 0;
    position_pid.SetRange(-250, 250);

    // param.RestoreFactoryParam();
    param.Load();
    float p = 0;
    float i = 0;
    float d = 0;
    param.GetParam(POS_P, &p);
    param.GetParam(POS_I, &i);
    param.GetParam(POS_D, &d);
    position_pid.SetPID(p, i, d);
    param.GetParam(SPEED_P, &p);
    param.GetParam(SPEED_I, &i);
    param.GetParam(SPEED_D, &d);
    speed_pid.SetPID(p, i, d);
}

EncoderMotor::~EncoderMotor()
{
    ;
}

void EncoderMotor::Reset()
{
    currentPos = 0;
    absoluteCounter = 0;
    targetPos = 0;
    targetSpeed = 0;
    memset(&runTime, 0xFF, 4);
    mode = SPEED_MODE;
}

void EncoderMotor::Run()
{
    // int32_t counter = encoder.read();
    // encoder.write(0);
    int32_t counter = encoder.ReadAndReset();
    // Serial.println(counter);

    float ratio = 0;
    long resolution = 0;
    param.GetParam(REDUCTION_RATIO, &ratio);
    param.GetParam(ENCODER_RESOLUTION, &resolution);
    realSpeed = (float)counter / resolution * 60.0 / TIME_STEP / ratio;
    absoluteCounter += counter;
    currentPos = (float)absoluteCounter * 360.0 / resolution / ratio;
    

    if (POSITION_MODE == mode)
    {
        if (fabs(targetPos - currentPos) < 2)
        {
            SetSpeed(0);
        }
        else
        {
            position_pid.SetEnv(targetPos, currentPos);
            float speed = position_pid.Compute(ABSOLUTE);
            if (speed > maxSpeed)
            {
                speed = maxSpeed;
            }
            else if (speed < -maxSpeed)
            {
                speed = -maxSpeed;
            }
            SetSpeed(speed);
        }
    }

    //  if the raw value of runTime is not 0xFFFFFFFF
    if (runTime == runTime)
    {
        if (runTime < 0)
        {
            SetSpeed(0);
        }
        runTime -= TIME_STEP;
    }

    speed_pid.SetEnv(targetSpeed, realSpeed);
    controlSpeed = speed_pid.Compute(INCREMENTAL);

    RunMotor(controlSpeed);
}

float EncoderMotor::GetCurrentPos()
{
    return currentPos;
}

float EncoderMotor::GetRealSpeed()
{
    return realSpeed;
}

void EncoderMotor::SetSpeed(float _targetSpeed)
{
    targetSpeed = _targetSpeed;
    if (fabs(_targetSpeed - targetSpeed) > 30)
    {
        controlSpeed = targetSpeed;
        // controlSpeed = controlSpeed / GetRealSpeed() * targetSpeed;
        speed_pid.SetControl(controlSpeed);
        RunMotor(controlSpeed);
    }
}

void EncoderMotor::SetRunTime(float * _runTime)
{
    memcpy(&runTime, _runTime, 4);
}

void EncoderMotor::SetMaxSpeed(float _maxSpeed)
{
    maxSpeed = _maxSpeed;
}

void EncoderMotor::SetPosition(float _targetPos)
{
    targetPos = _targetPos;
}

float EncoderMotor::GetDiffPos()
{
    if (POSITION_MODE == mode)
    {
        return (currentPos - targetPos);
    }
    else
    {
        float temp = 0;
        *((uint32_t *)&temp) = 0xFFFFFFFF;
        return temp;
    }
}

//  Run motor in open-loop
//  param:  speed: control speed
void EncoderMotor::RunMotor(float speed)
{
    float ratio = 0;
    param.GetParam(REDUCTION_RATIO, &ratio);
    float PWM = speed / 80 * ratio;

    if (PWM > 100)
    {
        PWM = 100;
    }
    if (PWM < -100)
    {
        PWM = -100;
    }
    
    if (PWM > 0)
    {
        digitalWrite(pinDir1, HIGH);
        digitalWrite(pinDir2, LOW);
    }
    else
    {
        digitalWrite(pinDir1, LOW);
        digitalWrite(pinDir2, HIGH);
    }
    //  use 254 instead of 255 to avoid "always on" 
    //  which may cause fault of H-bridge
    analogWrite(pinPWM, fabs(PWM) * 254.0 / 100.0);
}

//  scram
void EncoderMotor::Scram()
{
    RunMotor(0);
    while (1);
}

//  set the work mode
void EncoderMotor::SetMode(uint8_t _mode)
{
    mode = _mode;
    if (SPEED_MODE == mode)
    {
        maxSpeed = MAX_SPEED;
    }
}


void EncoderMotor::SetSpeedPID(float _p, float _i, float _d)
{
    speed_pid.SetPID(_p, _i, _d);
}

void EncoderMotor::SetPosPID(float _p, float _i, float _d)
{
    position_pid.SetPID(_p, _i, _d);
}
