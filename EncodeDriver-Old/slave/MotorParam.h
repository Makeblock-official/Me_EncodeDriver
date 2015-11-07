#ifndef _PARAM_H_
#define _PARAM_H_

#include "stdint.h"

//  offset of motor param
#define     POS_P               0x000
#define     POS_I               0x001
#define     POS_D               0x002
#define     SPEED_P             0x003
#define     SPEED_I             0x004
#define     SPEED_D             0x005
#define     ENCODER_RESOLUTION  0x006
#define     REDUCTION_RATIO     0x007
#define     MAX_SPEED           0x008

#define     MAGIC_WORD_VALUE    0x65392964

class MotorParam
{
public:
    MotorParam(uint32_t _baseAddr);
    void Save();
    void Load();
    void GetParam(uint8_t name, void * value);
    void SetParam(uint8_t name, void * value);
    //  restore factory parameters
    void RestoreFactoryParam();

    static uint8_t GetI2CAddr();
    static void SetI2CAddr(uint8_t addr);

private:
    static uint8_t i2cAddr;
    static uint32_t magicWord;

    //  parameters of motor
    float posP;
    float posI;
    float posD;
    float speedP;
    float speedI;
    float speedD;
    long  encoderResolution;
    float reductionRatio;
    // float maxSpeed;
    //  base address of parameters
    uint32_t baseAddr;
};

#endif
