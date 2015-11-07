#include "PackageHandler.h"
#include "MeHost_Packer.h"
#include "MotorParam.h"
#include "EncoderMotor.h"
#include "Wire.h"

extern EncoderMotor motor[2];

static void SendParam(uint8_t slot)
{
    uint8_t sendBuf[50];
    uint8_t paramBuf[42];
    float * paramListFloat;
    long  * paramListLong;

    paramBuf[0] = slot;
    paramBuf[1] = SHOW_PARAM;
    
    paramListFloat = (float *)&paramBuf[2];
    paramListLong = (long *)&paramBuf[2];

    motor[slot].param.GetParam(POS_P, paramListFloat + 0);
    motor[slot].param.GetParam(POS_I, paramListFloat + 1);
    motor[slot].param.GetParam(POS_D, paramListFloat + 2);
    motor[slot].param.GetParam(SPEED_P, paramListFloat + 3);
    motor[slot].param.GetParam(SPEED_I, paramListFloat + 4);
    motor[slot].param.GetParam(SPEED_D, paramListFloat + 5);
    motor[slot].param.GetParam(REDUCTION_RATIO, paramListFloat + 6);
    // motor[slot].param.GetParam(MAX_SPEED, paramListFloat + 7);
    motor[slot].param.GetParam(ENCODER_RESOLUTION, paramListLong + 8);
    paramListLong[9] = MotorParam::GetI2CAddr();

    MeHost_Pack(sendBuf, 50, 0x01, paramBuf, 42);
    Serial.write(sendBuf, 50);
}

static void TestParam(uint8_t slot, uint8_t * paramList)
{
    float * paramListFloat = (float *)paramList;
    long * paramListLong = (long *)paramList;

    motor[slot].SetPosPID(paramListFloat[0], 
                            paramListFloat[1], 
                            paramListFloat[2]);
    motor[slot].SetSpeedPID(paramListFloat[3], 
                          paramListFloat[4], 
                          paramListFloat[5]);

    motor[slot].param.SetParam(POS_P, paramListFloat + 0);
    motor[slot].param.SetParam(POS_I, paramListFloat + 1);
    motor[slot].param.SetParam(POS_D, paramListFloat + 2);
    motor[slot].param.SetParam(SPEED_P, paramListFloat + 3);
    motor[slot].param.SetParam(SPEED_I, paramListFloat + 4);
    motor[slot].param.SetParam(SPEED_D, paramListFloat + 5);
    motor[slot].param.SetParam(REDUCTION_RATIO, paramListFloat + 6);
    // motor[slot].param.SetParam(MAX_SPEED, paramListFloat + 7);
    motor[slot].param.SetParam(ENCODER_RESOLUTION, paramListLong + 8);
    MotorParam::SetI2CAddr(paramListLong[9]);
    
    Wire.begin(MotorParam::GetI2CAddr());
}

void PackageHandler(uint8_t * data, uint32_t length)
{
    uint8_t slot = data[0];
    uint8_t type = data[1];

    switch (type)
    {
    case GET_PARAM:
    {
        SendParam(slot);
        break;
    }
    case TEST_PARAM:
    {
        TestParam(slot, data + 2);
        break;
    }
    case SAVE_PARAM:
    {
        TestParam(slot, data + 2);
        motor[slot].param.Save();
        break;
    }
    case RESET:
    {
        motor[slot].Reset();
        break;
    }
    case RUN_STOP:
    {
        uint32_t temp = 0xFFFFFFFF;
        motor[slot].SetRunTime((float *)(&temp));
        motor[slot].SetMode(SPEED_MODE);
        motor[slot].SetSpeed(*((float *)(data + 2)));
        break;
    }
    case SPEED_TIME:
    {
        motor[slot].SetRunTime((float *)(data + 6));
        motor[slot].SetMode(SPEED_MODE);
        motor[slot].SetSpeed(*((float *)(data + 2)));
        break;
    }
    case GET_SPEED:
    {
        uint8_t temp[6] = {0};
        temp[0] = slot;
        temp[1] = GET_SPEED;
        *((float *)(temp + 2)) = motor[slot].GetRealSpeed();
        
        uint8_t buf[14] = {0};
        MeHost_Pack(buf, 14, 0x01, temp, 6);
        Wire.write(buf, 14);
        break;
    }
    case GET_POS:
    {
        uint8_t temp[6] = {0};
        temp[0] = slot;
        temp[1] = GET_POS;
        *((float *)(temp + 2)) = motor[slot].GetCurrentPos();
        
        uint8_t buf[14] = {0};
        MeHost_Pack(buf, 14, 0x01, temp, 6);
        Wire.write(buf, 14);
        break;
    }
    case MOVE:
    {
        uint32_t temp = 0xFFFFFFFF;
        motor[slot].SetRunTime((float *)(&temp));

        float pos = *((float *)(data + 2));
        float speed = *((float *)(data + 6));
        motor[slot].SetMode(POSITION_MODE);
        motor[slot].SetPosition(pos + motor[slot].GetCurrentPos());
        motor[slot].SetMaxSpeed(speed);
        break;
    }
    case MOVE_TO:
    {
        uint32_t temp = 0xFFFFFFFF;
        motor[slot].SetRunTime((float *)(&temp));
        
        float pos = *((float *)(data + 2));
        float speed = *((float *)(data + 6));
        motor[slot].SetMode(POSITION_MODE);
        motor[slot].SetPosition(pos);
        motor[slot].SetMaxSpeed(speed);
        break;
    }
    default:
        break;
    }
}
