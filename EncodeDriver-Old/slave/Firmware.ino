#include "Encoder.h"
#include "PID.h"
#include "PackageHandler.h"
#include "EncoderMotor.h"
#include "MotorParam.h"
#include "MeHost_Packer.h"
#include "MeHost_Parser.h"
#include "TimerOne.h"
#include "Arduino.h"
#include "Wire.h"

#define     BASE_ADDR_PARAM_0          0x100   //  motor 0 parameters
#define     BASE_ADDR_PARAM_1          0x200   //  motor 1 parameters

uint8_t PIN_MOTOR_DIR1[2]   = {A0, A2};
uint8_t PIN_MOTOR_DIR2[2]   = {A1, A3};
uint8_t PIN_MOTOR_PWM[2]    = {6, 5};
uint8_t PIN_ENCODER_SIG1[2] = {3, 2};
uint8_t PIN_ENCODER_SIG2[2] = {10, 12};

EncoderMotor motor[2] = 
{
    EncoderMotor(PIN_MOTOR_DIR1[0], PIN_MOTOR_DIR2[0], 
                 PIN_MOTOR_PWM[0], 
                 PIN_ENCODER_SIG1[0], PIN_ENCODER_SIG2[0], 
                 BASE_ADDR_PARAM_0), 
    EncoderMotor(PIN_MOTOR_DIR1[1], PIN_MOTOR_DIR2[1], 
                 PIN_MOTOR_PWM[1], 
                 PIN_ENCODER_SIG1[1], PIN_ENCODER_SIG2[1], 
                 BASE_ADDR_PARAM_1)
};

void setup()
{
    Serial.begin(115200);
    // Serial.begin(9600);
    InitI2C();

    Timer1.initialize(TIME_STEP * 1000000);
    Timer1.attachInterrupt(TimerISR);
}

void TimerISR()
{
    sei();
    motor[0].Run();
    motor[1].Run();

    uint8_t temp[6] = {0};
    uint8_t buf[14] = {0};

    float diffPos = 0;

    //  send situation of motor[0] to PC
    temp[0] = 0;
    temp[1] = GET_SPEED;
    *((float *)(temp + 2)) = motor[0].GetRealSpeed();
    MeHost_Pack(buf, 14, 0x01, temp, 6);
    Serial.write(buf, 14);

    diffPos = motor[0].GetDiffPos();
    if(diffPos == diffPos)
    {
        temp[1] = GET_DIFF_POS;
        *((float *)(temp + 2)) = diffPos;
        MeHost_Pack(buf, 14, 0x01, temp, 6);
        Serial.write(buf, 14);
    }

    //  send situation of motor[1] to PC
    temp[0] = 1;
    temp[1] = GET_SPEED;
    *((float *)(temp + 2)) = motor[1].GetRealSpeed();
    MeHost_Pack(buf, 14, 0x01, temp, 6);
    Serial.write(buf, 14);

    diffPos = motor[1].GetDiffPos();
    if(diffPos == diffPos)
    {
        temp[1] = GET_DIFF_POS;
        *((float *)(temp + 2)) = diffPos;
        MeHost_Pack(buf, 14, 0x01, temp, 6);
        Serial.write(buf, 14);
    }
}

#define BUF_SIZE 128
uint8_t pcBuf[BUF_SIZE];
uint8_t baseBuf[BUF_SIZE];
MeHost_Parser pcParser = MeHost_Parser();
MeHost_Parser baseParser = MeHost_Parser();

void loop()
{
    //  receive the data from PC
    while (Serial.available())
    {
        pcParser.PushByte(Serial.read());
    }
    pcParser.Run();
    if (pcParser.PackageReady())
    {
        PackageHandler(pcBuf, pcParser.GetData(pcBuf, BUF_SIZE));
    }
    while(!Serial.available());
}


//  initialize the I2C module
void InitI2C()
{
    //  register I2C handler
    // MotorParam::SetI2CAddr(0x09);
    Wire.begin(MotorParam::GetI2CAddr());

    Wire.onReceive(I2CReceiveHandler);
    Wire.onRequest(I2CRequestHandler);
}

//  handle the I2C receive event
void I2CReceiveHandler(int n)
{
    sei();
    while (Wire.available())
    {
        baseParser.PushByte(Wire.read());
    }
    baseParser.Run();
    if (1 == baseParser.PackageReady())
    {
        PackageHandler(baseBuf, baseParser.GetData(baseBuf, BUF_SIZE));
    }
}

//  handle the I2C request event
void I2CRequestHandler()
{
    baseParser.Run();
    if (1 == baseParser.PackageReady())
    {
        PackageHandler(baseBuf, baseParser.GetData(baseBuf, BUF_SIZE));
    }
}
