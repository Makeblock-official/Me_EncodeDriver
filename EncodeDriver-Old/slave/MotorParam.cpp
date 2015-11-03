#include "MotorParam.h"
#include <avr/eeprom.h>
#include <Arduino.h>
// #include "C:\\Program Files (x86)\\Arduino\\libraries\\EEPROM\\EEPROM.h"

#define PARAM_INIT_MODE_ADDR    0x000   //  the mode to init param (load/save)
#define LOAD_PARAM  0x00    //  load param from EEPROM in setup function
#define SAVE_PARAM  0xFF    //  save param to EEPROM in setup function 
                            //  (A factory fresh EEPROM should have 0xFF in all bytes)


#define     BASE_ADDR_PARAM_G          0x040   //  global parameters

//  offset of global param
#define     OFFSET_I2C_ADDR            0x000

//  global param
// static uint8_t i2cAddr = 0x00;

//  offset of motor param
#define     OFFSET_POS_P               0x000
#define     OFFSET_POS_I               0x004
#define     OFFSET_POS_D               0x008
#define     OFFSET_SPEED_P             0x00C
#define     OFFSET_SPEED_I             0x010
#define     OFFSET_SPEED_D             0x014
#define     OFFSET_ENCODER_RESOLUTION  0x018
#define     OFFSET_REDUCTION_RATIO     0x01C
// #define     OFFSET_MAX_SPEED           0x020

uint8_t MotorParam::i2cAddr = 0x09;

class EEPROMClass
{
  public:
    uint8_t read(int);
    void write(int, uint8_t);
};

uint8_t EEPROMClass::read(int address)
{
    return eeprom_read_byte((unsigned char *) address);
}

void EEPROMClass::write(int address, uint8_t value)
{
    eeprom_write_byte((unsigned char *) address, value);
}

EEPROMClass EEPROM;

MotorParam::MotorParam(uint32_t _baseAddr)
{
    baseAddr = _baseAddr;

    if (SAVE_PARAM == EEPROM.read(baseAddr/*PARAM_INIT_MODE_ADDR*/))
    {
        RestoreFactoryParam();
        // EEPROM.write(PARAM_INIT_MODE_ADDR, LOAD_PARAM);
    }

    Load();
}

//  save data to EEPROM
//  return: the size saved, 0 for error
static uint32_t EEPROM_Save(uint32_t addr, uint8_t * data, uint32_t size)
{
    //  the EEPROM size is 1K (0x000-0x3FF)
    if (addr + size < 0x400)
    {
        for (int i = 0; i < size; ++i)
        {
            EEPROM.write(addr + i, data[i]);
        }
        return size;
    }
    else
    {
        return 0;
    }
}

//  load data from EEPROM
//  return: the size loaded, 0 for error.
static uint32_t EEPROM_Load(uint32_t addr, uint8_t * data, uint32_t size)
{
    //  the EEPROM size is 1K (0x000-0x3FF)
    if (addr + size < 0x400)
    {
        for (int i = 0; i < size; ++i)
        {
            data[i] = EEPROM.read(addr + i);
        }
        return size;
    }
    else
    {
        return 0;
    }
}

//  save all the param to EEPROM
void MotorParam::Save()
{
    EEPROM_Save(BASE_ADDR_PARAM_G + OFFSET_I2C_ADDR , (uint8_t *)&i2cAddr          , 1);

    EEPROM_Save(baseAddr + OFFSET_POS_P             , (uint8_t *)&posP             , 4);
    EEPROM_Save(baseAddr + OFFSET_POS_I             , (uint8_t *)&posI             , 4);
    EEPROM_Save(baseAddr + OFFSET_POS_D             , (uint8_t *)&posD             , 4);
    EEPROM_Save(baseAddr + OFFSET_SPEED_P           , (uint8_t *)&speedP           , 4);
    EEPROM_Save(baseAddr + OFFSET_SPEED_I           , (uint8_t *)&speedI           , 4);
    EEPROM_Save(baseAddr + OFFSET_SPEED_D           , (uint8_t *)&speedD           , 4);
    EEPROM_Save(baseAddr + OFFSET_ENCODER_RESOLUTION, (uint8_t *)&encoderResolution, 4);
    EEPROM_Save(baseAddr + OFFSET_REDUCTION_RATIO   , (uint8_t *)&reductionRatio   , 4);
    // EEPROM_Save(baseAddr + OFFSET_MAX_SPEED         , (uint8_t *)&maxSpeed         , 4);
}

//  load all the param from EEPROM
void MotorParam::Load()
{
    EEPROM_Load(BASE_ADDR_PARAM_G + OFFSET_I2C_ADDR , (uint8_t *)&i2cAddr          , 1);

    EEPROM_Load(baseAddr + OFFSET_POS_P             , (uint8_t *)&posP             , 4);
    EEPROM_Load(baseAddr + OFFSET_POS_I             , (uint8_t *)&posI             , 4);
    EEPROM_Load(baseAddr + OFFSET_POS_D             , (uint8_t *)&posD             , 4);
    EEPROM_Load(baseAddr + OFFSET_SPEED_P           , (uint8_t *)&speedP           , 4);
    EEPROM_Load(baseAddr + OFFSET_SPEED_I           , (uint8_t *)&speedI           , 4);
    EEPROM_Load(baseAddr + OFFSET_SPEED_D           , (uint8_t *)&speedD           , 4);
    EEPROM_Load(baseAddr + OFFSET_ENCODER_RESOLUTION, (uint8_t *)&encoderResolution, 4);
    EEPROM_Load(baseAddr + OFFSET_REDUCTION_RATIO   , (uint8_t *)&reductionRatio   , 4);
    // EEPROM_Load(baseAddr + OFFSET_MAX_SPEED         , (uint8_t *)&maxSpeed         , 4);
}

void MotorParam::RestoreFactoryParam()
{
    i2cAddr           = 0x09;
    posP              = 0.5;
    posI              = 0;
    posD              = 0.1;
    speedP            = 0.5;
    speedI            = 0.2;
    speedD            = 0;
    encoderResolution = 48;
    reductionRatio    = 25.76852;   //74.83178;
    // maxSpeed          = 0;

    Save();
}

//  set i2c address
void MotorParam::SetI2CAddr(uint8_t addr)
{
    i2cAddr = addr;
    // Wire.begin(i2cAddr);
}

//  get i2c address
uint8_t MotorParam::GetI2CAddr()
{
    return i2cAddr;
}

void MotorParam::GetParam(uint8_t name, void * value)
{
    switch (name)
    {
    case POS_P:
        *((float *)value) = posP;
        return;
    case POS_I:
        *((float *)value) = posI;
        return;
    case POS_D:
        *((float *)value) = posD;
        return;
    case SPEED_P:
        *((float *)value) = speedP;
        return;
    case SPEED_I:
        *((float *)value) = speedI;
        return;
    case SPEED_D:
        *((float *)value) = speedD;
        return;
    case ENCODER_RESOLUTION:
        *((long *)value) = encoderResolution;
        return;
    case REDUCTION_RATIO:
        *((float *)value) = reductionRatio;
        return;
    // case MAX_SPEED:
    //     *((float *)value) = maxSpeed;
    //     return;
    default:
        return;
    }
}

void MotorParam::SetParam(uint8_t name, void * value)
{
    switch (name)
    {
    case POS_P:
        posP = *((float *)value);
        return;
    case POS_I:
        posI = *((float *)value);
        return;
    case POS_D:
        posD = *((float *)value);
        return;
    case SPEED_P:
        speedP = *((float *)value);
        return;
    case SPEED_I:
        speedI = *((float *)value);
        return;
    case SPEED_D:
        speedD = *((float *)value);
        return;
    case ENCODER_RESOLUTION:
        encoderResolution = *((long *)value);
        return;
    case REDUCTION_RATIO:
        reductionRatio = *((float *)value);
        return;
    // case MAX_SPEED:
    //     maxSpeed = *((float *)value);
    //     return;
    default:
        return;
    }
}
