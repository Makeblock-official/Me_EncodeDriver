#ifndef ENCODED_MOTOR_H
#define ENCODED_MOTOR_H

#define HOLD 0x40
#define SYNC 0x80
// move state and function
#define CMD_RESET 0x00
#define CMD_STOP 0x00
#define CMD_MOVE_TO 0x01
#define CMD_BREAK 0x02
#define CMD_MOVE_SPD 0x03
#define CMD_MOVE_TO_SPD 0x04
// config function
#define CMD_SET_PID 0x10
#define CMD_SET_HOLD 0x11
#define CMD_SET_POWER 0x12
#define CMD_SET_MODE 0x13
#define CMD_SET_PWM 0x14
#define CMD_SET_RATIO 0x15
#define CMD_SET_PULSE 0x16
#define CMD_SET_DEVID 0x17
// get motor status
#define CMD_GET_PID 0x20
#define CMD_GET_POWER 0x21
#define CMD_GET_POS 0x22
#define CMD_GET_SPEED 0x23
#define CMD_GET_RATIO 0x24
#define CMD_GET_PULSE 0x25

#define EEPROM_IF_HAVEPID_CHECK1 0XAB
#define EEPROM_IF_HAVEPID_CHECK2 0XCD
#define EEPROM_PID_START 0XAB
#define EEPROM_PID_MID 0XCD
#define EEPROM_PID_END 0XEF

#define EEPROM_START_POS         0
#define STORE_START_ADDR         EEPROM_START_POS + 2      //PID CHECK
#define STORE_DEVID_ADDR         STORE_START_ADDR + 1      //start data

#define STORE_PIDS0_ADDR         STORE_DEVID_ADDR + 2      //dev id
#define STORE_RATIO0_ADDR        STORE_PIDS0_ADDR + 16     //pids0 data, 4 float
#define STORE_PLUS0_ADDR         STORE_RATIO0_ADDR + 4     //ratio0 data


#define STORE_MID_ADDR           STORE_PLUS0_ADDR + 2     //devid0 data

#define STORE_PIDS1_ADDR         STORE_MID_ADDR + 1        //mid data
#define STORE_RATIO1_ADDR        STORE_PIDS1_ADDR + 16     //pids1 data, 4 float
#define STORE_PLUS1_ADDR         STORE_RATIO1_ADDR + 4     //ratio1 data

#define STORE_END_ADDR           STORE_PLUS1_ADDR + 2      //devid1 data

enum
{
  MOTOR_1 = 0x00,
  MOTOR_2,
};

typedef struct{
  int index;
  int devid;
  int pulse;
  int speed;      //rpm

  int stopCount;
  // output max pwm
  int power;
  // current output pwm
  int pwm;
  int targetSpd;

  long pos;
  long posLast;
  long posSpeed;
  // target position
  long targetPos;
  // position pid
  float p;
  float i;
  float d;
  float s;
  float PTerm;
  float ITerm;
  float DTerm;
  float STerm;
  float ratio;

  // state
  unsigned char hold;
  unsigned char state;
  unsigned char mode;
}EMotor;

typedef struct{
  byte  start;
  int devid;
  float p0;
  float i0;
  float d0;
  float s0;
  float ratio0;
  int pulse0;
  byte  mid;
  float p1;
  float i1;
  float d1;
  float s1;
  float ratio1;
  int pulse1;
  byte end;
}EPids;
#endif


