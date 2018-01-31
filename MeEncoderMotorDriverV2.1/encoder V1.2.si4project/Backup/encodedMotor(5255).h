/**
* \par Copyright (C), 2012-2016, MakeBlock
* @file 	slave.ino
* @author 	MakeBlock
* @version 	V1.0.0
* @date 	2016/05/06
* @brief 	Description: this file is encoded motor driver code.
* *
Function List:
* *
\par History:
* <pre>
* <Author> 		<Time>		 	<Version> 			<Descr>
* Mark Yan 		2016/05/06 			1.0.0 			build the new.
* Payton 		2017/07/13 			2.0.0 			modif control arithmetic
* </pre>
*
*/

#ifndef ENCODED_MOTOR_H
#define ENCODED_MOTOR_H

// move state and function
#define CMD_RESET         0x00
#define CMD_MOVE_TO       0x01
#define CMD_MOVE          0x02
#define CMD_MOVE_SPD      0x03
#define CMD_STOP          0x05

// config function
#define CMD_SET_SPEED_PID 0x10
#define CMD_SET_POS_PID   0x11
#define CMD_SET_CUR_POS   0x12
#define CMD_SET_MODE      0x13
#define CMD_SET_PWM       0x14
#define CMD_SET_RATIO     0x15
#define CMD_SET_PULSE     0x16
#define CMD_SET_DEVID     0x17

// get motor status
#define CMD_GET_SPEED_PID       0x20
#define CMD_GET_POS_PID         0x21
#define CMD_GET_POS             0x23
#define CMD_GET_SPEED           0x24
#define CMD_GET_RATIO           0x25
#define CMD_GET_PULSE           0x26
#define CMD_GET_LOCK_STATE      0x27

//Motor parameter settings
#define MAX_ENCODER_SPEED    	220
#define MIN_ENCODER_SPEED    	20
#define ENCODER_POS_DEADBAND    2

#define MOTOR_0             	0
#define MOTOR_1             	1
#define MOTOR0              	MOTOR_0
#define MOTOR1              	MOTOR_1

#define LOCK_STATE                0
#define RELEASE_STATE             1

#define MOTION_I2C_MODE         0x00
#define MOTION_PWM_MODE         0x01
#define MOTION_PWM_I2C_PWM      0x02
#define MOTION_IDLE_MODE		0x03
#define MOTION_POS_MODE			0x04
#define MOTION_SPEED_MODE		0x05


//EEPROM data
#define EEPROM_IF_HAVEPID_CHECK1 0XCE //EncodeDriver-New -> 0xCD
#define EEPROM_IF_HAVEPID_CHECK2 0xAB
#define EEPROM_PID_START 0xAB
#define EEPROM_PID_MID 0xCD
#define EEPROM_PID_END 0xEF

#define EEPROM_START_POS         0
#define STORE_START_ADDR         EEPROM_START_POS + 2      //PID CHECK
#define STORE_DEVID_ADDR         STORE_START_ADDR + 1      //start data
#define STORE_PID0_ADDR          STORE_DEVID_ADDR + 1      //dev id

#define STORE_RATIO0_ADDR        STORE_PID0_ADDR + 24      //speed and pos pid data, 6 float
#define STORE_PLUS0_ADDR         STORE_RATIO0_ADDR + 4     //ratio0 data


#define STORE_MID_ADDR           STORE_PLUS0_ADDR + 2      //pulse0 data

#define STORE_PID1_ADDR         STORE_MID_ADDR + 1         //mid data

#define STORE_RATIO1_ADDR        STORE_PID1_ADDR + 24      //speed and pos pid data, 6 float
#define STORE_PLUS1_ADDR         STORE_RATIO1_ADDR + 4     //ratio1 data

#define STORE_END_ADDR           STORE_PLUS1_ADDR + 2      //pulse1 data

typedef struct
{
  float P, I, D;
  float setpoint, output, integral, differential, last_error;
} PID;

typedef struct
{
  uint8_t slot;
  uint8_t mode;
  boolean mode_change;

  int16_t tar_pwm;
  int16_t cur_pwm;
  uint16_t encoder_pulse;

  float ratio;
  float current_speed;            //rpm
  float target_speed;
  float pre_speed;

  long current_pos;               //angle
  long previous_pos;              
  long target_pos;
  long pulse;

  PID  PID_speed;
  PID  PID_pos;
}encoder_data_type;

typedef struct
{
  uint8_t start_data;
  uint8_t devid;
  float speed0_p;
  float speed0_i;
  float speed0_d;
  float pos0_p;
  float pos0_i;
  float pos0_d;
  float ratio0;
  uint16_t pulse0;
  uint8_t mid_data;
  float speed1_p;
  float speed1_i;
  float speed1_d;
  float pos1_p;
  float pos1_i;
  float pos1_d;
  float ratio1;
  uint16_t pulse1;
  uint8_t end_data;
}encoder_eeprom_type;

#endif


