#include <Arduino.h>
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include "encodedMotor.h"

//Print message priority
//#define DEBUG_INFO_ERROR
//#define DEBUG_INFO_HIGH  
//#define DEBUG_INFO_MID
//#define DEBUG_INFO_LOW
//#define DEBUG_INFO

//Common parameter configuration
#define DEFALUT_I2C_ADDR    0x09
#define PULSE_PER_C         8
#define MOTOR_BOTH          2

//GPIO configuration
#define INT_1_PIN     3
#define DIR_1_PIN     A2
#define MOTOR_1_PWM   6
#define MOTOR_1_H1    7

#define INT_2_PIN     2
#define DIR_2_PIN     A3
#define MOTOR_2_PWM   5
#define MOTOR_2_H1    8

//Interrupt Configuration
#define MOTOR_1_IRQ         INT1_vect  
#define MOTOR_2_IRQ         INT0_vect
#define MOTOR_1_PIN         PINC
#define MOTOR_1_DIR         PINC2 
#define MOTOR_2_PIN         PINC
#define MOTOR_2_DIR         PINC3

/****************************************************************************************************
 * I2C slave code
 * These codes used to start the i2c slave service and its command processing, If it is not
 * necessary, Do not modify this part of the code
****************************************************************************************************/
union
{
  byte byteVal[16];
  int16_t intVal[8];
  float floatVal[4];
  long longVal[4];
}val;

char bufI2C[32];
int16_t rd,wr;

/**
 * \par Function
 *   I2C_init
 * \par Description
 *   I2C initialization function, set the baudrate, device address, and configuration some 
 *   registers
 * \param[in]
 *   address - device address 
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void I2C_init(uint8_t address)
{
  // load address into TWI address register
  TWAR = address;

  TWCR = 0x00;   //Disable TWI
  TWBR = 0x02;   //Set the baudrate: CPU Clock/16+2(TWBR)
  TWSR|= 0x00;   //Set Divider factor

  //Set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
  TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
}

/**
 * \par Function
 *   ISR(TWI_vect)
 * \par Description
 *   This function is used to process data from i2c host
 * \param[in]
 *   None 
 * \par Output
 *   bufI2C[]
 * \return
 *   None
 * \par Others
 *   None
 */
ISR(TWI_vect)
{
  // temporary stores the received data
  uint8_t data;

  // own address has been acknowledged
  if( (TWSR & 0xF8) == TW_SR_SLA_ACK )
  {  
    rd=0;
    wr=0;
    // clear TWI interrupt flag, prepare to receive next byte and acknowledge
    TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT) | (1<<TWEA);
  }
  else if( (TWSR & 0xF8) == TW_SR_DATA_ACK )
  { 
    // data has been received in slave receiver mode
    // save the received byte inside data 
    data = TWDR;
    bufI2C[rd++] = data;
    TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT) | (1<<TWEA);
  }
  else if((TWSR & 0xF8) == TW_ST_SLA_ACK)
  {
    // the start of i2c read
    TWDR = bufI2C[wr++]; // todo: errata of avr, to insert delay between twdr and twcr?
    // clear TWI interrupt flag, prepare to send next byte and receive acknowledge
    TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
  }
  else if( (TWSR & 0xF8) == TW_ST_DATA_ACK )
  { 
    // device has been addressed to be a transmitter
    // copy the specified buffer address into the TWDR register for transmission
    TWDR = bufI2C[wr++];
    // clear TWI interrupt flag, prepare to send next byte and receive acknowledge
    TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
  }
  else if((TWSR & 0xF8) == TW_SR_STOP)
  {
    ParseI2cCmd(bufI2C);
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
  }
  else
  {
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT)|_BV(TWSTO);
    while(TWCR & _BV(TWSTO))
    {
      continue;
    }
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
  }
}

/**
 * \par Function
 *   ParseI2cCmd
 * \par Description
 *   This function is used to process command from i2c host
 * \param[in]
 *   (bufI2C[])byte 0:   motor index
 *   (bufI2C[])byte 1:   cmd
 *   (bufI2C[])byte 2-n: parameters
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void ParseI2cCmd(char * c)
{
  int rpm;
  long angle;
  int power;
  uint8_t cmd =  (c[1] & 0x3f);
  uint8_t slot = c[0];
  memcpy(&val,c+2,16);
#ifdef DEBUG_INFO
  Serial.print("cmd:");
  Serial.print(cmd);
#endif
  switch(cmd){
    // move state and function
    case CMD_RESET:
      resetMotor();
      break;
    case CMD_MOVE_TO:
#ifdef DEBUG_INFO
      Serial.print("  ,slot:");
      Serial.print(slot);
      Serial.print("  ,flag:");
      Serial.print((uint8_t)val.floatVal[0]);
      Serial.print("  ,pos:");
      Serial.print(val.longVal[1]);
      Serial.print("  speed:");
      Serial.println(val.floatVal[2]);
#endif
      moveTo(slot,(uint8_t)val.floatVal[0],val.longVal[1],val.floatVal[2]);
      break;
    case CMD_MOVE:
      move(slot,(uint8_t)val.floatVal[0],val.longVal[1],val.floatVal[2]);
      break;
    case CMD_MOVE_SPD:
      move_speed(slot,(uint8_t)val.floatVal[0],val.floatVal[1]);
      break;
    case CMD_SET_SPEED_PID:
      setSpeedPid(slot,val.floatVal[0],val.floatVal[1],val.floatVal[2]);
      break;
    case CMD_SET_POS_PID:
      setPosPid(slot,val.floatVal[0],val.floatVal[1],val.floatVal[2]);
      break;
    case CMD_SET_MODE:
      setMode(slot,val.byteVal[0]);
      break;
    case CMD_SET_PWM:
      setMode(slot,PWM_I2C_PWM);
      set_pwm(slot,val.intVal[0]);
      break;
    case CMD_SET_RATIO:
      setRatio(slot,val.floatVal[0]);
      break;
    case CMD_SET_PULSE:
      setPulse(slot,val.intVal[0]);
      break;
    case CMD_SET_DEVID:
      setDevid(val.byteVal[0]);
      break;
    case CMD_GET_SPEED_PID:
      if(slot == MOTOR_1)
      {
        memcpy(&bufI2C[0],&encoder_eeprom.speed1_p,4);
        memcpy(&bufI2C[4],&encoder_eeprom.speed1_i,4);
        memcpy(&bufI2C[8],&encoder_eeprom.speed1_d,4);
      }
      else
      {
        memcpy(&bufI2C[0],&encoder_eeprom.speed0_p,4);
        memcpy(&bufI2C[4],&encoder_eeprom.speed0_i,4);
        memcpy(&bufI2C[8],&encoder_eeprom.speed0_d,4);
      }  
      break;
    case CMD_GET_POS_PID:
      if(slot == MOTOR_1)
      {
        memcpy(&bufI2C[0],&encoder_eeprom.pos1_p,4);
        memcpy(&bufI2C[4],&encoder_eeprom.pos1_i,4);
        memcpy(&bufI2C[8],&encoder_eeprom.pos1_d,4);
      }
      else
      {
        memcpy(&bufI2C[0],&encoder_eeprom.pos0_p,4);
        memcpy(&bufI2C[4],&encoder_eeprom.pos0_i,4);
        memcpy(&bufI2C[8],&encoder_eeprom.pos0_d,4);
      }     
      break;
    case CMD_GET_SPEED:
      memcpy(&bufI2C[0],&encoder_data[slot].current_speed,4); 
      break;
    case CMD_GET_POS:
      memcpy(&bufI2C[0],&encoder_data[slot].pulse,4);
      break;
    case CMD_GET_RATIO:
      if(slot == MOTOR_1)
      {
        memcpy(&bufI2C[0],&encoder_eeprom.ratio1,4);
      }
      else
      {
        memcpy(&bufI2C[0],&encoder_eeprom.ratio0,4);
      } 
      break;
    case CMD_GET_PULSE:
      if(slot == MOTOR_1)
      {
        memcpy(&bufI2C[0],&encoder_eeprom.pulse1,4);
      }
      else
      {
        memcpy(&bufI2C[0],&encoder_eeprom.pulse0,4);
      } 
      break;
  }
}


/****************************************************************************************************
 * Serial process code
 * These codes used to start the serial command processing.
****************************************************************************************************/
void parseGcode(char * cmd)
{
  char * tmp;
  char * str;
  char g_code_cmd;
  str = strtok_r(cmd, " ", &tmp);
  g_code_cmd = str[0];
#ifdef DEBUG_INFO
  Serial.print("g_code_cmd: ");
  Serial.println(g_code_cmd);
#endif  
  if(g_code_cmd == '0')
  {
    resetMotor();
  }
  else if(g_code_cmd == '1')
  {
    int8_t slot = MOTOR_0;
    int8_t lock_state = LOCK_STATE;
    float speed_temp = 0;
    long pos_temp = 0;
    while(str != NULL)
    {
      str = strtok_r(0, " ", &tmp);
      if((str[0]=='D') || (str[0]=='d'))
      {
        slot = atoi(str+1);
      }
      else if((str[0]=='L') || (str[0]=='l'))
      {
        lock_state = atoi(str+1);
      }
       else if((str[0]=='P') || (str[0]=='p'))
      {
        pos_temp = atol(str+1);
      }
      else if((str[0]=='S') || (str[0]=='s'))
      {
        speed_temp = atof(str+1);
      }
    }
    moveTo(slot,lock_state,pos_temp,speed_temp);
  }
  else if(g_code_cmd == '2')
  {
    int8_t slot = MOTOR_0;
    int8_t lock_state = LOCK_STATE;
    float speed_temp = 0;
    long pos_temp = 0;
    while(str != NULL)
    {
      str = strtok_r(0, " ", &tmp);
      if((str[0]=='D') || (str[0]=='d'))
      {
        slot = atoi(str+1);
      }
      else if((str[0]=='L') || (str[0]=='l'))
      {
        lock_state = atoi(str+1);
      }
       else if((str[0]=='P') || (str[0]=='p'))
      {
        pos_temp = atol(str+1);
      }
      else if((str[0]=='S') || (str[0]=='s'))
      {
        speed_temp = atof(str+1);
      }
    }
    move(slot,lock_state,pos_temp,speed_temp);
  }
  else if(g_code_cmd == '3')
  {
    int8_t slot = MOTOR_0;
    int8_t lock_state = LOCK_STATE;
    float speed_temp = 0;
    while(str != NULL)
    {
      str = strtok_r(0, " ", &tmp);
      if((str[0]=='D') || (str[0]=='d'))
      {
        slot = atoi(str+1);
      }
      else if((str[0]=='L') || (str[0]=='l'))
      {
        lock_state = atoi(str+1);
      }
      else if((str[0]=='S') || (str[0]=='s'))
      {
        speed_temp = atof(str+1);
      }
    }
    move_speed(slot,lock_state,speed_temp);
  }
  else if(g_code_cmd == '4')
  {
    int8_t slot = MOTOR_0;
    int8_t mode = I2C_MODE;
    int16_t pwm = 0;
    while(str != NULL)
    {
      str = strtok_r(0, " ", &tmp);
      if((str[0]=='D') || (str[0]=='d'))
      {
        slot = atoi(str+1);
      }
      else if((str[0]=='M') || (str[0]=='m'))
      {
        mode = atoi(str+1);
      }
      else if((str[0]=='W') || (str[0]=='w'))
      {
        pwm = atoi(str+1);
      }
    }
    setMode(slot,mode);
    set_pwm(slot,pwm);
  }
  else if(g_code_cmd == '5')
  {
    int8_t slot = MOTOR_0;
    while(str != NULL)
    {
      str = strtok_r(0, " ", &tmp);
      if((str[0]=='D') || (str[0]=='d'))
      {
        slot = atoi(str+1);
      }
    }
  }
  else if(g_code_cmd == '6')
  {
    int8_t dev_id = encoder_eeprom.devid;
    while(str != NULL)
    {
      str = strtok_r(0, " ", &tmp);
      if((str[0]=='D') || (str[0]=='d'))
      {
        dev_id = atoi(str+1);
      }
    }
    setDevid(dev_id);
    Serial.print("G6: ");
    Serial.println(dev_id);
  }
}

void ParseSerialCmd(char * cmd)
{
  if((cmd[0]=='g') || (cmd[0]=='G'))
  { 
    // gcode
    parseGcode(cmd+1);
  }  
}

/****************************************************************************************************
 * Parameters configuration
****************************************************************************************************/
void setDefaultvalue(void)
{
  encoder_eeprom.start_data = EEPROM_PID_START;
  encoder_eeprom.devid = DEFALUT_I2C_ADDR;

  encoder_eeprom.speed0_p = 0.18;
  encoder_eeprom.speed0_i = 0.64;
  encoder_eeprom.speed0_d = 0.0;
  encoder_eeprom.pos0_p = 1.8;
  encoder_eeprom.pos0_i = 0.0;
  encoder_eeprom.pos0_d = 1.2;
  encoder_eeprom.ratio0 = 26.9;
  encoder_eeprom.pulse0 = 7;

  encoder_eeprom.mid_data = EEPROM_PID_MID;
  
  encoder_eeprom.speed1_p = 0.18;
  encoder_eeprom.speed1_i = 0.64;
  encoder_eeprom.speed1_d = 0.0;
  encoder_eeprom.pos1_p = 1.8;
  encoder_eeprom.pos1_i = 0.0;
  encoder_eeprom.pos1_d = 1.2;
  encoder_eeprom.ratio1 = 26.9;
  encoder_eeprom.pulse1 = 7;
  encoder_eeprom.end_data = EEPROM_PID_END;
}

void FloatToEEPROM(int address,int num,float *f)
{
  int i,j,bakupaddress;
  bakupaddress = address +  EEPROM.length()/2;
  for(i=0;i<2;i++)
  {
    for(j=0;j<num;j++)
    {
      EEPROM.put(address, f[j]);  
      address = address + sizeof(float);
    }
    address = bakupaddress;
  }
}

void IntToEEPROM(int address,int num, uint16_t *data)
{
  int i,j,bakupaddress;
  bakupaddress = address +  EEPROM.length()/2;
  for(i=0;i<2;i++)
  {
    for(j=0;j<num;j++)
    {
      EEPROM.put(address, data[j]);  
      address = address + sizeof(uint16_t);
    }
    address = bakupaddress;
  }
}

void ByteToEEPROM(int address,int num, uint8_t *data)
{
  int i,j,bakupaddress;
  bakupaddress = address +  EEPROM.length()/2;
  for(i=0;i<2;i++)
  {
    for(j=0;j<num;j++)
    {
      EEPROM.put(address, data[j]);  
      address = address + sizeof(uint8_t);
    }
    address = bakupaddress;
  }
}

void writetoEEPROM(void)
{
  int16_t i,address, length;
  length = EEPROM.length();
  for(i = 0; i < 2; i++)
  {
    EEPROM.write(EEPROM_START_POS, EEPROM_IF_HAVEPID_CHECK1);
    EEPROM.write(EEPROM_START_POS + 1, EEPROM_IF_HAVEPID_CHECK2);
    EEPROM.put(STORE_START_ADDR, encoder_eeprom);
    address = length/2;
  } 
}

void updatefromBackuparea(void)
{
  int16_t address, length;
  length = EEPROM.length();
  address = length/2;
  if((EEPROM.read(address) == EEPROM_IF_HAVEPID_CHECK1) & (EEPROM.read(address + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    address = length/2 + 2;
    EEPROM.get(address, encoder_eeprom);
    if((encoder_eeprom.start_data == EEPROM_PID_START) & (encoder_eeprom.mid_data == EEPROM_PID_MID) & (encoder_eeprom.end_data == EEPROM_PID_END))
    {
      EEPROM.write(EEPROM_START_POS, EEPROM_IF_HAVEPID_CHECK1);
      EEPROM.write(EEPROM_START_POS+1, EEPROM_IF_HAVEPID_CHECK2);
      EEPROM.put(STORE_START_ADDR, encoder_eeprom);
    }
    else
    {
      setDefaultvalue();  
      writetoEEPROM(); 
    }
  }
}

boolean readEEPROM(void)
{
  int16_t length = EEPROM.length();
  Serial.println( "Read data from EEPROM " );
  if((EEPROM.read(EEPROM_START_POS) == EEPROM_IF_HAVEPID_CHECK1) & (EEPROM.read(EEPROM_START_POS + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    EEPROM.get(STORE_START_ADDR, encoder_eeprom);
    if((encoder_eeprom.start_data == EEPROM_PID_START) & (encoder_eeprom.mid_data == EEPROM_PID_MID) & (encoder_eeprom.end_data == EEPROM_PID_END))
    {
      return true;
    }
    else
    {
      Serial.println( "updatefromBackuparea" );
      updatefromBackuparea();  
      return true;  
    }
  }
  else if((EEPROM.read(length/2) == EEPROM_IF_HAVEPID_CHECK1) & (EEPROM.read(length/2 + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    Serial.println( "updatefromBackuparea 2" );
    updatefromBackuparea(); 
    return true;
  }
  else
  {
    setDefaultvalue();
    writetoEEPROM();
    return false;
  }
}

/****************************************************************************************************
 * Motor Control Functions
****************************************************************************************************/
ISR(MOTOR_1_IRQ)
{
  if(MOTOR_1_PIN & (1 << MOTOR_1_DIR))
  {
    encoder_data[MOTOR_0].pulse++;
  }
  else
  {
    encoder_data[MOTOR_0].pulse--;
  }
}

ISR(MOTOR_2_IRQ)
{
  if(MOTOR_2_PIN & (1 << MOTOR_2_DIR))
  {
    encoder_data[MOTOR_1].pulse++;
  }
  else
  {
    encoder_data[MOTOR_1].pulse--;
  }
}

void setMotorPwm(uint8_t slot, int16_t pwm)
{
//  Serial.print("mode:");
//  Serial.print(encoder_data[slot].mode);
//  Serial.print(" ,slot:");
//  Serial.print(slot);
//  Serial.print(" ,pwm:");
//  Serial.println(pwm);
  if(slot == MOTOR_1)
  {
    setMotor2Pwm(pwm);
  }
  else
  {
    setMotor1Pwm(pwm);
  }
}

void setMotor1Pwm(int16_t pwm)
{ 
  pwm = constrain(pwm,-250,250);
  if(pwm < 0)
  {
    digitalWrite(MOTOR_1_H1, HIGH);
    analogWrite(MOTOR_1_PWM, abs(pwm));  
  }
  else
  {
    digitalWrite(MOTOR_1_H1, LOW);
    analogWrite(MOTOR_1_PWM, abs(pwm));
  }
}

void setMotor2Pwm(int16_t pwm)
{
  pwm = constrain(pwm,-250,250);
  if(pwm < 0)
  {
    digitalWrite(MOTOR_2_H1, HIGH);
    analogWrite(MOTOR_2_PWM, abs(pwm));
  }
  else
  {
    digitalWrite(MOTOR_2_H1, LOW);
    analogWrite(MOTOR_2_PWM, abs(pwm));
  }
}

/****************************************************************************************************
 * processing function
****************************************************************************************************/
void updateSpeed(void)
{
  if((millis() - measurement_speed_time) > 20)
  {
    uint16_t dt = millis() - measurement_speed_time;
    long cur_pos_0 = encoder_data[MOTOR_0].pulse;
    long cur_pos_1 = encoder_data[MOTOR_1].pulse;
    encoder_data[MOTOR_0].current_speed = ((cur_pos_0 - last_pulse_pos_encoder0)/(encoder_data[MOTOR_0].encoder_pulse * encoder_data[MOTOR_0].ratio))*(1000/dt)*60;
    encoder_data[MOTOR_1].current_speed = ((cur_pos_1 - last_pulse_pos_encoder1)/(encoder_data[MOTOR_1].encoder_pulse * encoder_data[MOTOR_1].ratio))*(1000/dt)*60;
    last_pulse_pos_encoder0 = cur_pos_0;
    last_pulse_pos_encoder1 = cur_pos_1;
    measurement_speed_time = millis();
  }
}

void encoder_move(uint8_t slot)
{
  if(millis() - encoder_move_time > 40)
  {
    int16_t pwm_encoder = 0;
    encoder_move_time = millis();
    if((encoder_data[slot].motion_state == MOTION_WITH_POS_LOCK) ||
       (encoder_data[slot].motion_state == MOTION_WITH_POS_RELEASE))
    {
      pwm_encoder = pid_position_to_pwm(slot);
    }
    else if((encoder_data[slot].motion_state == MOTION_WITHOUT_POS_LOCK) ||
            (encoder_data[slot].motion_state == MOTION_WITHOUT_POS_RELEASE))
    {
      pwm_encoder = speed_without_pos(slot);
    }
    encoder_data[slot].cur_pwm = pwm_encoder;
  }
}

void set_pwm(uint8_t slot,int pwm)
{
  encoder_data[slot].tar_pwm = pwm;
}

void pwm_move(uint8_t slot)
{
  if(millis() - encoder_move_time > 40)
  {
    encoder_move_time = millis();
    encoder_data[slot].cur_pwm = 0.9 * encoder_data[slot].cur_pwm + 0.1 * encoder_data[slot].tar_pwm;
  }
}

void updateCurPos(void)
{
  encoder_data[MOTOR_0].current_pos = (long)((encoder_data[MOTOR_0].pulse/(encoder_data[MOTOR_0].encoder_pulse * encoder_data[MOTOR_0].ratio)) * 360);
  encoder_data[MOTOR_1].current_pos = (long)((encoder_data[MOTOR_1].pulse/(encoder_data[MOTOR_1].encoder_pulse * encoder_data[MOTOR_1].ratio)) * 360);
}

long encoder_distance_togo(uint8_t slot)
{
  return encoder_data[slot].target_pos - encoder_data[slot].current_pos;
}

int16_t pid_position_to_pwm(uint8_t slot)
{
  float cur_pos = 0;
  float seek_speed = 0;
  float seek_temp = 0;
  float pos_error = 0;
  float speed_error = 0;
  float d_component = 0;
  float out_put_offset = 0;

  pos_error = encoder_distance_togo(slot);
  if((lock_flag[slot] == false) && (dir_lock_flag[slot] == true) && (pos_error < 0))
  {
    d_component = encoder_data[slot].current_speed;
    out_put_offset = encoder_data[slot].PID_pos.D * d_component;
    encoder_data[slot].PID_pos.Output = -out_put_offset;
    encoder_output[slot] = encoder_data[slot].PID_pos.Output;
    lock_flag[slot] = true;
    encoder_data[slot].cur_pwm = encoder_output[slot];
    return encoder_output[slot];
  }
  else if((lock_flag[slot] == false) && (dir_lock_flag[slot] == false) && (pos_error > 0))
  {
    d_component = encoder_data[slot].current_speed;
    out_put_offset = encoder_data[slot].PID_pos.D * d_component;
    encoder_data[slot].PID_pos.Output = -out_put_offset;
    encoder_output[slot] = encoder_data[slot].PID_pos.Output;
    lock_flag[slot] = true;
    encoder_data[slot].cur_pwm = encoder_output[slot];
    return encoder_output[slot];
  }
      
  //speed pid;
  if((lock_flag[slot] == false) && (abs(pos_error) >= encoder_data[slot].target_speed * DECELERATION_DISTANCE_PITCH))
  {
    speed_error = encoder_data[slot].current_speed - encoder_data[slot].target_speed * (pos_error/abs(pos_error));
    out_put_offset = encoder_data[slot].PID_speed.P * speed_error;
    out_put_offset = constrain(out_put_offset,-25,25);
    encoder_data[slot].PID_speed.Output = encoder_output[slot];
    encoder_data[slot].PID_speed.Output -= out_put_offset;  
    encoder_data[slot].PID_speed.Output = constrain(encoder_data[slot].PID_speed.Output,-255,255);
    encoder_output[slot] = encoder_data[slot].PID_speed.Output;
  }

  //position pid;
  else
  {
    if((lock_flag[slot] == false) && (abs(pos_error) > ENCODER_POS_DEADBAND))
    {
      seek_speed = sqrt(abs(encoder_data[slot].target_speed * DECELERATION_DISTANCE_PITCH * (pos_error-ENCODER_POS_DEADBAND)))/DECELERATION_DISTANCE_PITCH;
      d_component = encoder_data[slot].current_speed - seek_speed * (pos_error/abs(pos_error));
      out_put_offset = encoder_data[slot].PID_pos.D * d_component;
      out_put_offset = constrain(out_put_offset,-20,20);

      encoder_data[slot].PID_pos.Output = encoder_output[slot];
      encoder_data[slot].PID_pos.Output -= out_put_offset;
      if(pos_error >= 0)
      {
        encoder_data[slot].PID_pos.Output = constrain(encoder_data[slot].PID_pos.Output,PWM_MIN_OFFSET,255);
      }
      else
      {
        encoder_data[slot].PID_pos.Output = constrain(encoder_data[slot].PID_pos.Output,-255,-PWM_MIN_OFFSET);
      }
      encoder_output[slot] = encoder_data[slot].PID_pos.Output;
    }
    else
    {
      if(encoder_data[slot].motion_state == MOTION_WITH_POS_LOCK)
      {
        d_component = encoder_data[slot].current_speed;
        out_put_offset = encoder_data[slot].PID_pos.D * d_component;
        out_put_offset = constrain(out_put_offset,-20,20);
        encoder_data[slot].PID_pos.Output = pos_error * encoder_data[slot].PID_pos.P;
        encoder_data[slot].PID_pos.Output -= out_put_offset;
        encoder_data[slot].PID_pos.Output = constrain(encoder_data[slot].PID_pos.Output,-255,255);
        encoder_output[slot] = encoder_data[slot].PID_pos.Output;
      }
      else
      {
        encoder_output[slot] = 0;
      }
    }
  }
#ifdef DEBUG_INFO
  Serial.print("tar1:");
  Serial.print(encoder_data[slot].target_pos);
  Serial.print(" ,current_speed:");
  Serial.print(encoder_data[slot].current_speed);
  Serial.print(" ,target_speed:");
  Serial.print(encoder_data[slot].target_speed);
  Serial.print(" ,cur1:");
  Serial.print(encoder_data[slot].current_pos);
  Serial.print(" ,pos_error1:");
  Serial.print(pos_error);
  Serial.print(" ,d_component1:");
  Serial.print(d_component);
  Serial.print(" ,motion_state1:");
  Serial.print(encoder_data[slot].motion_state);
  Serial.print(" ,out1:");
  Serial.println(encoder_output[slot]);
#endif
  return encoder_output[slot];
}

int16_t speed_without_pos(uint8_t slot)
{
  float speed_error;
  float out_put_offset;
  speed_error = encoder_data[slot].current_speed - encoder_data[slot].target_speed;
  out_put_offset = encoder_data[slot].PID_speed.P * speed_error;

  out_put_offset = constrain(out_put_offset,-25,25);
  encoder_data[slot].PID_speed.Output = encoder_output[slot];
  encoder_data[slot].PID_speed.Output -= out_put_offset;

  if((lock_flag[slot] == true) && (encoder_data[slot].motion_state == MOTION_WITHOUT_POS_LOCK))
  {
    encoder_data[slot].PID_speed.Integral += speed_error;
    out_put_offset = encoder_data[slot].PID_speed.I * encoder_data[slot].PID_speed.Integral;
    encoder_data[slot].PID_speed.Output = -out_put_offset;
  }
  else if((lock_flag[slot] == true) && (encoder_data[slot].motion_state == MOTION_WITHOUT_POS_RELEASE))
  {
    encoder_data[slot].PID_speed.Output = 0;
  }

  if((lock_flag[slot] == false) && (encoder_data[slot].target_speed == 0) && (abs(out_put_offset) < 15))
  {
    encoder_data[slot].PID_speed.Output = 0;
    lock_flag[slot] = true;
  }

  encoder_data[slot].PID_speed.Output = constrain(encoder_data[slot].PID_speed.Output,-255,255);
  encoder_output[slot] = encoder_data[slot].PID_speed.Output;
#ifdef DEBUG_INFO
  Serial.print("Mode:");
  Serial.print(encoder_data[slot].mode);
  Serial.print("slot:");
  Serial.print(slot);
  Serial.print(" ,tar:");
  Serial.print(encoder_data[slot].target_speed);
  Serial.print(" ,cur:");
  Serial.print(encoder_data[slot].current_speed);
  Serial.print(" ,speed_error:");
  Serial.print(speed_error);
  Serial.print(" ,out:");
  Serial.println(encoder_output[slot]);
#endif
  return encoder_output[slot];
}

void initMotor()
{
  memset(&encoder_data[MOTOR_0],0,sizeof(encoder_data_type));
  memset(&encoder_data[MOTOR_1],0,sizeof(encoder_data_type));
  dev_id = encoder_eeprom.devid;
  encoder_data[MOTOR_0].slot = 0;
  encoder_data[MOTOR_0].mode = I2C_MODE;
  encoder_data[MOTOR_0].motion_state = 0;
  encoder_data[MOTOR_0].tar_pwm = 0;
  encoder_data[MOTOR_0].cur_pwm = 0;
  encoder_data[MOTOR_0].pulse = 0;
  encoder_data[MOTOR_0].current_speed = 0;
  encoder_data[MOTOR_0].target_speed = 0;
  encoder_data[MOTOR_0].current_pos = 0;
  encoder_data[MOTOR_0].previous_pos = 0;
  encoder_data[MOTOR_0].target_pos = 0;
  encoder_data[MOTOR_0].PID_speed.P = encoder_eeprom.speed0_p;
  encoder_data[MOTOR_0].PID_speed.I = encoder_eeprom.speed0_i;
  encoder_data[MOTOR_0].PID_speed.D = encoder_eeprom.speed0_d;
  encoder_data[MOTOR_0].PID_pos.P = encoder_eeprom.pos0_p;
  encoder_data[MOTOR_0].PID_pos.I = encoder_eeprom.pos0_i;
  encoder_data[MOTOR_0].PID_pos.D = encoder_eeprom.pos0_d;
//  encoder_data[MOTOR_0].PID_speed.P = 0.18;
//  encoder_data[MOTOR_0].PID_speed.I = 0.64;
//  encoder_data[MOTOR_0].PID_speed.D = 0;
//  encoder_data[MOTOR_0].PID_pos.P = 1.8;
//  encoder_data[MOTOR_0].PID_pos.I = 0;
//  encoder_data[MOTOR_0].PID_pos.D = 1.2;
  encoder_data[MOTOR_0].ratio = encoder_eeprom.ratio0;
  encoder_data[MOTOR_0].encoder_pulse = encoder_eeprom.pulse0;

  encoder_data[MOTOR_1].slot = 1;
  encoder_data[MOTOR_1].mode = I2C_MODE;
  encoder_data[MOTOR_1].motion_state = 0;
  encoder_data[MOTOR_1].tar_pwm = 0;
  encoder_data[MOTOR_0].cur_pwm = 0;
  encoder_data[MOTOR_1].pulse = 0;
  encoder_data[MOTOR_1].current_speed = 0;
  encoder_data[MOTOR_1].target_speed = 0;
  encoder_data[MOTOR_1].current_pos = 0;
  encoder_data[MOTOR_1].previous_pos = 0;
  encoder_data[MOTOR_1].target_pos = 0;
  encoder_data[MOTOR_1].PID_speed.P = encoder_eeprom.speed1_p;
  encoder_data[MOTOR_1].PID_speed.I = encoder_eeprom.speed1_i;
  encoder_data[MOTOR_1].PID_speed.D = encoder_eeprom.speed1_d;
  encoder_data[MOTOR_1].PID_pos.P = encoder_eeprom.pos1_p;
  encoder_data[MOTOR_1].PID_pos.I = encoder_eeprom.pos1_i;
  encoder_data[MOTOR_1].PID_pos.D = encoder_eeprom.pos1_d;
//  encoder_data[MOTOR_1].PID_speed.P = 0.18;
//  encoder_data[MOTOR_1].PID_speed.I = 0.64;
//  encoder_data[MOTOR_1].PID_speed.D = 0;
//  encoder_data[MOTOR_1].PID_pos.P = 1.8;
//  encoder_data[MOTOR_1].PID_pos.I = 0;
//  encoder_data[MOTOR_1].PID_pos.D = 1.2;
  encoder_data[MOTOR_1].ratio = encoder_eeprom.ratio1;
  encoder_data[MOTOR_1].encoder_pulse = encoder_eeprom.pulse1;
}

void resetMotor(void)
{
  setMotor1Pwm(0);
  setMotor2Pwm(0);
  delay(10);  
  readEEPROM();
  delay(10);
  initMotor();
  I2C_init((dev_id) << 1);
  encoder_move_time = measurement_speed_time = millis();
}

void moveTo(uint8_t slot,uint8_t state,long turns,float speed)
{
  encoder_data[slot].mode = I2C_MODE;
  lock_flag[slot] = false;
  if(LOCK_STATE == state)
  {
    encoder_data[slot].motion_state = MOTION_WITH_POS_LOCK;
  }
  else
  {
    encoder_data[slot].motion_state = MOTION_WITH_POS_RELEASE;
  }

  encoder_data[slot].target_pos = turns;
  float speed_temp = constrain(speed,MIN_ENCODER_SPEED,MAX_ENCODER_SPEED);
  encoder_data[slot].target_speed = abs(speed_temp);
  if(encoder_distance_togo(slot) > 0)
  {
    dir_lock_flag[slot] = true;
  }
  else
  {
    dir_lock_flag[slot] = false;
  }
}

void move(uint8_t slot,uint8_t state,long turns,float speed)
{
  moveTo(slot,state,(encoder_data[slot].current_pos + turns),speed);
}

void move_speed(uint8_t slot,uint8_t state,float speed)
{
  encoder_data[slot].mode = I2C_MODE;
  lock_flag[slot] = false;
  if(LOCK_STATE == state)
  {
    encoder_data[slot].motion_state = MOTION_WITHOUT_POS_LOCK;
  }
  else
  {
    encoder_data[slot].motion_state = MOTION_WITHOUT_POS_RELEASE;
  }
  float speed_temp = constrain(speed,MIN_ENCODER_SPEED,MAX_ENCODER_SPEED);
  encoder_data[slot].target_speed = speed_temp;
}

void setMode(uint8_t slot,uint8_t mode)
{
  if((mode == I2C_MODE) || (mode == PWM_I2C_PWM))
  {
    encoder_data[slot].mode = mode;
  }
  else
  {
    encoder_data[slot].mode = PWM_MODE; 
  }
}

void setSpeedPid(uint8_t slot, float p,float i,float d)
{
  float f[3] = {p,i,d};
  if(slot == MOTOR_1)
  {
    encoder_eeprom.speed1_p = p;
    encoder_eeprom.speed1_i = i;
    encoder_eeprom.speed1_d = d;
    FloatToEEPROM(STORE_PID1_ADDR,3,f);
  }
  else
  {
    encoder_eeprom.speed0_p = p;
    encoder_eeprom.speed0_i = i;
    encoder_eeprom.speed0_d = d;
    FloatToEEPROM(STORE_PID0_ADDR,3,f);
  }
}

void setPosPid(uint8_t slot, float p,float i,float d)
{
  float f[3] = {p,i,d};
  if(slot == MOTOR_1)
  {
    encoder_eeprom.pos1_p = p;
    encoder_eeprom.pos1_i = i;
    encoder_eeprom.pos1_d = d;
    FloatToEEPROM(STORE_PID1_ADDR+12,3,f);
  }
  else
  {
    encoder_eeprom.pos0_p = p;
    encoder_eeprom.pos0_i = i;
    encoder_eeprom.pos0_d = d;
    FloatToEEPROM(STORE_PID0_ADDR+12,3,f);
  }
}

void setRatio(uint8_t slot, float ratio)
{
  if(slot == MOTOR_1)
  {
    encoder_eeprom.ratio1 = ratio;
    FloatToEEPROM(STORE_RATIO1_ADDR,1,&ratio);
  }
  else
  {
    encoder_eeprom.ratio0 = ratio;
    FloatToEEPROM(STORE_RATIO0_ADDR,1,&ratio);
  }
}

void setPulse(uint8_t slot, uint16_t pulse)
{
  if(slot == MOTOR_1)
  {
    encoder_eeprom.pulse1 = pulse;
    IntToEEPROM(STORE_PLUS1_ADDR,1,&pulse);
  }
  else
  {
    encoder_eeprom.pulse0 = pulse;
    IntToEEPROM(STORE_PLUS0_ADDR,1,&pulse);
  }
}

void setDevid(uint8_t devid)
{
  encoder_eeprom.devid = devid;
  ByteToEEPROM(STORE_DEVID_ADDR,1,&devid);
}

void pwm_frequency_init(void)
{
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);
}
/****************************************************************************************************
 * Arduino main function
****************************************************************************************************/
void setup() {
  delay(10);
  Serial.begin(115200);
  pwm_frequency_init();
  pinMode(INT_1_PIN, INPUT_PULLUP);
  pinMode(DIR_1_PIN, INPUT_PULLUP);
  pinMode(INT_2_PIN, INPUT_PULLUP);
  pinMode(DIR_2_PIN, INPUT_PULLUP);
  pinMode(MOTOR_1_H1,OUTPUT);
  pinMode(MOTOR_2_H1,OUTPUT);

  EICRA = (1 << ISC11) | (1 << ISC01);
  EIMSK = (1 << INT0) | (1 << INT1);

  delay(10);  
  readEEPROM();
  delay(10);
  initMotor();
  I2C_init((dev_id) << 1);
}

char Uart_Buf[64];
char bufindex;
void loop() 
{
  double pwm1_read_temp=0; 
  double pwm2_read_temp=0;
  //G code processing
  while(Serial.available() > 0)
  {
    char c = Serial.read();
    Serial.write(c);
    Uart_Buf[bufindex++]=c;
    if((c=='\n') || (c=='#'))
    {
      ParseSerialCmd(Uart_Buf);
      memset(Uart_Buf,0,64);
      bufindex = 0;
    }
  }
  updateCurPos();
  updateSpeed();

  if(encoder_data[MOTOR_0].mode == PWM_MODE)
  {
    for(int i=0;i<20;i++)
    { 
      pwm1_read_temp = pwm1_read_temp + analogRead(A1);  //Read PWM values
    }
    encoder_data[MOTOR_0].tar_pwm = (pwm1_read_temp/20-512)*255/512;
  }

  if(encoder_data[MOTOR_1].mode == PWM_MODE)
  {
    for(int i=0;i<20;i++)
    { 
      pwm1_read_temp = pwm1_read_temp + analogRead(A0);  //Read PWM values
    }
    encoder_data[MOTOR_1].tar_pwm = (pwm1_read_temp/20-512)*255/512;
  }

  //encoder move
  if(encoder_data[MOTOR_0].mode == I2C_MODE)
  {
    encoder_move(MOTOR_0);
    setMotorPwm(MOTOR_0,encoder_data[MOTOR_0].cur_pwm);
  }
  else
  {
    pwm_move(MOTOR_0);
    setMotorPwm(MOTOR_0,encoder_data[MOTOR_0].cur_pwm);
  }

  if(encoder_data[MOTOR_1].mode == I2C_MODE)
  {
    encoder_move(MOTOR_1);
    setMotorPwm(MOTOR_1,encoder_data[MOTOR_1].cur_pwm);
  }
  else
  {
    pwm_move(MOTOR_1);
    setMotorPwm(MOTOR_1,encoder_data[MOTOR_1].cur_pwm);
  }
}


