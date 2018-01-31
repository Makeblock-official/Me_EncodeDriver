#include <Arduino.h>
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include "encodedMotor.h"


//Print message priority
// #define DEBUG_INFO_ERROR
// #define DEBUG_INFO_HIGH
// #define DEBUG_INFO_MID
// #define DEBUG_INFO_LOW
// #define DEBUG_INFO

//Common parameter configuration
#define DEFALUT_I2C_ADDR    0x11
#define PULSE_PER_C         8
#define MOTOR_BOTH          2

#define INT_1_PIN     2
#define DIR_1_PIN     7
#define MOTOR_1_PWM_A 9
#define MOTOR_1_PWM_B 10

#define INT_2_PIN     3
#define DIR_2_PIN     8
#define MOTOR_2_PWM_A 6
#define MOTOR_2_PWM_B 5

#define SET_I2C_ADDR_PIN  A7
#define WORK_LED_PIN      4

#define MOTOR_DRV_ERR_FB_PIN  A0
#define MOTOR_DRV_SHUTDOWN_CTRL_PIN A2
#define MOTOR_DRV_STBYB_PIN   A1

//Interrupt Configuration
#define MOTOR_1_IRQ         INT0_vect
#define MOTOR_2_IRQ         INT1_vect
#define MOTOR_2_PIN         PINB
#define MOTOR_2_DIR         PINB0
#define MOTOR_1_PIN         PIND
#define MOTOR_1_DIR         PIND7


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

static char i2c_buffer[32];
static int16_t i2c_rd,i2c_wr;
static char uart_buffer[64];
static char uart_index;
static uint8_t devic_id;
static boolean lock_flag[2];
static int16_t encoder_output[2];
static long motor_ctrl_time;
static encoder_data_type encoder_data[2];
static encoder_eeprom_type encoder_eeprom;
const uint8_t pwm_input_pin[2] = {A1,A0};

void MotorDrvErrPcs(void)
{
  static uint8_t error_flag = 0;

  if(digitalRead(MOTOR_DRV_ERR_FB_PIN) == LOW)
  {
    error_flag = 1;
    //Serial.print("Sys error...");
  }
  else
  {
    //Serial.print("  Sys ok ...");
  }

  if(error_flag == 1)
  {
    static uint8_t count = 0;
    count++;
    if(count == 1)
    {
      digitalWrite(WORK_LED_PIN, HIGH);
    }
    else if(count == 10)
    {
      digitalWrite(WORK_LED_PIN, LOW);
    }
    else if(count > 40)
    {
      count = 0;;
    }
  }
  else
  {
    digitalWrite(WORK_LED_PIN, HIGH);
  }
}

void eeprom_default_value(void)
{
  encoder_eeprom.start_data = EEPROM_PID_START;
  encoder_eeprom.devid = DEFALUT_I2C_ADDR;

  encoder_eeprom.speed0_p = 0.5;
  encoder_eeprom.speed0_i = 0.1;
  encoder_eeprom.speed0_d = 0.0;
  encoder_eeprom.pos0_p = 1.0;
  encoder_eeprom.pos0_i = 0;
  encoder_eeprom.pos0_d = 1.2;
  encoder_eeprom.ratio0 = 39.43;
  encoder_eeprom.pulse0 = 9;

  encoder_eeprom.mid_data = EEPROM_PID_MID;

  encoder_eeprom.speed1_p = 0.5;
  encoder_eeprom.speed1_i = 0.1;
  encoder_eeprom.speed1_d = 0.0;
  encoder_eeprom.pos1_p = 1.0;
  encoder_eeprom.pos1_i = 0;
  encoder_eeprom.pos1_d = 1.2;
  encoder_eeprom.ratio1 = 39.43;
  encoder_eeprom.pulse1 = 9;
  encoder_eeprom.end_data = EEPROM_PID_END;
}

void eeprom_set_float(int address,int num,float *f)
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

void eeprom_set_int(int address,int num, uint16_t *data)
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

void eeprom_set_byte(int address,int num, uint8_t *data)
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

void eeprom_write(void)
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

void eeprom_updatefromBackup(void)
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
      eeprom_default_value();  
      eeprom_write(); 
    }
  }
}

boolean eeprom_read(void)
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
      Serial.println( "eeprom_updatefromBackup" );
      eeprom_updatefromBackup();  
      return true;  
    }
  }
  else if((EEPROM.read(length/2) == EEPROM_IF_HAVEPID_CHECK1) & (EEPROM.read(length/2 + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    Serial.println( "eeprom_updatefromBackup 2" );
    eeprom_updatefromBackup(); 
    return true;
  }
  else
  {
    eeprom_default_value();
    eeprom_write();
    return false;
  }
}

/**
 * \par Function
 *   i2c_init
 * \par Description
 *   I2C initialization function, set the baudrate, device address, and configuration some 
 *   registers
 * \param[in]
 *   address - device address 
 * \par output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void i2c_init(uint8_t address)
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
 * \par output
 *   i2c_buffer[]
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
    i2c_rd=0;
    i2c_wr=0;
    // clear TWI interrupt flag, prepare to receive next byte and acknowledge
    TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT) | (1<<TWEA);
  }
  else if( (TWSR & 0xF8) == TW_SR_DATA_ACK )
  { 
    // data has been received in slave receiver mode
    // save the received byte inside data 
    data = TWDR;
    i2c_buffer[i2c_rd++] = data;
    TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT) | (1<<TWEA);
  }
  else if((TWSR & 0xF8) == TW_ST_SLA_ACK)
  {
    // the start of i2c read
    TWDR = i2c_buffer[i2c_wr++]; // todo: errata of avr, to insert delay between twdr and twcr?
    // clear TWI interrupt flag, prepare to send next byte and receive acknowledge
    TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
  }
  else if( (TWSR & 0xF8) == TW_ST_DATA_ACK )
  { 
    // device has been addressed to be a transmitter
    // copy the specified buffer address into the TWDR register for transmission
    TWDR = i2c_buffer[i2c_wr++];
    // clear TWI interrupt flag, prepare to send next byte and receive acknowledge
    TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
  }
  else if((TWSR & 0xF8) == TW_SR_STOP)
  {
    parse_i2c_cmd(i2c_buffer);
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
 *   parse_i2c_cmd
 * \par Description
 *   This function is used to process command from i2c host
 * \param[in]
 *   (i2c_buffer[])byte 0:   motor index
 *   (i2c_buffer[])byte 1:   cmd
 *   (i2c_buffer[])byte 2-n: parameters
 * \par output
 *   None
 * \return
 *   None
 * \par Others
 *   None
 */
void parse_i2c_cmd(char * c)
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

  Serial.print("  ,slot:");
  Serial.print(slot);
  Serial.print("  ,flag:");
  Serial.print((uint8_t)val.floatVal[0]);
  Serial.print("  ,pos:");
  Serial.print(val.longVal[1]);
  Serial.print("  speed:");
  Serial.println(val.floatVal[2]);
#endif
  switch(cmd){
    // move state and function
    case CMD_RESET:
      motor_reset();
      break;
    case CMD_MOVE_TO:
      set_position_to(slot,(uint8_t)val.floatVal[0],val.longVal[1],val.floatVal[2]);
      break;
    case CMD_MOVE:
      set_position(slot,(uint8_t)val.floatVal[0],val.longVal[1],val.floatVal[2]);
      break;
    case CMD_MOVE_SPD:
      set_speed(slot,(uint8_t)val.floatVal[0],val.floatVal[1]);
      break;
    case CMD_SET_SPEED_PID:
      set_speed_pid(slot,val.floatVal[0],val.floatVal[1],val.floatVal[2]);
      break;
    case CMD_SET_POS_PID:
      set_position_pid(slot,val.floatVal[0],val.floatVal[1],val.floatVal[2]);
      break;
    case CMD_SET_MODE:
      set_mode(slot,val.byteVal[0]);
      break;
    case CMD_SET_PWM:
      set_mode(slot,MOTION_PWM_I2C_PWM);
      set_pwm(slot,val.intVal[0]);
      break;
    case CMD_SET_RATIO:
      set_ratio(slot,val.floatVal[0]);
      break;
    case CMD_SET_CUR_POS:
      set_curpositon(slot,val.longVal[0]);
      break;
    case CMD_SET_PULSE:
      set_pulse(slot,val.intVal[0]);
      break;
    case CMD_SET_DEVID:
      set_devicid(val.byteVal[0]);
      break;
    case CMD_GET_SPEED_PID:
      if(slot == MOTOR_1)
      {
        memcpy(&i2c_buffer[0],&encoder_eeprom.speed1_p,4);
        memcpy(&i2c_buffer[4],&encoder_eeprom.speed1_i,4);
        memcpy(&i2c_buffer[8],&encoder_eeprom.speed1_d,4);
      }
      else
      {
        memcpy(&i2c_buffer[0],&encoder_eeprom.speed0_p,4);
        memcpy(&i2c_buffer[4],&encoder_eeprom.speed0_i,4);
        memcpy(&i2c_buffer[8],&encoder_eeprom.speed0_d,4);
      }  
      break;
    case CMD_GET_POS_PID:
      if(slot == MOTOR_1)
      {
        memcpy(&i2c_buffer[0],&encoder_eeprom.pos1_p,4);
        memcpy(&i2c_buffer[4],&encoder_eeprom.pos1_i,4);
        memcpy(&i2c_buffer[8],&encoder_eeprom.pos1_d,4);
      }
      else
      {
        memcpy(&i2c_buffer[0],&encoder_eeprom.pos0_p,4);
        memcpy(&i2c_buffer[4],&encoder_eeprom.pos0_i,4);
        memcpy(&i2c_buffer[8],&encoder_eeprom.pos0_d,4);
      }     
      break;
    case CMD_GET_SPEED:
      memcpy(&i2c_buffer[0],&encoder_data[slot].current_speed,4); 
      break;
    case CMD_GET_POS:
      memcpy(&i2c_buffer[0],&encoder_data[slot].current_pos,4);
      break;
    case CMD_GET_LOCK_STATE:
      memcpy(&i2c_buffer[0],&lock_flag[slot],2);
      break;
    case CMD_GET_RATIO:
      if(slot == MOTOR_1)
      {
        memcpy(&i2c_buffer[0],&encoder_eeprom.ratio1,4);
      }
      else
      {
        memcpy(&i2c_buffer[0],&encoder_eeprom.ratio0,4); 
      } 
      break;
    case CMD_GET_PULSE:
      if(slot == MOTOR_1)
      {
        memcpy(&i2c_buffer[0],&encoder_eeprom.pulse1,2);
      }
      else
      {
        memcpy(&i2c_buffer[0],&encoder_eeprom.pulse0,2);
      } 
      break;
  }
}

void parse_serial_cmd(char * cmd)
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
    motor_reset();
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
    set_position_to(slot,lock_state,pos_temp,speed_temp);
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
    set_position(slot,lock_state,pos_temp,speed_temp);
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
    set_speed(slot,lock_state,speed_temp);
  }
  else if(g_code_cmd == '4')
  {
    int8_t slot = MOTOR_0;
    int8_t mode = MOTION_PWM_I2C_PWM;
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
    set_mode(slot,mode);
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
    set_devicid(dev_id);
    Serial.print("G6: ");
    Serial.println(dev_id);
  }
}

void set_pwm(uint8_t slot,int16_t pwm)
{
  encoder_data[slot].mode = MOTION_PWM_I2C_PWM;
  encoder_data[slot].tar_pwm = pwm;
}

void set_position_to(uint8_t slot,uint8_t state,long turns,float speed)
{
  if(abs(speed) >= MIN_ENCODER_SPEED)
  {
    encoder_data[slot].mode = MOTION_POS_MODE;
    lock_flag[slot] = state;
    encoder_data[slot].target_pos = turns;
    float speed_temp = constrain(speed,-MAX_ENCODER_SPEED,MAX_ENCODER_SPEED);
    encoder_data[slot].target_speed = abs(speed_temp);
  }
  else
  {
    encoder_data[slot].mode = MOTION_IDLE_MODE;
  }
}

void set_position(uint8_t slot,uint8_t state,long turns,float speed)
{
  set_position_to(slot,state,(encoder_data[slot].current_pos + turns),speed);
}

void set_speed(uint8_t slot,uint8_t state,float speed)
{
  encoder_data[slot].mode = MOTION_SPEED_MODE;
  lock_flag[slot] = state;
  float speed_temp = constrain(speed,-MAX_ENCODER_SPEED,MAX_ENCODER_SPEED);
  encoder_data[slot].target_speed = speed_temp;
}

void set_mode(uint8_t slot,uint8_t mode)
{
  if(mode == MOTION_PWM_MODE)
  {
    encoder_data[slot].mode = MOTION_PWM_MODE;
  }
  else
  {
    encoder_data[slot].mode = MOTION_IDLE_MODE;
  }
}

void set_speed_pid(uint8_t slot, float p,float i,float d)
{
  float f[3] = {p,i,d};
  if(slot == MOTOR_1)
  {
    encoder_eeprom.speed1_p = p;
    encoder_eeprom.speed1_i = i;
    encoder_eeprom.speed1_d = d;
    eeprom_set_float(STORE_PID1_ADDR,3,f);
  }
  else
  {
    encoder_eeprom.speed0_p = p;
    encoder_eeprom.speed0_i = i;
    encoder_eeprom.speed0_d = d;
    eeprom_set_float(STORE_PID0_ADDR,3,f);
  }
}

void set_position_pid(uint8_t slot, float p,float i,float d)
{
  float f[3] = {p,i,d};
  if(slot == MOTOR_1)
  {
    encoder_eeprom.pos1_p = p;
    encoder_eeprom.pos1_i = i;
    encoder_eeprom.pos1_d = d;
    eeprom_set_float(STORE_PID1_ADDR+12,3,f);
  }
  else
  {
    encoder_eeprom.pos0_p = p;
    encoder_eeprom.pos0_i = i;
    encoder_eeprom.pos0_d = d;
    eeprom_set_float(STORE_PID0_ADDR+12,3,f);
  }
}

void set_ratio(uint8_t slot, float ratio)
{
  if(slot == MOTOR_1)
  {
    encoder_eeprom.ratio1 = ratio;
    eeprom_set_float(STORE_RATIO1_ADDR,1,&ratio);
  }
  else
  {
    encoder_eeprom.ratio0 = ratio;
    eeprom_set_float(STORE_RATIO0_ADDR,1,&ratio);
  }
}

void set_pulse(uint8_t slot, uint16_t pulse)
{
  if(slot == MOTOR_1)
  {
    encoder_eeprom.pulse1 = pulse;
    eeprom_set_int(STORE_PLUS1_ADDR,1,&pulse);
  }
  else
  {
    encoder_eeprom.pulse0 = pulse;
    eeprom_set_int(STORE_PLUS0_ADDR,1,&pulse);
  }
}

void set_curpositon(uint8_t slot, long pos)
{
  encoder_data[slot].mode = MOTION_IDLE_MODE;
  encoder_data[slot].pulse = (long)(pos * (encoder_data[slot].encoder_pulse * encoder_data[slot].ratio) / 360);//payton add
  encoder_data[slot].current_pos = pos;
}

void set_devicid(uint8_t devid)
{
  encoder_eeprom.devid = devid;
  eeprom_set_byte(STORE_DEVID_ADDR,1,&devid);
}

/**********************
0x0000 -> 1023  VCC
0x0100 -> 816
0x0010 -> 678
0x0110 -> 580
0x0001 -> 512
0x0101 -> 454
0x0011 -> 408
0x0111 -> 370
**********************/
uint8_t get_devceid(void)
{
  int val = analogRead(SET_I2C_ADDR_PIN);
  uint8_t address = DEFALUT_I2C_ADDR;

#ifdef DEBUG_INFO
  Serial.print("voltage of Addr_Set: ");
  Serial.println((val));
#endif

  if( val > 919)
  {
    address = DEFALUT_I2C_ADDR+0x00;
  }
  else if(val > 747)
  {
    address = DEFALUT_I2C_ADDR+0x04;
  }
  else if(val > 629)
  {
    address = DEFALUT_I2C_ADDR+0x02;
  }
  else if(val > 546)
  {
    address = DEFALUT_I2C_ADDR+0x06;
  }
  else if(val > 483)
  {
    address = DEFALUT_I2C_ADDR+0x01;
  }
  else if(val > 431)
  {
    address = DEFALUT_I2C_ADDR+0x05;
  }
  else if(val > 389)
  {
    address = DEFALUT_I2C_ADDR+0x03;
  }
  else if(val > 0)
  {
    address = DEFALUT_I2C_ADDR+0x07;
  }

  Serial.print("IIC address: ");
  Serial.println(address);
  return address;
}

/****************************************************************************************************
 * Motor Control Functions
****************************************************************************************************/
void motor_init(void)
{
  memset(&encoder_data[MOTOR_0],0,sizeof(encoder_data_type));
  memset(&encoder_data[MOTOR_1],0,sizeof(encoder_data_type));
  devic_id = encoder_eeprom.devid;
  encoder_data[MOTOR_0].slot = 0;
  encoder_data[MOTOR_0].mode = MOTION_IDLE_MODE;
  encoder_data[MOTOR_0].mode_change = true;
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
  encoder_data[MOTOR_0].ratio = encoder_eeprom.ratio0;
  encoder_data[MOTOR_0].encoder_pulse = encoder_eeprom.pulse0;

  encoder_data[MOTOR_1].slot = 1;
  encoder_data[MOTOR_1].mode = MOTION_IDLE_MODE;
  encoder_data[MOTOR_1].mode_change = true;
  encoder_data[MOTOR_1].tar_pwm = 0;
  encoder_data[MOTOR_1].cur_pwm = 0;
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
  encoder_data[MOTOR_1].ratio = encoder_eeprom.ratio1;
  encoder_data[MOTOR_1].encoder_pulse = encoder_eeprom.pulse1;
}

void motor_reset(void)
{
  motor_set_pwm(MOTOR_0,0);
  motor_set_pwm(MOTOR_1,0);
  delay(10);
  eeprom_read();
  delay(10);
  motor_init();
  //i2c_init((devic_id) << 1);//payton modify
  //i2c_init((get_devceid()) << 1);
}

void motor_set_pwm(uint8_t slot, int16_t set_pwm)
{
  //encoder_data[slot].cur_pwm = encoder_data[slot].cur_pwm * 0.0 + set_pwm  * 1.0;
  if(set_pwm > encoder_data[slot].cur_pwm+30)
    encoder_data[slot].cur_pwm += 30;
  else if(set_pwm < encoder_data[slot].cur_pwm-30)
    encoder_data[slot].cur_pwm -= 30;
  else
    encoder_data[slot].cur_pwm = set_pwm;

  int16_t pwm = encoder_data[slot].cur_pwm;

  if(slot == MOTOR_1)
  {
    pwm = constrain(pwm,-250,250);
    if(pwm < 0)
    {
      analogWrite(MOTOR_2_PWM_A, 0);
      analogWrite(MOTOR_2_PWM_B, abs(pwm));
    }
    else
    {
      analogWrite(MOTOR_2_PWM_B, 0);
      analogWrite(MOTOR_2_PWM_A, abs(pwm));
    }
  }
  else // slot == MOTOR_2
  {
    pwm = constrain(pwm,-245,250);
    if(pwm < 0)
    {
      analogWrite(MOTOR_1_PWM_B, 0);
      analogWrite(MOTOR_1_PWM_A, abs(pwm));
    }
    else
    {
      analogWrite(MOTOR_1_PWM_A, 0);
      analogWrite(MOTOR_1_PWM_B, abs(pwm));
    }
  }
}

void motor_idle_mode(uint8_t slot)
{
  encoder_output[slot] = 0;
  motor_set_pwm(slot,encoder_output[slot]);
}

void motor_pwm_mode(uint8_t slot)
{
  double pwm_read_temp=0; 
  static int8_t index = 0; 
  static int pwm_buff[8] = {0};
  static int sum = 0;

  if(encoder_data[slot].mode_change)
  {

  }

  pwm_buff[index] = analogRead(pwm_input_pin[slot]);  //Read PWM values
  sum += pwm_buff[index];
  pwm_read_temp = (sum / 8); 
  index++;
  index %= 8;
  sum -= pwm_buff[index];

  encoder_data[slot].tar_pwm = (pwm_read_temp-512)*255/512;
  
  encoder_output[slot] = 0.8 * encoder_output[slot] + 0.2 * encoder_data[slot].tar_pwm;
  if((abs(encoder_output[slot])<=20) && (encoder_data[slot].tar_pwm==0))
  {
    encoder_output[slot] = 0;
  }

  motor_set_pwm(slot,encoder_output[slot]);
}

void motor_pwmi2c_mode(uint8_t slot)
{
  if(encoder_data[slot].mode_change)
  {
 
  }
  
  encoder_output[slot] = 0.9 * encoder_output[slot] + 0.1 * encoder_data[slot].tar_pwm;
  if((abs(encoder_output[slot])<=20) && (encoder_data[slot].tar_pwm==0))
  {
    encoder_output[slot] = 0;
  }

  motor_set_pwm(slot,encoder_output[slot]);
}

void motor_position_mode(uint8_t slot)
{
  float speed_error;
  float pos_error;
  float target_speed;

  if(encoder_data[slot].mode_change)
  {
    encoder_data[slot].PID_speed.integral = encoder_output[slot] / encoder_data[slot].PID_speed.I;
    encoder_data[slot].PID_speed.output = 0;
    encoder_data[slot].PID_pos.integral = 0;
    encoder_data[slot].PID_pos.output = 0;
    encoder_data[slot].PID_pos.last_error = 0;
  }

  pos_error = encoder_data[slot].target_pos - encoder_data[slot].current_pos;
  if(abs(pos_error)>ENCODER_POS_DEADBAND || abs(encoder_data[slot].current_speed)>MIN_ENCODER_SPEED)
  {   
    //speed pid
    target_speed = abs(pos_error) * 0.7;
    target_speed = constrain(target_speed,MIN_ENCODER_SPEED,abs(encoder_data[slot].target_speed));
    if(pos_error > 0){
      target_speed= abs(target_speed);
    }
    else{
      target_speed= -abs(target_speed);
    }
    speed_error = target_speed - encoder_data[slot].current_speed;
    speed_error = constrain(speed_error,-20,20);
    encoder_data[slot].PID_speed.integral += speed_error;
    encoder_data[slot].PID_speed.integral = constrain(encoder_data[slot].PID_speed.integral,-1024,1024);
    encoder_data[slot].PID_speed.output = encoder_data[slot].PID_speed.P * speed_error + \
                                          encoder_data[slot].PID_speed.I * encoder_data[slot].PID_speed.integral;
    encoder_data[slot].PID_speed.output = constrain(encoder_data[slot].PID_speed.output,-255,255);
    
    //pos pid
    pos_error = constrain(pos_error,-20,20);
    encoder_data[slot].PID_pos.output = encoder_data[slot].PID_pos.P * pos_error + \
                                        encoder_data[slot].PID_pos.D * (pos_error - encoder_data[slot].PID_pos.last_error);
    encoder_data[slot].PID_pos.output = constrain(encoder_data[slot].PID_pos.output,-255,255);
    encoder_data[slot].PID_pos.last_error = pos_error;
  }
  else
  {
    encoder_data[slot].PID_speed.integral = 0;
    encoder_data[slot].PID_speed.output = 0;
    encoder_data[slot].PID_pos.integral = 0;
    encoder_data[slot].PID_pos.output = 0;
  }

  encoder_output[slot] = encoder_data[slot].PID_pos.output + encoder_data[slot].PID_speed.output;
  encoder_output[slot] = constrain(encoder_output[slot],-255,255);

  motor_set_pwm(slot,encoder_output[slot]);

#ifdef DEBUG_INFO
  Serial.print("slot:");
  Serial.print(slot);
  Serial.print(" ,tar_pos:");
  Serial.print(encoder_data[slot].target_pos);
  Serial.print(" ,cur_pos:");
  Serial.print(encoder_data[slot].current_pos);
  Serial.print(" ,pos_error:");
  Serial.print(pos_error);

  Serial.print(" ,tar_speed:");
  Serial.print(target_speed);
  Serial.print(" ,cur_speed:");
  Serial.print(encoder_data[slot].current_speed);
  Serial.print(" ,speed_error:");
  Serial.print(speed_error);

  Serial.print(" ,integral:");
  Serial.print( encoder_data[slot].PID_speed.integral);
  
  Serial.print(" ,out:");
  Serial.println(encoder_output[slot]);
#endif
}

void motor_speed_mode(uint8_t slot)
{
  float speed_error;

  if(encoder_data[slot].mode_change)
  {
    encoder_data[slot].PID_speed.integral = encoder_output[slot] / encoder_data[slot].PID_speed.I;
    encoder_data[slot].PID_speed.output = 0;

    encoder_data[slot].PID_speed.P = 0.5;
    encoder_data[slot].PID_speed.I = 0.1;
  }
  
  speed_error = encoder_data[slot].target_speed - encoder_data[slot].current_speed;
  speed_error = constrain(speed_error,-20,20);
  encoder_data[slot].PID_speed.integral += speed_error;
  encoder_data[slot].PID_speed.integral = constrain(encoder_data[slot].PID_speed.integral,-1024,1024);
  encoder_data[slot].PID_speed.output = encoder_data[slot].PID_speed.P * speed_error + \
                                        encoder_data[slot].PID_speed.I * encoder_data[slot].PID_speed.integral;
  encoder_data[slot].PID_speed.output = constrain(encoder_data[slot].PID_speed.output,-255,255);
  encoder_output[slot] = encoder_data[slot].PID_speed.output;

  motor_set_pwm(slot,encoder_output[slot]);

#ifdef DEBUG_INFO
  Serial.print("slot:");
  Serial.print(slot);
  Serial.print(",Kp:");
  Serial.print(encoder_data[slot].PID_speed.P);
  Serial.print(slot);
  Serial.print(",Ki:");
  Serial.print(encoder_data[slot].PID_speed.I);
  Serial.print(slot);
  Serial.print(",integral:");
  Serial.print(encoder_data[slot].PID_speed.integral);
  Serial.print(" ,tar_speed:");
  Serial.print(encoder_data[slot].target_speed);
  Serial.print(" ,cur_speed:");
  Serial.print(encoder_data[slot].current_speed);
  Serial.print(" ,speed_error:");
  Serial.print(speed_error);
  Serial.print(" ,out:");
  Serial.println(encoder_output[slot]);
#endif
}

void motor_process(uint8_t slot)
{
  static uint8_t pre_state[2] = {0xFF};

  encoder_data[slot].mode_change = false;
  if(pre_state[slot] != encoder_data[slot].mode)
  {
    encoder_data[slot].mode_change = true;
  }
  pre_state[slot] = encoder_data[slot].mode;

  switch(encoder_data[slot].mode)
  {
    case MOTION_IDLE_MODE:
      motor_idle_mode(slot);
      break;
        
    case MOTION_PWM_MODE:
      motor_pwm_mode(slot);
      break;
        
    case MOTION_PWM_I2C_PWM:
      motor_pwmi2c_mode(slot);
      break; 
        
    case MOTION_POS_MODE:
      motor_position_mode(slot);
      break;

    case MOTION_SPEED_MODE:
      motor_speed_mode(slot);
      break;
        
    default: 
      motor_idle_mode(slot);
      break;
  }
}

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

void update_speed(void)
{
  static long last_pulse_pos_encoder[2] = {0};
  static long measurement_speed_time = 0;
  uint8_t ch;

  if((millis() - measurement_speed_time) > 10)
  {
    uint16_t dt = millis() - measurement_speed_time;
    measurement_speed_time = millis();
    for(ch=0; ch<2; ch++)
    {
      long cur_pos = encoder_data[ch].pulse;
      encoder_data[ch].current_speed = ((cur_pos - last_pulse_pos_encoder[ch])/(encoder_data[ch].encoder_pulse * encoder_data[ch].ratio))*(1000/dt)*60;
      last_pulse_pos_encoder[ch] = cur_pos;
    }
  }
}

void update_position(void)
{
  encoder_data[MOTOR_0].current_pos = (long)((encoder_data[MOTOR_0].pulse/(encoder_data[MOTOR_0].encoder_pulse * encoder_data[MOTOR_0].ratio)) * 360);
  encoder_data[MOTOR_1].current_pos = (long)((encoder_data[MOTOR_1].pulse/(encoder_data[MOTOR_1].encoder_pulse * encoder_data[MOTOR_1].ratio)) * 360);
}

void pwm_frequency_init(void)
{
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);
}

void setup() {
  delay(10);
  Serial.begin(115200);
  pwm_frequency_init();
  pinMode(INT_1_PIN, INPUT_PULLUP);
  pinMode(DIR_1_PIN, INPUT_PULLUP);
  pinMode(INT_2_PIN, INPUT_PULLUP);
  pinMode(DIR_2_PIN, INPUT_PULLUP);

  pinMode(WORK_LED_PIN, OUTPUT);
  digitalWrite(WORK_LED_PIN, HIGH);

  // Drive pinMode configure
  pinMode(MOTOR_1_PWM_A, OUTPUT);
  pinMode(MOTOR_1_PWM_B, OUTPUT);
  pinMode(MOTOR_2_PWM_A, OUTPUT);
  pinMode(MOTOR_2_PWM_B, OUTPUT);

  // Drive Control pinMode configure
  pinMode(MOTOR_DRV_ERR_FB_PIN,INPUT_PULLUP);
  pinMode(MOTOR_DRV_SHUTDOWN_CTRL_PIN,OUTPUT);
  digitalWrite(MOTOR_DRV_SHUTDOWN_CTRL_PIN, HIGH);
  pinMode(MOTOR_DRV_STBYB_PIN,INPUT);  // do not use INPUT_PULLUP mode !!!

  EICRA = 0x00 | (1 << ISC11) | (1 << ISC01);  //interrupt 1 & interrupt 0 set falling edge trig
  EIMSK = (1 << INT0) | (1 << INT1);

  delay(10);
  eeprom_read();
  delay(10);
  motor_init();
  //i2c_init((devic_id) << 1);
  i2c_init((get_devceid()) << 1);
}

void loop()
{
  //G code processing
  while(Serial.available() > 0)
  {
    char c = Serial.read();
    Serial.write(c);
    uart_buffer[uart_index++]=c;
    if((c=='\n') || (c=='#'))
    {
      if((uart_buffer[0]=='g') || (uart_buffer[0]=='G'))
      {
        parse_serial_cmd(&uart_buffer[1]);
      }
      memset(uart_buffer,0,64);
      uart_index = 0;
    }
  }

  update_position();
  update_speed();

  if(millis() - motor_ctrl_time > 10)
  {
    motor_ctrl_time = millis();
    motor_process(MOTOR_0);
    motor_process(MOTOR_1);

    MotorDrvErrPcs();
  }
}


