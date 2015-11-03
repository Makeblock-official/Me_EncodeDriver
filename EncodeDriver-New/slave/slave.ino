#include <Arduino.h>
#include <avr/io.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <EEPROM.h>
#include "encodedMotor.h"

#define DEFALUT_I2C_ADDR 0x09
#define ADJUSTMENT_AREA  160
//#define DEBUG_MESSAGE
//#define INFO_MESSAGE 1

#define INT_1_PIN 3
#define DIR_1_PIN 10
#define INT_2_PIN 2
#define DIR_2_PIN 12

#define MOTOR_1_PWM 6
#define MOTOR_1_H1 A0
#define MOTOR_1_H2 A1

#define MOTOR_2_PWM 5
#define MOTOR_2_H1 A2
#define MOTOR_2_H2 A3

// #define ST_PIN A2

#define MOTOR_0 0
#define MOTOR_1 1
#define MOTOR_BOTH 2

#define MOTOR_1_IRQ INT1_vect
#define MOTOR_1_PIN PINB
#define MOTOR_1_DIR PINB2

#define MOTOR_2_IRQ INT0_vect
#define MOTOR_2_PIN PINB
#define MOTOR_2_DIR PINB4

#define PULSE_PER_C 8 
EMotor motor[2];
EPids Pids;

#ifdef INFO_MESSAGE
unsigned char pre_state[2];
#endif

union{
  byte byteVal[16];
  int intVal[8];
  float floatVal[4];
  long longVal[4];
}
val;

void setDefaultvalue()
{
  Serial.println("setDefaultvalue");
  Pids.start = EEPROM_PID_START;
  Pids.mid = EEPROM_PID_MID;
  Pids.end = EEPROM_PID_END;
  Pids.devid = DEFALUT_I2C_ADDR;
  Pids.p0 = 2.0;
  Pids.i0 = 2.0;
  Pids.d0 = 0.2;
  Pids.s0 = 2.0;
  Pids.ratio0 = 46.666;
  Pids.pulse0 = 8;
  Pids.p1 = 2.0;
  Pids.i1 = 2.0;
  Pids.d1 = 0.2;
  Pids.s1 = 2.0;
  Pids.ratio1 = 46.666;
  Pids.pulse1 = 8;
}

void updatetoEEPROM(int address,int num,float *f)
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


void updatetoEEPROM(int address,int num,int *data)
{
  int i,j,bakupaddress;
  bakupaddress = address +  EEPROM.length()/2;
  for(i=0;i<2;i++)
  {
    for(j=0;j<num;j++)
    {
      EEPROM.put(address, data[j]);  
      address = address + sizeof(int);
    }
    address = bakupaddress;
  }
}

void writetoEEPROM()
{
  int i,address, length;
  length = EEPROM.length();
  for(i = 0; i < 2; i++)
  {
    EEPROM.write(EEPROM_START_POS, EEPROM_IF_HAVEPID_CHECK1);
    EEPROM.write(EEPROM_START_POS + 1, EEPROM_IF_HAVEPID_CHECK2);
    EEPROM.put(STORE_START_ADDR, Pids);
    address = length/2;
  } 
}

void updatefromBackuparea()
{
  int address, length;
  length = EEPROM.length();
  address = length/2;
  if((EEPROM.read(address) == EEPROM_IF_HAVEPID_CHECK1) & (EEPROM.read(address + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    address = length/2 + 2;
    EEPROM.get(address, Pids);
    if((Pids.start == EEPROM_PID_START) & (Pids.mid == EEPROM_PID_MID) & (Pids.end == EEPROM_PID_END))
    {
      EEPROM.write(EEPROM_START_POS, EEPROM_IF_HAVEPID_CHECK1);
      EEPROM.write(EEPROM_START_POS+1, EEPROM_IF_HAVEPID_CHECK2);
      EEPROM.put(STORE_START_ADDR, Pids);
    }
    else
    {
      setDefaultvalue();  
      writetoEEPROM(); 
    }
  }
}

int readEEPROM(){
  Serial.println( "Read data from EEPROM " );
  int i,j,length;
  i = sizeof(motor);
  length = EEPROM.length();
  if((EEPROM.read(EEPROM_START_POS) == EEPROM_IF_HAVEPID_CHECK1) & (EEPROM.read(EEPROM_START_POS + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    EEPROM.get(STORE_START_ADDR, Pids);
    if((Pids.start == EEPROM_PID_START) & (Pids.mid == EEPROM_PID_MID) & (Pids.end == EEPROM_PID_END))
    {
      return 1;
    }
    else
    {
      Serial.println( "updatefromBackuparea" );
      updatefromBackuparea();  
      return 1;  
    }
  }
  else if((EEPROM.read(length/2) == EEPROM_IF_HAVEPID_CHECK1) & (EEPROM.read(length/2 + 1) == EEPROM_IF_HAVEPID_CHECK2))
  {
    Serial.println( "updatefromBackuparea 2" );
    updatefromBackuparea(); 
    return 1;
  }
  else
  {
    setDefaultvalue();
    writetoEEPROM();    
  }
}

/******* I2C slave ********/
char bufI2C[32];
int rd,wr;
void I2C_init(uint8_t address){
  // load address into TWI address register
  TWAR = address;

  //TWCR = 0x00; //Disable TWI
  TWBR = 0x02;   //Set the baudrate锟�
  TWSR|= 0x00;   //Set Divider factor
  //TWCR = 0x45; //Start TWI
  // set the TWCR to enable address matching and enable TWI, clear TWINT, enable TWI interrupt
  TWCR = (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWEN);
}

void I2C_stop(void){
  // clear acknowledge and enable bits
  TWCR &= ~( (1<<TWEA) | (1<<TWEN) );
}

ISR(TWI_vect){
  // temporary stores the received data
  uint8_t data;
  //Serial.print(TWSR,HEX);
  // own address has been acknowledged
  if( (TWSR & 0xF8) == TW_SR_SLA_ACK ){  
    //Serial.println("1");
    rd=0;
    wr=0;
    // clear TWI interrupt flag, prepare to receive next byte and acknowledge
    TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT) | (1<<TWEA);
  }
  else if( (TWSR & 0xF8) == TW_SR_DATA_ACK ){ // data has been received in slave receiver mode
    // save the received byte inside data 
    //Serial.println("2");
    data = TWDR;
    bufI2C[rd++] = data;
    //Serial.print((int)data,HEX);Serial.print(",");
    TWCR = (1<<TWEN) | (1<<TWIE) | (1<<TWINT) | (1<<TWEA);
  }
  else if((TWSR & 0xF8)==TW_ST_SLA_ACK){
    // the start of i2c read
    TWDR = bufI2C[wr++]; // todo: errata of avr, to insert delay between twdr and twcr?
    // clear TWI interrupt flag, prepare to send next byte and receive acknowledge
    TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
  }
  else if( (TWSR & 0xF8) == TW_ST_DATA_ACK ){ // device has been addressed to be a transmitter
    // copy the specified buffer address into the TWDR register for transmission
    TWDR = bufI2C[wr++];
    // clear TWI interrupt flag, prepare to send next byte and receive acknowledge
    TWCR |= (1<<TWIE) | (1<<TWINT) | (1<<TWEA) | (1<<TWEN); 
  }
  else if((TWSR & 0xF8) == TW_SR_STOP){
    //Serial.println("5");
    parseCmd(bufI2C);
    // if none of the above apply prepare TWI to be addressed again
    //TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT)|_BV(TWSTO);
    //while(TWCR & _BV(TWSTO)){
    //  continue;
    //}
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
  }
  else{
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT)|_BV(TWSTO);
    while(TWCR & _BV(TWSTO)){
      continue;
    }
    TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA) | _BV(TWINT);
  }
}

void setMotor1Pwm(int pwm)
{
  pwm = constrain(pwm,-255,255);
  if(pwm<0){
    digitalWrite(MOTOR_1_H1, LOW);
    digitalWrite(MOTOR_1_H2, HIGH);
    analogWrite(MOTOR_1_PWM, abs(pwm));
  }
  else{
    digitalWrite(MOTOR_1_H1, HIGH);
    digitalWrite(MOTOR_1_H2, LOW);
    analogWrite(MOTOR_1_PWM, abs(pwm));
  }
}

void setMotor2Pwm(int pwm)
{
  pwm = constrain(pwm,-255,255);
  if(pwm<0){
    digitalWrite(MOTOR_2_H1, LOW);
    digitalWrite(MOTOR_2_H2, HIGH);
    analogWrite(MOTOR_2_PWM, abs(pwm));
  }
  else{
    digitalWrite(MOTOR_2_H1, HIGH);
    digitalWrite(MOTOR_2_H2, LOW);
    analogWrite(MOTOR_2_PWM, abs(pwm));
  }
}

float dt=0.01; // 10ms
float calcPid1(EMotor * m)
{
  float error,spderror;
  float output;
  error = m->pos - m->targetPos;
  spderror = m->targetSpd - m->speed;
  m->PTerm = error*m->p;
  if(m->state==CMD_STOP || m->state==CMD_BREAK){
    if(m->state==CMD_STOP && m->hold==0) return 0;
    m->ITerm= m->ITerm+m->i*error*dt;
    m->ITerm = constrain(m->ITerm,(-m->power),(m->power));
    m->DTerm = m->d*(m->pos-m->posLast)/dt; // not interferenced by setpoint change
    output = constrain((m->PTerm + m->ITerm + m->DTerm),(-m->power),(m->power));
    if(m->state==CMD_BREAK && abs(error)==1){
      m->state = CMD_STOP;
    }
  }
  else if(m->state==CMD_MOVE_TO){
    output = constrain(m->PTerm,-m->power,m->power);
    if(abs(error)<10){
      m->ITerm = 0;
      m->state = CMD_BREAK;
    }
  }
  else if(m->state==CMD_MOVE_SPD){
    m->STerm = m->STerm - m->s*spderror*dt;
    m->STerm = constrain(m->STerm,-m->power,m->power);
    output = m->STerm;
  }
  m->posLast = m->pos;
  return output;
}

float previous_error = 0;

int calcPid(EMotor * m)
{
  float error,spderror;
  float integral,derivative;
  int output;
  error =  m->targetPos - m->pos;
//  integral = integral + error*dt;
//  derivative = (error - previous_error)/dt;
//  previous_error = error;
//  
//  output = m->p*error + m->i*integral + m->d*derivative;
//  output = constrain(output,(-m->power),(m->power));
  
  spderror = m->targetSpd - m->speed;
  m->PTerm = error*m->p;
#ifdef DEBUG_MESSAGE
  Serial.println("\r\n\r\n\r\n\r\n\r\n-----------------calcPid-------------------");
  Serial.print("error: ");
  Serial.print(error);
  Serial.print("       spderror: ");
  Serial.print(spderror);
  Serial.print("       m->PTerm: ");
  Serial.print(m->PTerm);
  Serial.print("       m->state: ");
  Serial.println(m->state);
  
  Serial.print("m->hold: ");
  Serial.print(m->hold);
  Serial.print("       m->targetSpd: ");
  Serial.print(m->targetSpd);
  Serial.print("       m->targetPos: ");
  Serial.print(m->targetPos);
  Serial.print("       m->power: ");
  Serial.println(m->power);
  Serial.println("-----------------End-------------------");
 #endif
#ifdef INFO_MESSAGE
  if(pre_state[m->index] != m->state)
  {
    Serial.print("m->state:");
    Serial.println(m->state);
    Serial.print("m->targetSpd:");
    Serial.println(m->targetSpd);
    pre_state[m->index] = m->state;  
  }
#endif
  if(m->state == CMD_STOP || m->state == CMD_BREAK)
  {
    if(m->state == CMD_STOP && m->hold == 0)
    {
#ifdef DEBUG_MESSAGE
      m->STerm = 0;
      Serial.println("m-state is 0 or hold is 0");
#endif
      return 0;
    }
    m->ITerm= m->ITerm+m->i*error*dt;
    m->ITerm = constrain(m->ITerm,(-m->power),(m->power));
    m->DTerm = m->d*(m->posLast - m->pos)/dt; // not interferenced by setpoint change
#ifdef INFO_MESSAGE
    Serial.print("m->PTerm: ");
    Serial.print(m->PTerm);
    Serial.print("  m->ITerm: ");
    Serial.print( m->ITerm);
    Serial.print("  m->DTerm: ");
    Serial.println( m->DTerm);
#endif
    output = constrain((m->PTerm + m->ITerm + m->DTerm),(-m->power),(m->power));
    if(m->state == CMD_BREAK && abs(error)==1){
      m->state = CMD_STOP;
      m->STerm = 0;
      return 0;
    }
  }
  else if(m->state==CMD_MOVE_TO){
    if((m->targetSpd) != 0)
    {
      Serial.print("Ofset: ");
      Serial.println(m->s*spderror*dt);
      m->STerm = m->STerm + m->s*spderror*dt;
      m->STerm = constrain(m->STerm,-m->power,m->power);
      output = m->STerm;         
    }
    else
    {
#ifdef DEBUG_MESSAGE
      Serial.println("targetSpd is 0:");
#endif
#ifdef INFO_MESSAGE
      Serial.print("m->PTerm: ");
      Serial.println(m->PTerm);
      Serial.print("m->power: ");
      Serial.println(m->power);
#endif
      if((m->power) > 200)
      {
        output = constrain(m->PTerm,-((m->power)/2),(m->power)/2);
      }
      else
      {
        output = constrain(m->PTerm,-m->power,m->power);
      }
    }
    if(abs(error) < ADJUSTMENT_AREA){
      m->ITerm = 0;
      m->state = CMD_BREAK;
    }
  }
  else if(m->state == CMD_MOVE_SPD){
    m->STerm = m->STerm + m->s*spderror*dt;
    m->STerm = constrain(m->STerm,-m->power,m->power);
//    Serial.print("Ofset: ");
//    Serial.println(m->s*spderror*dt);
//    Serial.print("output: ");
//    Serial.println(m->STerm);
    output = m->STerm;
  }
  m->posLast = m->pos;
  return output;
}

void updateSpeed(EMotor * m)
{
  m->speed = m->pos - m->posSpeed;
  m->posSpeed = m->pos;
}

void initMotor()
{
  memset(&motor[0],0,sizeof(EMotor));
  memset(&motor[1],0,sizeof(EMotor));
  motor[0].index = 0;
  motor[0].mode = 0;
  motor[0].p = Pids.p0;
  motor[0].i = Pids.i0;
  motor[0].d = Pids.d0;
  motor[0].s = Pids.s0;
  motor[0].ratio = Pids.ratio0;
  motor[0].pulse = Pids.pulse0;
  motor[1].index = 1;
  motor[1].mode = 0;
  motor[1].p = Pids.p1;
  motor[1].i = Pids.i1;
  motor[1].d = Pids.d1;
  motor[1].s = Pids.s1;
  motor[1].ratio = Pids.ratio1;
  motor[1].pulse = Pids.pulse1;
}

void resetMotor(EMotor * m){
  m->ITerm = 0;
  m->STerm = 0;
  m->pos = 0;
  m->posLast = 0;
  m->state = CMD_STOP;
  m->stopCount = -1;
  m->power = 255;
  m->hold = 0;
}

void moveTo(EMotor * m, long angle, int spd)
{
   m->targetPos = (long)((float)angle * (m->ratio) * (m->pulse)/360);
  if(spd!=0){
     m->targetSpd = (int)((float)spd * (m->ratio)*(m->pulse)/600);// change to degree per 100ms
     if(m->targetPos - m->pos> 0)
     {
       m->targetSpd = abs(m->targetSpd);
     }
     else
     {
       m->targetSpd = -abs(m->targetSpd);
     }
  }
#ifdef INFO_MESSAGE
   Serial.print("targetPos: ");
   Serial.println(m->targetPos);
   Serial.print("spd: ");
   Serial.println(m->targetSpd);
#endif
  m->STerm = m->targetSpd;
  m->state = CMD_MOVE_TO;
}

void moveSpeed(EMotor * m, int rpm, int cntDown)
{
  m->targetSpd = (int)((float)rpm * (m->ratio)*(m->pulse)/600);// change to degree per 100ms
#ifdef INFO_MESSAGE
  Serial.print("rpm: ");
  Serial.print(rpm);
  Serial.print("     targetSpd: ");
  Serial.println(m->targetSpd);
#endif
  m->stopCount = cntDown;
  if(rpm==0){
    m->state = CMD_STOP;
  }
  else{
    m->state = CMD_MOVE_SPD;
  }
}

void setPid(EMotor * m,float p,float i,float d,float s)
{
  float f[4] = {p,i,d,s};
  m->p = p;
  m->i = i;
  m->d = d;
  m->s = s;
  if(m->index == 0)
  {
    Pids.p0 = p;
    Pids.i0 = i;
    Pids.d0 = d;
    Pids.s0 = s; 
    updatetoEEPROM(STORE_PIDS0_ADDR,4,f);
  }
  if(m->index == 1)
  {
    Pids.p1 = p;
    Pids.i1 = i;
    Pids.d1 = d;
    Pids.s1 = s; 
    updatetoEEPROM(STORE_PIDS1_ADDR,4,f);
  }
}

void setRatio(EMotor * m,float ratio)
{
  m->ratio = ratio;
  if(m->index == 0)
  {
    Pids.ratio0 = ratio;
    updatetoEEPROM(STORE_RATIO0_ADDR,1,&ratio);    
  }
  if(m->index == 1)
  {
    Pids.ratio1 = ratio;
    updatetoEEPROM(STORE_RATIO1_ADDR,1,&ratio);     
  }
}

void setPulse(EMotor * m,int pulse)
{
  m->pulse = pulse;
  if(m->index == 0)
  {
    Pids.pulse0 = pulse;
    updatetoEEPROM(STORE_PLUS0_ADDR,1,&pulse);      
  }
  if(m->index == 1)
  {
    Pids.pulse1 = pulse;
    updatetoEEPROM(STORE_PLUS1_ADDR,1,&pulse);     
  }
}

void setDevid(EMotor * m,int devid)
{
  Pids.devid = devid;
  updatetoEEPROM(STORE_DEVID_ADDR,1,&devid);
}

/***** parse cmd *****/
// byte 0: motor index
// byte 1: cmd
// byte 2-n: parameters
void parseCmd(char * c)
{
  int rpm;
  long angle;
  int power;
  EMotor * m = &motor[c[0]];
  m->index = c[0];
  unsigned char hold = (c[1]&HOLD);
  unsigned char sync = (c[1]&SYNC);
  unsigned char cmd = (c[1]&0x3f);
  memcpy(&val,c+2,16);
#ifdef INFO_MESSAGE
  Serial.print("parseCmd: 0x");
  Serial.println(cmd,HEX);
#endif
  switch(cmd){
    // move state and function
  case CMD_STOP:
    resetMotor(m);
    break;
  case CMD_MOVE_TO:    
    moveTo(m,val.longVal[0],0);
    break;
  case CMD_MOVE_TO_SPD:
    moveTo(m,val.longVal[0],val.longVal[1]);
    break;
  case CMD_MOVE_SPD:
    moveSpeed(m,val.intVal[0],-1);
    break;
  case CMD_SET_PID:
    setPid(m,val.floatVal[0],val.floatVal[1],val.floatVal[2],val.floatVal[3]);
    break;
  case CMD_SET_HOLD:
    m->hold = val.byteVal[0];
    break;
  case CMD_SET_POWER:
    m->power = val.byteVal[0]*255/100;
    break;
  case CMD_SET_MODE:
    m->mode = val.byteVal[0];
    m->pwm = 0;
    m->power = 0;
    break;
  case CMD_SET_PWM:
    m->pwm = val.intVal[0];
    break;
  case CMD_SET_RATIO:
//    m->ratio = val.floatVal[0];
    setRatio(m,val.floatVal[0]);
    break;
  case CMD_SET_PULSE:
//    m->pulse = val.intVal[0];
    setPulse(m,val.intVal[0]);
    break;
  case CMD_SET_DEVID:
//    m->devid = val.intVal[0];
    setDevid(m,val.intVal[0]);
    break;
    // get motor status
  case CMD_GET_PID:    
//    bufI2C[0] = (uint8_t)(m->p*100);
//    bufI2C[1] = (uint8_t)(m->i*100);
//    bufI2C[2] = (uint8_t)(m->d*100);
//    bufI2C[3] = (uint8_t)(m->s*100);
     memcpy(&bufI2C[0],&m->p,4);
     memcpy(&bufI2C[4],&m->i,4);
     memcpy(&bufI2C[8],&m->d,4);
     memcpy(&bufI2C[12],&m->s,4);
    break;
  case CMD_GET_POWER:
    bufI2C[0] = (m->pwm*100/255);
    memcpy(bufI2C,&power,1);
    break;
  case CMD_GET_SPEED:
    rpm = (int)((float)m->speed*600/m->pulse/m->ratio); // change to rpm
//    rpm = (int)((float)m->speed*600/PULSE_PER_C/m->ratio); // change to rpm
    memcpy(bufI2C,&rpm,2);
    break;
  case CMD_GET_POS:
    angle = (long)((float)m->pos/(m->pulse*m->ratio/360.0));
//    angle = (long)((float)m->pos/(PULSE_PER_C*m->ratio/360.0));
    memcpy(bufI2C,&angle,4);
    break;
    case CMD_GET_RATIO:
    memcpy(bufI2C,&m->ratio,4);
    break;
    case CMD_GET_PULSE:
    memcpy(bufI2C,&m->pulse,2);
    break;
  }
}

/******* loop delay function *******/
#define FIXDELAY 10*1000 // 10ms loop delay
long time,deltaTime;
void fixdelay()
{
  deltaTime = micros()-time;
  delayMicroseconds(FIXDELAY-deltaTime);
  time = micros();
}

void setup() {
  Serial.begin(115200);
  pinMode(INT_1_PIN, INPUT_PULLUP);
  pinMode(DIR_1_PIN, INPUT_PULLUP);
  pinMode(INT_2_PIN, INPUT_PULLUP);
  pinMode(DIR_2_PIN, INPUT_PULLUP);

  pinMode(MOTOR_1_H1,OUTPUT);
  pinMode(MOTOR_1_H2,OUTPUT);
  pinMode(MOTOR_2_H1,OUTPUT);
  pinMode(MOTOR_2_H2,OUTPUT);
  EICRA = (1 << ISC11) | (1 << ISC01);
  EIMSK = (1 << INT0) | (1 << INT1);
  delay(10);  
  readEEPROM();
  delay(10);  
  Serial.print("\r\n-------EEPROM.get ---------\r\n");
  Serial.print("----Pids.ratio0: ");
  Serial.println(Pids.ratio0);
  Serial.print("----Pids.pulse0: ");
  Serial.println(Pids.pulse0);
  Serial.print("----Pids.ratio1: ");
  Serial.println(Pids.ratio1);
  Serial.print("----Pids.pulse1: ");
  Serial.println(Pids.pulse1);
  Serial.print("----Pids.start: ");
  Serial.println(Pids.start,HEX);
  Serial.print("----Pids.mid: ");
  Serial.println(Pids.mid,HEX);
  Serial.print("----Pids.end: ");
  Serial.println(Pids.end,HEX);
  Serial.print("----Pids.devid: ");
  Serial.println(Pids.devid);
  if(Pids.devid == 0)
  {
    Serial.println( "Set Default init" );
    setDefaultvalue();
    writetoEEPROM();  
  }
  I2C_init((Pids.devid)<<1);
  initMotor();
  resetMotor(&motor[0]);
  resetMotor(&motor[1]);
}

int speedCnt = 0;
void loop() {
  int pwm;
  if(motor[0].mode)
  {
    setMotor1Pwm(motor[0].pwm);
  }
  else
  {
    pwm = calcPid(&motor[0]);
    setMotor1Pwm(pwm);
    motor[0].pwm = pwm;
  }


  if(motor[1].mode)
  {
    setMotor2Pwm(motor[1].pwm);
  }
  else
  {
    pwm=calcPid(&motor[1]);
    setMotor2Pwm(pwm);
    motor[1].pwm = pwm;
  }

  if(speedCnt++ == 9)
  {
    updateSpeed(&motor[0]);
    updateSpeed(&motor[1]);
    speedCnt = 0;
  }
//  Serial.print(motor[0].speed);
//  Serial.print(',');
//  Serial.print(sin(PI*((float)motor[0].pos/(PULSE_PER_C*motor[0].ratio/360.0)/180)));
//  Serial.println();
  /*
    Serial.print(motor[1].pos);Serial.print(',');
   Serial.print(motor[1].targetPos);Serial.print(',');
   Serial.print(pwm);Serial.print(',');
   Serial.print(motor[1].ITerm);Serial.print(',');
   Serial.print(motor[1].speed);Serial.print(',');
   Serial.println(motor[1].STerm);
   */
  fixdelay();
}

ISR(MOTOR_1_IRQ)
{
  if(MOTOR_1_PIN & (1<<MOTOR_1_DIR))
  {
    motor[0].pos--;
  }
  else{
    motor[0].pos++;
  }
}

ISR(MOTOR_2_IRQ)
{
  if(MOTOR_2_PIN & (1<<MOTOR_2_DIR))
  {
    motor[1].pos--;
  }
  else{
    motor[1].pos++;
  }
}

