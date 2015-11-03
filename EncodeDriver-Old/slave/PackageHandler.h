#include "Arduino.h"
#include "stdint.h"

//  frame type
#define GET_PARAM     0x01
#define SAVE_PARAM    0x02
#define TEST_PARAM    0x03
#define SHOW_PARAM    0x04
#define RUN_STOP      0x05
#define GET_DIFF_POS  0x06
#define RESET         0x07
#define SPEED_TIME    0x08
#define GET_SPEED     0x09
#define GET_POS       0x10
#define MOVE          0x11
#define MOVE_TO       0x12
#define DEBUG_STR     0xCC
#define ACKNOWLEDGE   0xFF


void PackageHandler(uint8_t * data, uint32_t length);
