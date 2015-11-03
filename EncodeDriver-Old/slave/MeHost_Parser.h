#ifndef _ME_HOST_PARSER_H_
#define _ME_HOST_PARSER_H_

#include "stdint.h"

#define BUF_SIZE            256
#define MASK                255

class MeHost_Parser
{
public:
    MeHost_Parser();
    ~MeHost_Parser();

    //  push data to buffer
    uint8_t PushStr(uint8_t * str, uint32_t length);
    uint8_t PushByte(uint8_t ch);
    //  run state machine
    uint8_t Run();
    //  get the package ready state
    uint8_t PackageReady();
    //  copy data to user's buffer
    uint8_t GetData(uint8_t *buf, uint32_t size);

    void Print(char *str, uint32_t * cnt);
private:
    int state;
    uint8_t buffer[BUF_SIZE];
    uint32_t in;
    uint32_t out;
    uint8_t packageReady;

    uint8_t module;
    uint32_t length;
    uint8_t *data;
    uint8_t check;

    uint32_t lengthRead;
    uint32_t currentDataPos;

    uint8_t GetByte(uint8_t * ch);
};

#endif
