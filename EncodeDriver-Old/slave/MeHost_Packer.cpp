#include "MeHost_Packer.h"

//  function:       pack data into a package to send
//  param:  buf     buffer to save package
//          bufSize size of buf
//          module  the associated module of package
//          data    the data to pack
//          length  the length(size) of data
//  return: 0       error
//          other   package size
uint32_t MeHost_Pack(uint8_t * buf,
                     uint32_t bufSize, 
                     uint8_t module, 
                     uint8_t * data, 
                     uint32_t length)
{
    uint32_t i = 0;

    //  head: 0xA5
    buf[i++] = 0xA5;
    buf[i++] = module;
    //  pack length
    buf[i++] = *((uint8_t *)&length + 0);
    buf[i++] = *((uint8_t *)&length + 1);
    buf[i++] = *((uint8_t *)&length + 2);
    buf[i++] = *((uint8_t *)&length + 3);
    //  pack data
    for(uint32_t j = 0; j < length; ++j)
    {
        buf[i++] = data[j];
    }

    //  calculate the LRC
    uint8_t check = 0x00;
    for(uint32_t j = 0; j < length; ++j)
    {
        check ^= data[j];
    }
    buf[i++] = check;

    //  tail: 0x5A
    buf[i++] = 0x5A;

    if (i > bufSize)
    {
        return 0;
    }
    else
    {
        return i;
    }
}
