#include "stdint.h"

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
                     uint32_t length);
