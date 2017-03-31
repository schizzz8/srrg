#pragma once
#include <stdint.h>
#include "dspic_machine_config.h"

#ifdef __cplusplus
extern "C" {
#endif


  void serialInit();
  char writeByte(uint8_t);
  char writeBuffer(const uint8_t* buffer, int size);
  int writeBufferSize();
  void flushBuffer();

  uint16_t readBufferSize();
  uint8_t readByte();

#ifdef __cplusplus
} // extern "C" 
#endif
  


