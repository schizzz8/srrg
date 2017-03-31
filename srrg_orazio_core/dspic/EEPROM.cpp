#include "EEPROM.h"

void FakeEEPROM::init() {
  for (int i=0; i<256; i++)
    buffer[i]=0;
}

char FakeEEPROM::read(uint16_t address) {
  return buffer[address];
}

char FakeEEPROM::write(uint16_t address, char c){
  buffer[address]=c;
}

FakeEEPROM EEPROM;
