#pragma once
#include <stdint.h>

class FakeEEPROM{
public:
  void init();
  char read(uint16_t address);
  char write(uint16_t address, char c);
protected:
  char buffer[256];
};

extern FakeEEPROM EEPROM;


