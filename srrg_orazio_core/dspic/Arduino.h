#pragma once
#include <stdint.h>

#define OUTPUT 0
#define INPUT 2
#define LOW 0
#define HIGH 1
#define A0 0
#define A1 1

class BaseUART{
public:
  BaseUART();
  void begin(uint32_t baudrate);
  uint16_t available();
  void write(uint8_t value);
  uint8_t read();
  void flush();
protected:
  bool initialized;
};


uint8_t  digitalRead(uint16_t channel);
void     digitalWrite(uint16_t channel, uint8_t value);
uint16_t analogRead(uint16_t channel);
void     analogWrite(uint16_t channel, uint8_t value);
void     pinMode(uint16_t pin, uint8_t mode);

extern BaseUART Serial;

void initArduinoEmulator();
