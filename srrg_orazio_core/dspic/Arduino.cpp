#include "Arduino.h"

#ifdef __cplusplus 
extern "C" {
#endif

#include <p33Fxxxx.h>
#include <pwm12.h>
#include <uart.h>

#ifdef __cplusplus 
}
#endif

#include "dspic_settings.h"
#include "dspic_serial.h"


BaseUART::BaseUART() {initialized=false;}
void BaseUART::begin(uint32_t baudrate){
}

uint16_t BaseUART::available() {
  return readBufferSize();
}

void BaseUART::write(uint8_t value) {
  writeByte(value);
}

uint8_t BaseUART::read() {
  return readByte();
}

void BaseUART::flush() {
  flushBuffer();
}

uint8_t digitalRead(uint16_t channel) {return 0;}
void    digitalWrite(uint16_t channel, uint8_t value){}
uint16_t analogRead(uint16_t channel) {return 0;}
void    analogWrite(uint16_t channel, uint8_t value){}
void    pinMode(uint16_t pin, uint8_t mode) {}


BaseUART Serial;

/***************** PWM Layer, configures a timer and outputs to the pins *****/

/*
const int NUM_CHANNELS=2;

class PWMController {
public:
  struct Channel{
    Channel () {
      dutycycle=0;
      period=0;
    }
    uint16_t dutycycle;
    uint16_t period;
  };
  PWMController(uint8_t mode, uint16_t period);
  inline uint8_t numChannels() const {return NUM_CHANNELS;}
  int16_t period(uint8_t channel) const {return channels[channel].period;}
  uint16_t dutycycle(uint8_t channel) const {return channels[channel].dutycycle;}
  void setDutycycle(uint8_t channel, int16_t dutycycle);
protected:
  Channel channels[NUM_CHANNELS];
};

PWMController pwm_controller (0,1024);

//PWM configuration
PWMController::PWMController(uint8_t mode, uint16_t period)
{
    //TRISTATE SETTATI COME OUTPUT
    _TRISB0 = 0;
    _TRISB1 = 0;
    _TRISB4 = 0;
    _TRISB9 = 0;
    _TRISB12 = 0;
    _TRISB14 = 0;

    //PWM
    TRISBbits.TRISB12 = 0b0;
    TRISBbits.TRISB13 = 0b0;
    TRISBbits.TRISB14 = 0b0;
    TRISBbits.TRISB15 = 0b0;

    unsigned int sptime = 0x0;
    unsigned int config1 = PWM1_EN & PWM1_IDLE_CON & PWM1_OP_SCALE1 & PWM1_IPCLK_SCALE1 & PWM1_MOD_UPDN;
    unsigned int config2;
    //TANK 3A
    if(mode==0)
    config2 = PWM1_MOD1_IND & PWM1_PEN1L & PWM1_PDIS1H & PWM1_MOD2_IND & PWM1_PEN2L & PWM1_PDIS2H & PWM1_PDIS3H & PWM1_PDIS3L;

    //COMPLEMENTARY
    if(mode==1)
    config2 = PWM1_MOD1_COMP & PWM1_PEN1L & PWM1_PEN1H & PWM1_MOD2_COMP & PWM1_PEN2L & PWM1_PEN2H & PWM1_PDIS3H & PWM1_PDIS3L;

    unsigned int config3 = PWM1_SEVOPS1 & PWM1_OSYNC_PWM & PWM1_UEN;
    OpenMCPWM1(period, sptime, config1, config2, config3);
}


//set pwm dutycyle
void PWMController::setDutycycle(uint8_t numPwm, int16_t dutycycle){
    channels[numPwm].dutycycle=dutycycle;   
    SetDCMCPWM1(numPwm+1, dutycycle,0);
}

*/
void initArduinoEmulator(){
  Micro_init();
  //pwm_controller =  PWMController(0,1024);
  serialInit();
}

