#include "encoder.h"
#include "Arduino.h"

const char Encoder::_transition_table []=
  {
      0,  //0000
     -1, //0001
      1,  //0010
      0,  //0011
      1,  //0100
      0,  //0101
      0,  //0110
     -1, //0111
     -1, //1000
      0,  //1001
      0,  //1010
      1,  //1011
      0,  //1100
      1,  //1101
     -1,  //1110
      0   //1111
    };


/****************** ENCODER  SETUP AND GLOBALS *******************/
Encoder encoders[num_motors];

// last 4 bits of port b used for encoders1
static const  char encoder_mask=0xF; 

// encoder interrupt
ISR(PCINT0_vect) {
  cli();
  char port_value=PINB&encoder_mask;
  encoders[0].handleTransition(port_value&0x3);
  encoders[1].handleTransition(port_value>>2);
  sei();
}

void initEncoders(){
  cli();
  DDRB &= ~encoder_mask; //set encoder pins as input
  PORTB |= encoder_mask; //enable pull up resistors
  PCICR |= (1 << PCIE0); // set interrupt on change, looking up PCMSK0
  PCMSK0 |= encoder_mask;   // set PCINT0 to trigger an interrupt on state change 
  sei();                     // turn on interrupts
}
