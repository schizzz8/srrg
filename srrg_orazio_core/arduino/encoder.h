#pragma once
#include <stdint.h>
#include <avr/interrupt.h>
#include "packets.h"

using namespace srrg_orazio_core;
//! quadrature encoder class
//! handles quadrature encoder transition and sampling
//! optimized for interrupts

class Encoder {
public:
  typedef int16_t ScalarType;
  
  Encoder() {
    _pin_state=0;
    clearCounters();
  }

  //! zeroes the encoder counts
  inline void clearCounters(){
    _position = 0;
    _sampled_position = 0;
    _previous_sampled_position = 0;
    _pin_state=0;
  }
  
  //! to be called woithin the ISR
  //! @ new_pin_state: the last 2 bits are the A and B channels of the encoder
  //! it updates the counters
  inline void handleTransition(uint8_t new_pin_state) {
    _position+=_transition_table[new_pin_state<<2|_pin_state];
    _pin_state=new_pin_state;
  }

  //! copies the current encoder values in member variables (used to guarantee synchronous sampling)
  inline void sample(){
    _previous_sampled_position=_sampled_position;
    cli();
    _sampled_position=_position;
    sei();
  }


  //! returns the current (immeidate) position of the encoder
  inline const ScalarType position() const {return _position;}
  
  //! returns the position stored at the last sample() call 
  inline const ScalarType sampledPosition() const {return _sampled_position;}
  
  //! returns the difference between the two last sampled positions
  //! useful to complite the velocity
  inline const ScalarType sampledDelta() const {return _sampled_position-_previous_sampled_position;}
protected:
  uint8_t _pin_state;
  ScalarType _position;
  ScalarType _sampled_position, _previous_sampled_position;

private:
  static const char _transition_table [];
};


//! encoder global setup
extern Encoder encoders[srrg_orazio_core::num_motors];
void initEncoders();
