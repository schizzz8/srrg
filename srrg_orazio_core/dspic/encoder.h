#pragma once
#include <stdint.h>
#include "packets.h"

//! quadrature encoder class
//! handles quadrature encoder transition and sampling
//! latches to an external register than is the hardware QEI

class Encoder {
public:
  typedef int16_t ScalarType;
  
  Encoder();

  void setPositionRegister(uint16_t* position_register){
    _position_register=position_register;
  }
  
  //! zeroes the encoder counts
  inline void clearCounters(){
    if (_position_register)
      *_position_register = 0;
    _sampled_position = 0;
    _previous_sampled_position = 0;
  }
  

  //! copies the current encoder values in member variables (used to guarantee synchronous sampling)
  inline void sample(){
    _previous_sampled_position=_sampled_position;
    if (_position_register)
      _sampled_position=*_position_register;
  }

  //! returns the current (immeidate) position of the encoder
  inline const ScalarType position() const {
    if (_position_register)
      return *_position_register;
    return 0;
  }
  
  //! returns the position stored at the last sample() call 
  inline const ScalarType sampledPosition() const {return _sampled_position;}
  
  //! returns the difference between the two last sampled positions
  //! useful to complite the velocity
  inline const ScalarType sampledDelta() const {return _sampled_position-_previous_sampled_position;}
protected:
  ScalarType _sampled_position, _previous_sampled_position;
  uint16_t* _position_register;
};

//! encoder global setup
extern Encoder encoders[num_motors];
void initEncoders();
