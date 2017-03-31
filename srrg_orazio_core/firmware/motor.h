#pragma once
#include <stdint.h>
#include "packets.h"

//! motor class. Handles a PWM channel with break
//! good for arduino motor shield
//! a motor can be in three states:
//! Disabled: (no current)
//! Brake:    the H bridge is active and actively breaks the motor
//! Enabled:  the motor is spinning, at a speed (roughly) proportional to the PWM duty cycle

class Motor{
public:
  enum Status {Disabled=0x0, Brake=0x1, Enabled=0x2};
  //! ctor, instantiates a motor and sets the arduino pins as the one passed as arguments
  Motor(int pwm1_pin, int pwm2_pin, int brake_pin, int sense_pin, uint8_t mode=0);
  //! ctor, instantiates a detached motor
  Motor();

  //! attaches a motor to the pins passed as input
  //! mode=0: DirAndPWM, mode=1 PWMPlusMinus
  void init(int pwm1_pin, int pwm2_pin, int brake_pin, int sense_pin, uint8_t mode=0);
  
  inline int pwm1Pin() const {return _pwm1_pin;} //< @returns the pwm pin
  inline int pwm2Pin() const {return _pwm2_pin;} //< @returns the direction pin or the pwm of neg channel
  inline int brakePin() const  {return _brake_pin;} //< @returns the brake pin
  inline int sensePin() const {return _sense_pin;}  //< @returns the current sense pin
  inline int16_t maxPWM() const {return _max_pwm;} //< max pwm duty cicle. controls are clamped to this value;}
  inline void setMaxPWM(int16_t max_pwm) {_max_pwm=max_pwm;} //< sets max pwm duty cycle
  
  Status status() const {return _status;} //< returns the motor status (Enable, Brake od Disable)
  void setStatus(Status s) {_status=s;}   //< sets the motor status

  inline int16_t pwmSpeed() const {return _pwm_speed;} //< @returns the pwm speed set

  //! sets the pwm speed
  //! the speed is not sent to the timer (and thus to the motor) until you call update
  //! yet this choice is to guarantee things to happen synchronously
  inline void setPWMSpeed(int16_t pwm_speed) {
    _pwm_speed=pwm_speed;
    _pwm_speed=(_pwm_speed<_max_pwm)?_pwm_speed:_max_pwm;
    _pwm_speed=(_pwm_speed>-_max_pwm)?_pwm_speed:-_max_pwm;
  } 

  //!how much is my motor draining?
  //!use with care as it calls a time consuming adc sampling
  int16_t currentSense() const; 

  //! flush all values to the hardware
  void update();
  inline uint8_t mode() {return _mode;} 
protected:
  uint8_t _mode;
  int _pwm1_pin;
  int _pwm2_pin; // direction if mode is DirAndPWM, pwm of negative channel otherwise
  int _brake_pin;
  int _sense_pin;
  Status _status;
  int16_t _pwm_speed;
  int16_t _max_pwm;
};

