#include "motor.h"
#include "Arduino.h"

Motor::Motor(int pwm1_pin, int dir_pin, int brake_pin, int sense_pin, uint8_t  m) {
  init(pwm1_pin, dir_pin, brake_pin, sense_pin, m);
  _status=Disabled;
  _max_pwm=255;
  _mode = 0;
}

Motor::Motor(){
  _pwm1_pin=-1;
  _pwm2_pin=-1;
  _brake_pin=-1;
  _sense_pin=-1;
  _max_pwm=255;
}

int16_t Motor::currentSense() const { 
  return analogRead(_sense_pin); 
} 
  
void Motor::init(int pwm1_pin, int pwm2_pin, int brake_pin, int sense_pin, uint8_t m) {
  _mode=m;
  _pwm1_pin=pwm1_pin;
  _pwm2_pin=pwm2_pin;
  _brake_pin=brake_pin;
  _sense_pin=sense_pin;
  _pwm_speed=0;
  _status=Disabled;

  if (_pwm1_pin<0)
    pinMode(_pwm1_pin, INPUT);
  else
  pinMode(_pwm1_pin, OUTPUT);
  
  if (_pwm2_pin<0)
    pinMode(_pwm2_pin, INPUT);
  else
    pinMode(_pwm2_pin, OUTPUT);
  
  if (_brake_pin<0)
    pinMode(_brake_pin, INPUT);
  else
  pinMode(_brake_pin, OUTPUT);

  if (_sense_pin<0)
    pinMode(_sense_pin, INPUT);
  else
    pinMode(_sense_pin, OUTPUT);
  
  update();
}

void Motor::update() {
  switch(_status){
  case Disabled: 
    _pwm_speed=0;
    switch(_mode) {
    case 0 /*DirAndPWM*/:
      analogWrite(_pwm1_pin, 0);
      digitalWrite(_brake_pin, LOW);
      digitalWrite(_pwm2_pin, LOW);
      break;
    case 1 /*DirAndPWM*/:
      analogWrite(_pwm1_pin, 0);
      analogWrite(_pwm2_pin, 0);
      digitalWrite(_brake_pin, LOW);
      break;
    /*case 2 :
      analogWrite(_pwm1_pin, 0);
      digitalWrite(_brake_pin, LOW);
      digitalWrite(_pwm2_pin, LOW);
      break;*/
    }
  case Brake:
    _pwm_speed=0;
    switch(_mode) {
    case 0 /*DirAndPWM*/:
      analogWrite(_pwm1_pin, 0);
      analogWrite(_pwm2_pin, 0);
      digitalWrite(_brake_pin, HIGH);
      break;
    case 1 /*DirAndPWM*/:
      analogWrite(_pwm1_pin, 0);
      analogWrite(_pwm2_pin, 0);
      digitalWrite(_brake_pin, HIGH);
      break;
    /*case 2 :
      analogWrite(_pwm1_pin, 0);
      analogWrite(_pwm2_pin, 0);
      digitalWrite(_brake_pin, HIGH);
      break;*/
    }
  case Enabled:
    uint8_t pwm=0;
    switch(_mode) {
	    case 0 /*DirAndPWM*/:
	      digitalWrite(_brake_pin, LOW);
	      if (_pwm_speed>0) {
			pwm=_pwm_speed;
			digitalWrite(_pwm2_pin, LOW);
	      } else {
			pwm=-_pwm_speed;
			digitalWrite(_pwm2_pin, HIGH);
	      }
	      analogWrite(_pwm1_pin, pwm);
	    break;

	    case 1/*PWMPlusMinus*/:
	      digitalWrite(_brake_pin, HIGH);
	      if (_pwm_speed>0) {
			pwm=_pwm_speed;
			digitalWrite(_pwm2_pin, LOW);
			analogWrite(_pwm1_pin, pwm);
	      } else {
			pwm=-_pwm_speed;
			digitalWrite(_pwm1_pin, LOW);
			analogWrite(_pwm2_pin, pwm);
	      }
	      break;

	    /*case 2 :
	      digitalWrite(_brake_pin, LOW);
	      pwm=_pwm_speed;
	      analogWrite(_pwm1_pin, pwm);
			
	      break;*/
    }
    break;
  }
}


