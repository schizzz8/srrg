#include "joint_controller.h"

namespace srrg_orazio_core {

  JointController::JointController(){
    _output=_measure=_ramp_reference=_reference=0;
    _error_integral=_error=_previous_error=0;
  } 


  inline int16_t clamp(int16_t v, int16_t value){
    if (v>value)
      return value;
    if (v<-value)
      return -value;
    return v;
  }

 
  void JointController::compute(int16_t measure, int16_t reference) {
    _measure=measure;
    _reference=reference;
    // clamp the reference to the maximum value
    _reference=clamp(_reference, _params.max_speed);

    // ramp
    _error=clamp(_reference-_measure, _params.slope);
    _ramp_reference=measure+_error;
 
    // pid
    _error_integral+=_error;
    _error_integral=clamp(_error_integral,_params.max_i);
    int16_t d_error=_error-_previous_error;
    _output=_ramp_reference+(_params.kp*_error+_params.ki*_error_integral+_params.kd*d_error)/32;
    
    _output=clamp(_output, _params.max_pwm);
    if (_output<_params.min_pwm && _output>-_params.min_pwm)
      _output=0;
    
    _previous_error=_error;
  }
}
