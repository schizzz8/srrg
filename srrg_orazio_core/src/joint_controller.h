#pragma once
#include <stdint.h>
#pragma pack(push, 1)

namespace srrg_orazio_core {

  //struct containing the information of a joint
  //A joint can be un a specidic mode (see the mode field)
  struct JointInfo{
    enum Mode {Disabled=0x0, Brake=0x1, PWM=0x2, PID=0x3}; // same as in the motor class, plus control
    JointInfo() {
      mode=0;
      encoder_position=0;
      encoder_speed=0;
      desired_speed=0;
      pwm=0;
      sensed_current=0;
    }
    uint8_t mode; 
    uint16_t encoder_position; // absolute position
    int16_t  encoder_speed;    // difference in position between two ticks
    int16_t  desired_speed;    // speed set from pc (ignored if mode ==PWM)
    int16_t pwm;               // pwm value
    int16_t sensed_current;    // curret sampled from the uc
  };

  //! parameters of a joint controller
  //! a joint controller is a PID, with ramp on input
  //! saturation on output and anti-windup
  struct JointParams{
    JointParams() {
      kp=256;
      ki=64;
      kd=0;
      max_i=200;
      max_pwm=254;
      min_pwm=20;
      max_speed=100;
      slope=10;
    }
    int16_t kp, ki, kd;       // pid parameters*256
    int16_t max_i;            // max value of the integral term in PID
    int16_t min_pwm, max_pwm; //< minimum and maximum magnitude of values that can be sent as output
    //< values whose norm is higher than max_pwm will be clamped
    //< values whose notm is lower than min_pwm will be zeroed
    int16_t max_speed;        //< max_input value (encoder ticks)
    int16_t slope;            //< max slope for ramp between two cycles (encoder ticks)
  };

  struct JointControl{
    JointControl(){
      mode=0xff;
      speed=0;
    }
    uint8_t mode;
    int16_t speed;
  };

#pragma pack(pop)

  /**
     Implementation of a PID controller + ramp on input + clamping on input and output.
     If this sounds weird have a look as the cpp file and see the compute
   
     To use it:
     1. Instantiate one object of this class
     2. set the parameters (See packets.h::JointParams)
     3. measure the system output
     4. set your desired output
     5. call controller.compute(measure,desired)
     6. feed your system with the controller output().
  */
  class  JointController { 
  public:
    JointController();

    inline   int16_t output() const { return _output;} //< controller output
    inline void setParams(const JointParams& params_) { _params = params_;} //< set joint parameters
    inline const JointParams& params() const {return _params;} //< get  joint params

    // calls the control and makes side effect on the _output.
    void compute(int16_t measure, int16_t reference);
    //
    inline void reset() {
      _error=0;
      _error_integral=0;
      _previous_error=0;
    }
  
  protected:
    JointParams _params;
    int16_t _output, _measure, _ramp_reference, _reference;
    int16_t _error_integral, _error, _previous_error;
  };

}
