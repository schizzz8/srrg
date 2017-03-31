#pragma once
#include <stdint.h>

namespace srrg_orazio_core {
  class DifferentialDriveKinematics{
  public:
    DifferentialDriveKinematics(float kr_=1e-3, float kl_=1e-3, float baseline_=0.3);
    void setBaseParams(float kr_, float kl_, float baseline_);
    void setVelocityParams(float max_translational_velocity, 
			   float max_translational_acceleration, 
			   float max_translational_brake, 
			   float max_rotational_velocity, 
			   float max_rotational_acceleration);
    inline float kl() const {return _kl;}
    inline float kr() const {return _kr;}
    inline float baseline() const {return _baseline;}
    inline float x() const {return _x;}
    inline float y() const {return _y;}
    inline float theta() const {return _theta;}
    inline float translationalVelocityMeasured() const {return _translational_velocity_measured;}
    inline float rotationalVelocityMeasured() const { return _rotational_velocity_measured;}
    
    inline float translationalVelocityAdjusted() const {return _translational_velocity_adjusted;}
    inline float rotationalVelocityAdjusted() const { return _rotational_velocity_adjusted;}
    inline float translationalVelocityDesired() const { return _translational_velocity_desired;}
    inline float rotationalVelocityDesired() const { return _rotational_velocity_desired;}

    inline void setDesiredVelocities(float des_tv, float des_rv){
      _rotational_velocity_desired=des_rv;
      _translational_velocity_desired=des_tv;
    }


    void update(uint16_t right_encoder, uint16_t left_encoder, float time_interval);
    void reset(float x_=0, float y_=0, float theta_=0);

    //! converts the velocity to the ticks, according to the current velocity profile
    //! the velocity is obtained by the member
    //! _*_velocity_desired in the class
    void velocity2ticks(int16_t& right_ticks, 
			int16_t& left_ticks,
			float time_interval);
  protected:
    float _previous_left_encoder, _previous_right_encoder;
    float _x, _y, _theta;
    float _kl, _kr, _baseline;
    float _max_translational_acceleration,
          _max_translational_brake,
          _max_translational_velocity;
    float _max_rotational_acceleration,
          _max_rotational_velocity;
    float _ikl, _ikr, _ibaseline;
    float _translational_velocity_measured, _rotational_velocity_measured;
    float _translational_velocity_adjusted, _rotational_velocity_adjusted;
    float _translational_velocity_desired, _rotational_velocity_desired;

  };

}
