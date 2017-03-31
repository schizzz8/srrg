#include "differential_drive_kinematics.h"
#include <math.h>
const float EPSILON=1e-6;

#ifdef _DSPIC_
const float M_PI=3.141592654;
#endif
namespace srrg_orazio_core {

  DifferentialDriveKinematics::DifferentialDriveKinematics(float kr_, float kl_, float baseline_){
    _previous_right_encoder=0;
    _previous_left_encoder=0;
    setBaseParams(kr_, kl_, baseline_);
    setVelocityParams(1.0,  //max_translational_velocity, 
		      0.5, // max_translational_acceleration, 
		      1.0,  // max_translational_brake, 
		      0.5,  //max_rotational_velocity, 
		      0.5  // max_rotatonal_acceleration);
		      );

    reset();
  }

  void DifferentialDriveKinematics::setBaseParams(float kr_, float kl_, float baseline_){
    _kr=kr_;
    _kl=kl_;
    _baseline=baseline_;
    _ikl=1./_kl;
    _ikr=1./_kr;
    _ibaseline=1./_baseline;
  }

  inline float sinThetaOverTheta(float theta) {
    if (fabs(theta)<EPSILON)
      return 1;
    return sin(theta)/theta;
  }

  inline float oneMinisCosThetaOverTheta(float theta) {
    if (fabs(theta)<EPSILON)
      return 0;
    return (1.0f-cos(theta))/theta;
  }

#define _USE_TAYLOR_EXPANSION_

  const float cos_coeffs[]={0., 0.5 ,  0.    ,   -1.0/24.0,   0.    , 1.0/720.0};
  const float sin_coeffs[]={1., 0.  , -1./6. ,      0.    ,   1./120, 0.   };

  void computeThetaTerms(float& sin_theta_over_theta,
			 float& one_minus_cos_theta_over_theta,
			 float theta) {
#ifdef _USE_TAYLOR_EXPANSION_
    // evaluates the taylor expansion of sin(x)/x and (1-cos(x))/x,
    // where the linearization point is x=0, and the functions are evaluated
    // in x=theta
    sin_theta_over_theta=0;
    one_minus_cos_theta_over_theta=0;
    float theta_acc=1;
    for (uint8_t i=0; i<6; i++) {
      if (i&0x1)
	one_minus_cos_theta_over_theta+=theta_acc*cos_coeffs[i];
      else 
	sin_theta_over_theta+=theta_acc*sin_coeffs[i];
      theta_acc*=theta;
    }
#else
    sin_theta_over_theta=sinThetaOverTheta(theta);
    one_minus_cos_theta_over_theta=oneMinisCosThetaOverTheta(theta) ;
#endif 
  }



  void DifferentialDriveKinematics::update(uint16_t right_encoder, uint16_t left_encoder, float time_interval){
    int16_t left_ticks=left_encoder-_previous_left_encoder;
    int16_t right_ticks=right_encoder-_previous_right_encoder;
    if (left_ticks==0 && right_ticks==0){
      _translational_velocity_measured=0;
      _rotational_velocity_measured=0;
    }
  
    // left and right motion of the wheels in meters
    float delta_l=_kl*left_ticks;
    float delta_r=_kr*right_ticks;
    _previous_right_encoder=right_encoder;
    _previous_left_encoder=left_encoder;

    // odometry update
    // the formulas below derive from the kinematic mnodel with instantaneous center of rotation
    // delta_theta = (right-left)/baseline = delta_minus/baseline;
    // R=(right+left)/delta_theta = delta_plus/theta [radius of curvature]
    // delta_x=R*sin(delta_theta)
    //        =delta_plus * [sin(delta_theta)/delta_theta];
    //        =delta_plus * [taylor expansion around 0 of sin(dth)/dth]. This avoids singularities
    // delta_y=R*[1-cos(delta_theta)]
    //        =delta_plus * [1-cos(delta_theta)]/delta_theta
    //	    =delta_plus * [taylor expansion around 0 of (1-cos(dth))/dth]. 
  
    float delta_plus=delta_r+delta_l;
    float delta_minus=delta_r-delta_l;
    float dth=delta_minus/_baseline;
    float one_minus_cos_theta_over_theta, sin_theta_over_theta;
    computeThetaTerms(sin_theta_over_theta, one_minus_cos_theta_over_theta, dth);
    float dx=.5*delta_plus*sin_theta_over_theta;
    float dy=.5*delta_plus*one_minus_cos_theta_over_theta;

    //apply the increment to the previous estimate
    float s=sin(_theta);
    float c=cos(_theta);
    _x+=c*dx-s*dy;
    _y+=s*dx+c*dy;
    _theta+=dth;
    // normallize theta;
    if (_theta>M_PI)
      _theta-=2*M_PI;
    else if (_theta<-M_PI)
      _theta+=2*M_PI;

    // velocity update
    _translational_velocity_measured=.5*delta_plus/time_interval;
    _rotational_velocity_measured=dth/time_interval;
  }

  void DifferentialDriveKinematics::reset(float x_, float y_, float theta_){
    _x=x_;
    _y=y_;
    _theta=theta_;
    _translational_velocity_measured=0.;
    _rotational_velocity_measured=0.;
    _translational_velocity_adjusted=0.;
    _rotational_velocity_adjusted=0.;
    _translational_velocity_desired=0.;
    _rotational_velocity_desired=0.;
  }

  void DifferentialDriveKinematics::setVelocityParams(float max_translational_velocity, 
						      float max_translational_acceleration, 
						      float max_translational_brake, 
						      float max_rotational_velocity, 
						      float max_rotational_acceleration){
    _max_translational_velocity=max_translational_velocity;
    _max_translational_acceleration=max_translational_acceleration;
    _max_translational_brake=max_translational_brake;
    _max_rotational_velocity=max_rotational_velocity;
    _max_rotational_acceleration=max_rotational_acceleration;
  }


  inline float clamp(const float& value, const float& threshold) {
    if (value>threshold)
      return threshold;
    if (value<-threshold)
      return -threshold;
    return value;
  }

  void DifferentialDriveKinematics::velocity2ticks(int16_t& right_ticks, int16_t& left_ticks,
						   float time_interval) {
  // compute the maximum increments according to the current time interval
    float max_tv_increment=_max_translational_acceleration*time_interval;
    float max_rv_increment=_max_rotational_acceleration*time_interval;
    float translational_velocity=_translational_velocity_desired;
    float rotational_velocity=_rotational_velocity_desired;

    // if desired  velocities set to 0, enable the breaking acceleration threshold
    if(_translational_velocity_desired==0.0f){
      max_tv_increment=_max_translational_brake*time_interval;
    }

    // clamp desired speed according to acceleration profile
    float dtv=translational_velocity-_translational_velocity_measured;
    dtv=clamp(dtv, max_tv_increment);
    translational_velocity=_translational_velocity_measured+dtv;

    // clamp according to max tv
    _translational_velocity_adjusted=clamp(translational_velocity,_max_translational_velocity);

    
    float drv=rotational_velocity-_rotational_velocity_measured;
    drv=clamp(drv, max_rv_increment);
    rotational_velocity= _rotational_velocity_measured+drv;

    // clamp according to max rv
    _rotational_velocity_adjusted=clamp(rotational_velocity, _max_rotational_velocity);

    // convert speed to distance sum/subtraction the _2 means the half of it
    float delta_plus_2  = translational_velocity*time_interval;
    float delta_minus_2 = .5*_baseline*rotational_velocity*time_interval;
    right_ticks = _ikr* (delta_plus_2+delta_minus_2);
    left_ticks  = _ikl* (delta_plus_2-delta_minus_2);
  }
  

}
