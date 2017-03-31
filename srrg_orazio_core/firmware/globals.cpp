#include "Arduino.h"
#include "globals.h"

/****************** MOTOR  SETUP AND GLOBALS *******************/

Motor motors[num_motors];

/****************** SYSTEM STATUS *******************/
SystemParamPacket system_params;
SystemStatusPacket system_status;

void initSystemParams() {
  system_params=SystemParamPacket();
  system_params.seq=0;
  system_status=SystemStatusPacket();
}

void updateSystemStatus(){
  system_status.id=SystemStatusPacket::default_id;
  system_status.seq++;
}


/****************** GLOBAL STATUS *******************/
JointController joint_controllers[num_motors];
JointStatusPacket joints_status;
JointParamPacket joints_params;

void updateJointsStatus(){
  joints_status.id=JointStatusPacket::default_id;
  for (int i=0; i<num_motors; i++){
    JointInfo& joint=joints_status.joints[i];
    joint.encoder_position=encoders[i].sampledPosition();
    joint.encoder_speed=encoders[i].sampledDelta();
    //joint.sensed_current=motors[i].currentSense();
    joint.pwm=motors[i].pwmSpeed();
  }
  joints_status.seq=system_status.seq;
}


/****************** CONFIGURATION *******************/

void initJoints(){
  joints_params=JointParamPacket();
  joints_params.seq=0;
  joints_status=JointStatusPacket();
  //loadParamsFromEEPROM();
  updateJointsStatus();
  
}

void updateJointsParams() {
  for (int i=0; i<num_motors; i++) {
    joint_controllers[i].setParams(joints_params.params[i]);
    motors[i].setMaxPWM(joints_params.params[i].max_pwm);
  }
}

/****************** KINEMATICS *******************/
DifferentialDriveKinematics kinematics;
DifferentialDriveParamPacket kinematics_params;
DifferentialDriveStatusPacket kinematics_status;

void initKinematics(){
  kinematics_params=DifferentialDriveParamPacket();
  updateKinematicsParams();
  kinematics_status=DifferentialDriveStatusPacket();
}

void updateKinematicsParams(){
  kinematics.setBaseParams(kinematics_params.kr, kinematics_params.kl, kinematics_params.baseline);
  kinematics.setVelocityParams(kinematics_params.max_translational_velocity,
			       kinematics_params.max_translational_acceleration,
			       kinematics_params.max_translational_brake,
			       kinematics_params.max_rotational_velocity,
			       kinematics_params.max_rotational_acceleration);

}

void updateKinematicsStatus(){
  kinematics.update(encoders[kinematics_params.right_joint_index].sampledPosition(),
   		    encoders[kinematics_params.left_joint_index].sampledPosition(),
   		    system_params.timer_period*system_params.comm_cycles);
  kinematics_status.id=DifferentialDriveStatusPacket::default_id;
  kinematics_status.seq=system_status.seq;
  kinematics_status.odom_x=kinematics.x();
  kinematics_status.odom_y=kinematics.y();
  kinematics_status.odom_theta=kinematics.theta();
  kinematics_status.translational_velocity_measured=kinematics.translationalVelocityMeasured();
  kinematics_status.rotational_velocity_measured=kinematics.rotationalVelocityMeasured();
  kinematics_status.translational_velocity_adjusted=kinematics.translationalVelocityAdjusted();
  kinematics_status.rotational_velocity_adjusted=kinematics.rotationalVelocityAdjusted();
  kinematics_status.translational_velocity_desired=kinematics.translationalVelocityDesired();
  kinematics_status.rotational_velocity_desired=kinematics.rotationalVelocityDesired();
}

/****************** JOINT CONTROL *****************/

void handleJointsISR(){
  bool do_reset=false;
  if (system_params.watchdog_cycles>0){
      system_status.watchdog_count--;
      if(system_status.watchdog_count<0){
	system_status.watchdog_count=0;
	do_reset=true;
	kinematics.setDesiredVelocities(0.0f, 0.0f);
      }
  }
  for (int i=0; i<num_motors; i++) {
    Encoder& encoder=encoders[i];
    encoder.sample();
    JointInfo& info=joints_status.joints[i];
    Motor& motor=motors[i];
    JointController& controller=joint_controllers[i];
    if (do_reset){
      info.mode=JointInfo::Disabled;
    }
    switch(info.mode) {
    case JointInfo::Disabled:
      motor.setStatus(Motor::Disabled);
      info.desired_speed=0;
      info.pwm=0;
      controller.reset();
      break;
    case JointInfo::Brake:
      motor.setStatus(Motor::Brake);
      info.desired_speed=0;
      info.pwm=0;
      controller.reset();
      break;
    case JointInfo::PWM:
      controller.reset();
      motor.setStatus(Motor::Enabled);
      break;
    case JointInfo::PID:
      motor.setStatus(Motor::Enabled);
      controller.compute(encoder.sampledDelta(), info.desired_speed);
      info.pwm=controller.output();
      break;
    }
    motor.setPWMSpeed(info.pwm);
    motor.update();
  }
}

/******* Differential drive velocity control **********/

void handleBaseVelocities() {
  int16_t left_joint_desired_speed;
  int16_t right_joint_desired_speed;
  kinematics.velocity2ticks(right_joint_desired_speed,
			    left_joint_desired_speed,
			    system_params.timer_period);
  uint8_t right_idx=kinematics_params.right_joint_index;
  uint8_t left_idx=kinematics_params.left_joint_index;
  JointInfo& right_joint=joints_status.joints[right_idx];
  JointInfo& left_joint=joints_status.joints[left_idx];
  right_joint.mode=JointInfo::PID;
  left_joint.mode=JointInfo::PID;
  right_joint.desired_speed=right_joint_desired_speed;
  left_joint.desired_speed=left_joint_desired_speed;
}
