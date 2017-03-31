#pragma once
#include <stdint.h>
#include "joint_controller.h"
namespace srrg_orazio_core {

#pragma pack(push, 1)
  const uint8_t num_motors=2;
  const uint8_t max_packet_id=20;

  //base packet header
  struct Packet{
    uint8_t id;  //tells which packet
    uint16_t seq; // increasing number
  };

  // sent by the robot when something goes wrong
  // the seq of the error packet is set to the incoming packet that triggered the error
  struct ResponsePacket: public Packet{				
    enum ResponseResult {Ok=0x0, NoHandler=0x1};
    static const uint8_t default_id=0x00;
    ResponsePacket() {
      id=default_id;
      seq=0;
    }
    uint8_t src_id; // id of the input packet making mess
    uint8_t error_code; // error
  };

  //! sent from the pc to the robot causes
  //! the robot to send a ParamPacket to the PC (with the same seq)
  struct ParamControlPacket: public Packet{
    static const uint8_t default_id=0x1;

    //0: send current params
    //1: load params from eeprom and send them
    //2: write current params to eeprom, read them and send them
    uint8_t command;

    // identifies the parameter class on which command will be executed
    // 0: system
    // 1: joints
    // 2: platform
    uint8_t param_type;
    ParamControlPacket() {
      id=default_id;
      command=0;
      param_type=0;
    }
  };

  struct SystemStatusPacket:public Packet{
    static const uint8_t default_id=0x01;
    SystemStatusPacket(){
      id=default_id;
      rx_packets=0;
      tx_packets=0;
      rx_packet_errors=0;
      watchdog_count=0;
      battery_level=0;
      last_packet_id=0;
      idle_cycles=0;
    }
    uint16_t rx_packets;
    uint16_t tx_packets;
    uint16_t rx_packet_errors;
    uint16_t battery_level;
    int16_t watchdog_count;
    uint8_t num_motors;
    uint8_t last_packet_id;
    uint32_t idle_cycles;
  };

  struct SystemParamPacket: public Packet{
    static const uint8_t default_id=0x02;
    SystemParamPacket(){
      id=default_id;
      comm_speed=115200;
      comm_cycles=2;
      watchdog_cycles=50;
      timer_period=0.01; //100 hz
      motor_mode=0;
    }
    float timer_period;
    uint32_t comm_speed;
    uint16_t comm_cycles;
    uint16_t watchdog_cycles;
    uint8_t motor_mode; // 0: pwm_and_dir, 1: pwm+-
  };

  //packet periodically send from robot to pc
  // summarizes the status of the platform (joints, battery level, comm status and so on)
  struct JointStatusPacket: public Packet{
    static const uint8_t default_id=0x3;
    JointStatusPacket() {
      id=default_id;
    }
    JointInfo joints[num_motors];  // the joint states
  };


  struct JointControlPacket: public Packet{
    static const uint8_t default_id=0x4;
    JointControlPacket(){
      id=default_id;
    }
    JointControl controls[num_motors];
  };


  struct JointParamPacket: public Packet{
    static const uint8_t default_id=0x5;
    JointParamPacket(){
      id=default_id;
    }
    JointParams params[num_motors];
  };

  //! sent from the pc to the robot causes
  struct DifferentialDriveStatusPacket: public Packet{
    static const uint8_t default_id=0x6;
    DifferentialDriveStatusPacket() {
      id=DifferentialDriveStatusPacket::default_id;
      odom_x=odom_y=odom_theta=0;
      translational_velocity_measured=0;
      rotational_velocity_measured=0;
      translational_velocity_adjusted=0;
      rotational_velocity_adjusted=0;
      translational_velocity_desired=0;
      rotational_velocity_desired=0;
    }
    float odom_x, odom_y, odom_theta;
    float translational_velocity_measured;
    float rotational_velocity_measured;
    float translational_velocity_desired;
    float rotational_velocity_desired;
    float translational_velocity_adjusted;
    float rotational_velocity_adjusted;
    
  };

  //! sent from the pc to the robot causes
  struct DifferentialDriveControlPacket: public Packet{
    static const uint8_t default_id=0x7;
    DifferentialDriveControlPacket() {
      id=default_id;
      translational_velocity=0;
      rotational_velocity=0;
    }
    float translational_velocity;
    float rotational_velocity;
  };

  struct DifferentialDriveParamPacket: public Packet{
    static const uint8_t default_id=0x8;
    DifferentialDriveParamPacket(){
      id=default_id;
      baseline=0.3;
      kl=0.0001;
      kr=-0.0001;
      right_joint_index=1;
      left_joint_index=0;

      max_translational_velocity=1;
      max_translational_acceleration=1;
      max_translational_brake=3;
      max_rotational_velocity=2;
      max_rotational_acceleration=10;
    }

    float kr;
    float kl;
    float baseline;
    uint8_t right_joint_index;
    uint8_t left_joint_index;
    
    // ! new differential drive base parameters
    float max_translational_velocity;
    float max_translational_acceleration;
    float max_translational_brake;
    float max_rotational_velocity;
    float max_rotational_acceleration;
  };

  struct MonitorModePacket: public Packet{
    static const uint8_t default_id=0x9;
    MonitorModePacket(){
      id=default_id;
      enabled=0;
    }
    uint8_t enabled;
  };


#pragma pack(pop)

}
