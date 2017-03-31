#include "Arduino.h"
#include "globals.h"
#include "comm_globals.h"
#include <string.h>
#include "eeprom.h"

/****************** COMM  SETUP AND GLOBALS *******************/
using namespace srrg_orazio_core;

PacketEncoder packet_encoder;
PacketDecoder packet_decoder;

inline void putResponse(Packet* input_packet, uint8_t error_code) {
  ResponsePacket response;
  response.id=ResponsePacket::default_id;
  response.src_id=input_packet->id;
  response.seq=input_packet->seq;
  response.error_code=error_code;
  packet_encoder.putPacket(response);
  system_status.rx_packets++;
} 

typedef bool (*PacketHandlerFn)(Packet*);

PacketHandlerFn packet_handlers[max_packet_id];

bool handleUnknownPacket(Packet* packet) {
  putResponse(packet, ResponsePacket::NoHandler);
  return false;
}

bool handleParamControlPacket(Packet* packet_){
  if (packet_->id!=ParamControlPacket::default_id)
    return false;
  ParamControlPacket* packet=reinterpret_cast<ParamControlPacket*>(packet_);
  if (packet->command==0){
    switch (packet->param_type){
    case 0:
      system_params.id=SystemParamPacket::default_id;
      packet_encoder.putPacket(system_params);
      putResponse(packet, ResponsePacket::Ok);
     break;
    case 1:
      joints_params.id=JointParamPacket::default_id;
      packet_encoder.putPacket(joints_params);
      putResponse(packet, ResponsePacket::Ok);
      break;
    case 2:
      kinematics_params.id=DifferentialDriveParamPacket::default_id;
      packet_encoder.putPacket(kinematics_params);
      putResponse(packet, ResponsePacket::Ok);
      break;
    default:
      putResponse(packet, ResponsePacket::NoHandler);
    }
  } else {
    if (packet->command==1){
      loadParamsFromEEPROM(packet->param_type);
      putResponse(packet, ResponsePacket::Ok);
      return true;
    }
    if (packet->command==2){
      writeParamsToEEPROM(packet->param_type);
      putResponse(packet, ResponsePacket::Ok);
      return true;
    }
  }
  return true;
}

bool handleSystemParamPacket(Packet* packet_){
  if (packet_->id!=SystemParamPacket::default_id)
    return false;
  SystemParamPacket* packet=reinterpret_cast<SystemParamPacket*>(packet_);
  system_params=*packet;
  system_params.seq=1;
  putResponse(packet, ResponsePacket::Ok);
  return true;
}


inline bool handleJointControlPacket(Packet* packet_){
  if (packet_->id!=JointControlPacket::default_id)
    return false;
  JointControlPacket* packet=reinterpret_cast<JointControlPacket*>(packet_);
  for (int i=0; i<num_motors; i++){
    JointInfo& info=joints_status.joints[i];
    const JointControl& control=packet->controls[i];
    switch (control.mode){
    case JointInfo::Disabled:
    case JointInfo::Brake:
      info.mode=control.mode;
      info.desired_speed=0;
      info.pwm=0;
      break;
    case JointInfo::PWM:
      info.mode=control.mode;
      info.pwm=control.speed;
      break;
    case JointInfo::PID:
      info.mode=control.mode;
      info.desired_speed=control.speed;
      break;
    }
  }
  return true;
}

bool handleJointsParamPacket(Packet* packet_){
  if (packet_->id!=JointParamPacket::default_id)
    return false;
  JointParamPacket* packet=reinterpret_cast<JointParamPacket*>(packet_);
  joints_params=*packet;
  joints_params.seq=1;
  putResponse(packet, ResponsePacket::Ok);
  return true;
}


inline bool handleDifferentialDriveControlPacket(Packet* packet_) {
  if (packet_->id!=DifferentialDriveControlPacket::default_id)
    return false;
  DifferentialDriveControlPacket* packet=reinterpret_cast<DifferentialDriveControlPacket*>(packet_);
  uint8_t right_idx=kinematics_params.right_joint_index;
  uint8_t left_idx=kinematics_params.left_joint_index;
  JointInfo& right_joint=joints_status.joints[right_idx];
  JointInfo& left_joint=joints_status.joints[left_idx];
  right_joint.mode=JointInfo::PID;
  left_joint.mode=JointInfo::PID;
  kinematics.setDesiredVelocities(packet->translational_velocity,
				  packet->rotational_velocity);
  // kinematics.velocity2ticks(right_joint.desired_speed,
  // 			    left_joint.desired_speed,
  // 			    system_params.timer_period);
  				  
  return true;
}


bool handleDifferentialDriveParamPacket(Packet* packet_){
  if (packet_->id!=DifferentialDriveParamPacket::default_id)
    return false;
  DifferentialDriveParamPacket* packet=reinterpret_cast<DifferentialDriveParamPacket*>(packet_);
  kinematics_params=*packet;
  kinematics_params.seq=1;
  updateKinematicsParams();
  putResponse(packet, ResponsePacket::Ok);
  return true;
}



bool handlePacket(){
  unsigned char* rxb=packet_decoder.readBufferStart();
  Packet* packet=reinterpret_cast<Packet*>(rxb);
  if (packet->id>=max_packet_id){
    putResponse(packet, ResponsePacket::NoHandler);
    return false;
  }
  bool result=(*packet_handlers[packet->id])(packet);
  if (! result){
    system_status.rx_packet_errors++;
  } else {
    system_status.rx_packets++;
    system_status.last_packet_id=packet->id;
  }
  return result;
}


bool handleCommunication() {
  bool packet_good=false;
  while(Serial.available()){
    int incoming_byte=Serial.read();
    bool packet_ready=packet_decoder.putChar(incoming_byte);
    if(packet_ready)
      packet_good|=handlePacket();
  }
  return packet_good;
}

void flushCommunication() {
  if(packet_encoder.bytesToSend())
    Serial.write(packet_encoder.getChar());
}

void initCommunication() {
  for (int i=0; i<max_packet_id; i++){
    packet_handlers[i]=&handleUnknownPacket;
  }
  packet_handlers[ParamControlPacket::default_id]=&handleParamControlPacket;
  packet_handlers[SystemParamPacket::default_id]=&handleSystemParamPacket;
  packet_handlers[JointControlPacket::default_id]=&handleJointControlPacket;
  packet_handlers[JointParamPacket::default_id]=&handleJointsParamPacket;
  packet_handlers[DifferentialDriveControlPacket::default_id]=&handleDifferentialDriveControlPacket;
  packet_handlers[DifferentialDriveParamPacket::default_id]=&handleDifferentialDriveParamPacket;
  
  // call the constructor for global variables
  packet_decoder=PacketDecoder();
  packet_encoder=PacketEncoder();
  Serial.begin(system_params.comm_speed);
}
