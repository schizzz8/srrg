#include "orazio_robot_connection.h"
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include "serial.h"
#include <iostream>
#include <cassert>
using namespace std;

namespace srrg_orazio_core{
  OrazioRobotConnection::OrazioRobotConnection(){
    _seq=1;
    _fd=0;
    _post_joint_control=false;
    _packet_retries=10; 
    pthread_mutex_init(&_comm_mutex, 0);
  }

  OrazioRobotConnection::~OrazioRobotConnection(){
    pthread_mutex_destroy(&_comm_mutex);
    disconnect();
  }

  void OrazioRobotConnection::connect(const char* device){
    if(_fd){
      disconnect();
    }
    _fd=serial_open(device);
    if (_fd<0){
      _fd=0;
      return;
    }
    serial_set_interface_attribs(_fd, B115200, 0);
    serial_set_blocking(_fd,true);
    _packet_count=0;
    _packet_errors=0;
    for (int i=0; i< _packet_retries; i++)
      spinOnce();
  }

  void OrazioRobotConnection::disconnect(){
    if (_fd)
      close(_fd);
    _fd=0;
  }

  void OrazioRobotConnection::jointPositions(int16_t* ticks) const{
    for (int i=0; i<numJoints(); i++){
      ticks[i]=_joint_status.joints[i].encoder_position;
    }
  }

  void OrazioRobotConnection::jointSpeeds(int16_t* joint_speeds) const {
    for (int i=0; i<numJoints(); i++){
      joint_speeds[i]=_joint_status.joints[i].encoder_speed;
    }
  }

  void OrazioRobotConnection::jointPWM(int16_t* joint_pwms) const {
    for (int i=0; i<numJoints(); i++){
      joint_pwms[i]=_joint_status.joints[i].pwm;
    }
  }

  void OrazioRobotConnection::setJointSpeeds(const int16_t* joint_speeds) {
    JointControlPacket control_packet;
    _joint_control.seq=_seq;
    _seq++;
    for (int i=0; i<numJoints(); i++){
      _joint_control.controls[i].mode=JointInfo::PID;
      _joint_control.controls[i].speed=joint_speeds[i];
    }
    _packet_encoder.putPacket(_joint_control);
  }

  void OrazioRobotConnection::setJointPWM(const int16_t* joint_pwms) {
    JointControlPacket control_packet;
    _joint_control.seq=_seq;
    _seq++;
    for (int i=0; i<numJoints(); i++){
      _joint_control.controls[i].mode=JointInfo::PWM;
      _joint_control.controls[i].speed=joint_pwms[i];
    }
    _packet_encoder.putPacket(_joint_control);
  }

  bool OrazioRobotConnection::queryParams(uint8_t param_type){
    return controlParams(0,param_type,_packet_retries);
  }


  bool OrazioRobotConnection::loadParams(uint8_t param_type){
    return controlParams(1, param_type, _packet_retries);
  }

  bool OrazioRobotConnection::saveParams(uint8_t param_type){
    return controlParams(2, param_type, _packet_retries);
  }

  bool OrazioRobotConnection::pushParams(uint8_t param_type){
    return controlParams(3, param_type, _packet_retries);
  }

  bool OrazioRobotConnection::controlParams(uint8_t command, uint8_t param_type, int retries){
    _seq++;
    uint16_t current_seq=_seq;
    if (command<3){
      ParamControlPacket param_control;
      param_control.seq=_seq;
      param_control.param_type=param_type;
      param_control.command=command;
      _packet_encoder.putPacket(param_control);
    } else {
      switch(param_type){
      case 0:
	_system_params.seq=current_seq;
	_packet_encoder.putPacket(_system_params); 
	break;
      case 1: 
	_joint_params.seq=current_seq;
	_packet_encoder.putPacket(_joint_params); 
	break;
      case 2: 
	_kinematics_params.seq=current_seq;
	_packet_encoder.putPacket(_kinematics_params); 
	break;
      }
    }
    if (retries==0)
      return true;
    int num_retries=0;
    while(num_retries<retries){
      spinOnce();
      if (_last_response.seq==current_seq){
	return _last_response.error_code==ResponsePacket::Ok;
      }
      num_retries++;
    }
    return false;
  } 

  bool OrazioRobotConnection::controlJoint(uint8_t index, JointInfo::Mode mode, int16_t speed){
    if(index>=num_motors){
      _post_joint_control=false;
      return false;
    }
    _joint_control.controls[index].mode=mode;
    _joint_control.controls[index].speed=speed;
    _post_joint_control=true;
    return true;
  } 
  
  void OrazioRobotConnection::setBaseVelocities(float tv, float rv){
    _post_base_velocities=true;
    _base_velocities.seq=_seq++;
    _base_velocities.translational_velocity=tv;
    _base_velocities.rotational_velocity=rv;
  }
  
  void OrazioRobotConnection::spinOnce(){
    if (! _fd)
      return;

    pthread_mutex_lock(&_comm_mutex);
    if (_post_joint_control) {
      _packet_encoder.putPacket(_joint_control);
      for (int i=0; i<num_motors; i++){
	_joint_control.controls[i].mode=0xff;
	_joint_control.controls[i].speed=0;
      }
      _post_joint_control=false;
    } 

    if (_post_base_velocities) {
      _packet_encoder.putPacket(_base_velocities);
      _post_base_velocities=false;
    }
    while(_fd && _packet_encoder.bytesToSend()){
      unsigned char c=_packet_encoder.getChar();
      int k=write(_fd,&c,1);
      if (!k)
	cerr << "write failure" << endl;
    }
  
    bool packet_complete=false;
    
    do{
      char c;
      int n = read (_fd, &c, 1);
      if (n)
	packet_complete=_packet_decoder.putChar(c);
    } while (! packet_complete);
  
    if (packet_complete){
      _packet_count++;
      unsigned char* rxb=_packet_decoder.readBufferStart();
      Packet* packet=reinterpret_cast<Packet*>(rxb);
      switch(packet->id) {
      case SystemStatusPacket::default_id:
	_system_status = *reinterpret_cast<SystemStatusPacket*>(packet);
	break;
      case SystemParamPacket::default_id:
	_system_params = *reinterpret_cast<SystemParamPacket*>(packet);
	break;
      case JointStatusPacket::default_id:
	_joint_status = *reinterpret_cast<JointStatusPacket*>(packet);
	break;
      case JointParamPacket::default_id:
	_joint_params = *reinterpret_cast<JointParamPacket*>(packet);
	break;
      case DifferentialDriveStatusPacket::default_id:
	_kinematics_status= *reinterpret_cast<DifferentialDriveStatusPacket*>(packet);
	break;
      case DifferentialDriveParamPacket::default_id:
	_kinematics_params= *reinterpret_cast<DifferentialDriveParamPacket*>(packet);
	break;
      case ResponsePacket::default_id:
	_last_response= *reinterpret_cast<ResponsePacket*>(packet);
	{
	  size_t l=_packet_decoder.readBufferEnd()-_packet_decoder.readBufferStart();
	
	  if(l!=sizeof(ResponsePacket)){
	    cerr << "packet->id:" << (int) packet->id << endl;
	    cerr << "size: " << l << " exp_size: " << sizeof(ResponsePacket) << endl;
	    assert(0);
	  }
	
	}
	break;
      default:
	_packet_errors++;
      }
      pthread_mutex_unlock(&_comm_mutex);
    }

  }
}
