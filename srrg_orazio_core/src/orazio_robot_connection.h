#pragma once
#include "packet_decoder.h"
#include "packet_encoder.h"
#include "packets.h"
#include <pthread.h>

namespace srrg_orazio_core {

  using namespace std;

  class OrazioRobotConnection{
  public:
    OrazioRobotConnection();
    ~OrazioRobotConnection();

    // high level interface, wrapped packets
    void connect(const char* device);
    void disconnect(); 
    inline bool isConnected() {return  _fd;}
    bool queryParams(uint8_t param_type); //< requests the params from the base (retries times) 
    bool pushParams(uint8_t param_type);  //< sends the current params to the base (retries times)
    bool loadParams(uint8_t param_type);  //< loads the params from eeprom (retries times)
    bool saveParams(uint8_t param_type);  //< saves current params to eeprom (retries times)

    // connection stats
    inline int packetCount() const {return _packet_count;}
    inline int packetErrors() const {return _packet_errors;}

    // control of the status packet
    inline float timerPeriod() const 
    { return _system_params.timer_period;} //< period of the inner timer
  
    inline bool setCommCycles(uint16_t cycles) 
    {_system_params.comm_cycles=cycles; return pushParams(0);}

    inline int commCycles() const 
    { return _system_params.comm_cycles;}
  
    inline int watchdogCycles() const 
    { return _system_params.watchdog_cycles;}
    
    inline bool setWatchdogCycles(uint16_t cycles) 
    {_system_params.watchdog_cycles=cycles; return pushParams(0);}


    inline bool setMotorMode(uint8_t motor_mode) 
    {_system_params.motor_mode=motor_mode; return pushParams(0);}


    inline uint8_t motorMode() const 
    {return _system_params.motor_mode;}

    inline uint16_t systemSeq() const 
    { return _system_status.seq; } //< id of the last system packet received
  
    inline uint16_t serverRxPackets() const 
    { return _system_status.rx_packets;}
  
    inline uint16_t serverTxPackets() const 
    { return _system_status.tx_packets;}
  
    inline uint16_t serverRxErrors() const  
    { return _system_status.rx_packet_errors;}

    //joint interface
    inline uint8_t numJoints() const 
    { return num_motors;}
  
    inline int16_t minPWM(uint8_t joint_num) const 
    { return _joint_params.params[joint_num].min_pwm;} 
  
    inline bool setMinPWM(uint8_t joint_num, int16_t min_pwm) 
    { _joint_params.params[joint_num].min_pwm=min_pwm; return pushParams(1);}

    inline int16_t  maxPWM(uint8_t joint_num) const 
    { return _joint_params.params[joint_num].max_pwm;}
  
    inline bool setMaxPWM(uint8_t joint_num, int16_t max_pwm) 
    { _joint_params.params[joint_num].max_pwm=max_pwm; return pushParams(1);}

    inline int16_t kp(uint8_t joint_num) const 
    { return _joint_params.params[joint_num].kp;}
  
    inline bool setKp(uint8_t joint_num, int16_t kp) 
    { _joint_params.params[joint_num].kp=kp; return pushParams(1);}

    inline int16_t ki(uint8_t joint_num) const 
    { return _joint_params.params[joint_num].kp;}
  
    inline bool setKi(uint8_t joint_num, int16_t ki) 
    { _joint_params.params[joint_num].ki=ki; return pushParams(1);}

    inline int16_t kd(uint8_t joint_num) const 
    { return _joint_params.params[joint_num].kd;}
  
    inline bool setKd(uint8_t joint_num, int16_t kd) 
    { _joint_params.params[joint_num].kd=kd; return pushParams(1);}

    inline int16_t maxSpeed(uint8_t joint_num) const 
    { return _joint_params.params[joint_num].max_speed;}
  
    inline bool setMaxSpeed(uint8_t joint_num, int16_t max_speed) 
    { _joint_params.params[joint_num].max_speed=max_speed; return pushParams(1);}

    inline int16_t slope(uint8_t joint_num) const 
    { return _joint_params.params[joint_num].slope;}

    inline bool setSlope(uint8_t joint_num, uint16_t slope_) 
    {_joint_params.params[joint_num].slope=slope_; return pushParams(1);}
  
    inline int16_t maxI(uint8_t joint_num) const  
    { return _joint_params.params[joint_num].max_i; }

    inline bool setMaxI(uint8_t joint_num, uint16_t max_i) 
    {_joint_params.params[joint_num].max_i=max_i; return pushParams(1);}

    inline uint16_t jointSeq() const 
    {return _joint_status.seq;}
  
    void jointPositions(int16_t* ticks) const;
  
    void jointSpeeds(int16_t* joint_speeds) const;
  
    void jointPWM(int16_t* joint_pwms) const;
  
    void setJointSpeeds(const int16_t* joint_speeds);
  
    void setJointPWM(const int16_t* joint_pwms);

    // kinematic interface
    inline uint16_t kinematicsSeq() const 
    {return _kinematics_status.seq;}

    inline float kr() const 
    {return _kinematics_params.kr;}
  
    inline bool setKr(float kr) 
    {_kinematics_params.kr=kr; return pushParams(2);}

    inline float kl() const 
    {return _kinematics_params.kl;}
  
    inline bool setKl(float kl) 
    {_kinematics_params.kl=kl; return pushParams(2);}

    inline float baseline() const 
    {return _kinematics_params.baseline;}
  
    inline bool setBaseline(float baseline) 
    {_kinematics_params.baseline=baseline; return pushParams(2);}

    inline uint8_t rightMotorIndex() const 
    {return _kinematics_params.right_joint_index;}
  
    inline bool setRightMotorIndex(uint8_t idx) 
    {_kinematics_params.right_joint_index = idx; return pushParams(2);}

    inline uint8_t leftMotorIndex() const 
    {return _kinematics_params.left_joint_index;}
  
    inline bool setLeftMotorIndex(uint8_t idx) 
    {_kinematics_params.left_joint_index = idx; return pushParams(2);}

    inline bool setMaxTranslationalVelocity(const float tv)
    {_kinematics_params.max_translational_velocity=tv; return pushParams(2);}
    
    inline bool setMaxTranslationalAcceleration(const float ta)
    { _kinematics_params.max_translational_acceleration=ta; return pushParams(2); }

    inline bool setMaxTranslationalBrake(const float td)
    { _kinematics_params.max_translational_brake=td; return pushParams(2); }

    inline bool setMaxRotationalVelocity(const float rv)
    { _kinematics_params.max_rotational_velocity=rv; return pushParams(2); }

    inline bool setMaxRotationalAcceleration(const float ra)
    { _kinematics_params.max_rotational_acceleration=ra; return pushParams(2);}
    
    inline float maxTranslationalVelocity() const {return _kinematics_params.max_translational_velocity;}
    
    inline float maxRotationalVelocity() const {return _kinematics_params.max_rotational_velocity;}
    
    inline float maxTranslationalAcceleration() const {return _kinematics_params.max_translational_acceleration;}
    
    inline float maxTranslationalDeceleration() const {return _kinematics_params.max_translational_acceleration;}
    
    inline float maxRotationalAcceleration() const {return _kinematics_params.max_rotational_acceleration;}

    inline float x() const 
    {return _kinematics_status.odom_x;} //< odometry x [m]
  
    inline float y() const 
    {return _kinematics_status.odom_y;} //< odometry y [m]
  
    inline float theta() const 
    {return _kinematics_status.odom_theta;} //< odometry theta [rad]
  
    inline float translationalVelocityMeasured() const 
    {return _kinematics_status.translational_velocity_measured;} //< translational_speed [m/s]
  
    inline float rotationalVelocityMeasured() const 
    {return _kinematics_status.rotational_velocity_measured;} //< rotational_speed [rad/s]

    inline float translationalVelocityDesired() const 
    {return _kinematics_status.translational_velocity_desired;} //< translational_speed [m/s]
  
    inline float rotationalVelocityDesired() const 
    {return _kinematics_status.rotational_velocity_desired;} //< rotational_speed [rad/s]

    inline float translationalVelocityAdjusted() const 
    {return _kinematics_status.translational_velocity_adjusted;} //< translational_speed [m/s]
  
    inline float rotationalVelocityAdjusted() const 
    {return _kinematics_status.rotational_velocity_adjusted;} //< rotational_speed [rad/s]

    void setBaseVelocities(float translational_speed, float rotational_speed); 


    // low level interface
    inline const JointStatusPacket& jointStatus() const {return _joint_status;}
    inline  JointParamPacket& jointParams()  {return _joint_params;}
    inline const ResponsePacket& lastResponse() const {return _last_response;}
    inline const SystemStatusPacket& systemStatus() const {return _system_status;}
    inline SystemParamPacket& systemParams()  {return _system_params;}
    inline const DifferentialDriveStatusPacket& kinematicsStatus() const {return _kinematics_status;}
    inline DifferentialDriveParamPacket& kinematicsParams() {return _kinematics_params;}
    bool controlParams(uint8_t command, uint8_t param_type, int retries=0);
    bool controlJoint(uint8_t index, JointInfo::Mode mode, int16_t speed);
    void spinOnce();


  protected:
    pthread_mutex_t _comm_mutex;
    PacketEncoder _packet_encoder;
    PacketDecoder _packet_decoder;
    int _packet_count;
    int _packet_errors;

    //Stats
    uint16_t _seq;

    // received packets
    ResponsePacket _last_response;
    SystemStatusPacket _system_status;
    JointStatusPacket _joint_status;
    SystemParamPacket _system_params;
    JointParamPacket _joint_params;
    DifferentialDriveStatusPacket _kinematics_status;
    DifferentialDriveParamPacket _kinematics_params;

    //posted packets;
    bool _post_joint_control;
    JointControlPacket _joint_control;
  
    bool _post_base_velocities;
    DifferentialDriveControlPacket _base_velocities;
    int _packet_retries;
    int _fd;
  };
}
