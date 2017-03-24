#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <srrg_core_ros/Ticks.h>
#include "orazio_robot_connection.h"
#include <iostream>
#include <tf/tf.h>
using namespace std;
using namespace srrg_orazio_core;

OrazioRobotConnection robot;

const double uint16_to_radians = 2*M_PI/65536;

void commandVelCallback(const geometry_msgs::TwistConstPtr twist){
  robot.setBaseVelocities(twist->linear.x, twist->angular.z);
}

int main(int argc, char** argv) {
  std::string serial_device;
  std::string odom_topic;
  std::string ticks_topic;
  std::string joint_state_topic;
  std::string odom_frame_id;
  std::string command_vel_topic;

  ros::init(argc, argv, "orazio_robot_node");
  ros::NodeHandle nh("~");
  nh.param("serial_device", serial_device, std::string("/dev/ttyACM0"));
  nh.param("odom_topic", odom_topic, std::string("/odom"));
  nh.param("ticks_topic", ticks_topic, std::string("/ticks"));
  nh.param("joint_state_topic", joint_state_topic, std::string("/joint_state"));
  nh.param("command_vel_topic", command_vel_topic, std::string("/cmd_vel"));
  nh.param("odom_frame_id", odom_frame_id, std::string("/odom"));



  cerr << "running with params: ";
  cerr << "serial_device: " << serial_device << endl;
  cerr << "odom_topic: " << odom_topic << endl;
  cerr << "odom_frame_id: " << odom_frame_id << endl;
  cerr << "command_vel_topic: " << command_vel_topic << endl;
  ros::Subscriber command_vel_subscriber = nh.subscribe<geometry_msgs::TwistConstPtr>(command_vel_topic, 1, &commandVelCallback);
  ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 1);
  ros::Publisher ticks_publisher = nh.advertise<srrg_core_ros::Ticks>(ticks_topic, 1);
  ros::Publisher joint_state_publisher = nh.advertise<sensor_msgs::JointState>(joint_state_topic, 1);
  robot.connect(serial_device.c_str());
  cerr << "querying system params... ";
  while (!robot.queryParams(0))
    cerr << ".";
  cerr << " OK" << endl;

  cerr << "querying joint params... ";
  while(!robot.queryParams(1))
    cerr << ".";
  cerr << " OK" << endl;

  cerr << "querying base params... ";
  while (!robot.queryParams(2))
    cerr << ".";
  cerr << " OK" << endl;

  nav_msgs::Odometry odom;
  srrg_core_ros::Ticks ticks;
  sensor_msgs::JointState joint_state;
  joint_state.name.resize(2);
  joint_state.position.resize(2);
  joint_state.velocity.resize(2);
  joint_state.effort.resize(2);

  joint_state.name[0]="left_wheel";
  joint_state.name[1]="right_wheel";
  joint_state.effort[0]=0;
  joint_state.effort[1]=0;
  
  odom.header.frame_id = odom_frame_id;
  int seq = 0;
  while(ros::ok()){
    ros::spinOnce();
    robot.spinOnce();

    if (robot.kinematicsSeq()==seq){
      continue;
    }
    seq=robot.kinematicsSeq();
    // send the odometry
    ros::Time this_time=ros::Time::now();
    odom.header.seq = seq;
    odom.header.stamp = this_time;
    odom.pose.pose.position.x = robot.x();
    odom.pose.pose.position.y = robot.y();
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation=tf::createQuaternionMsgFromYaw(robot.theta());
    odom_publisher.publish(odom);
    uint16_t encoder_ticks[num_motors];
    JointStatusPacket joint_status=robot.jointStatus();
    
    uint16_t left_encoder_position=joint_status.joints[robot.leftMotorIndex()].encoder_position;
    uint16_t right_encoder_position=joint_status.joints[robot.rightMotorIndex()].encoder_position;
    int16_t left_encoder_speed=joint_status.joints[robot.leftMotorIndex()].encoder_speed;
    int16_t right_encoder_speed=joint_status.joints[robot.rightMotorIndex()].encoder_speed;
    
    ticks.header.seq = seq;
    ticks.header.stamp = this_time;
    ticks.leftEncoder = left_encoder_position;
    ticks.rightEncoder = right_encoder_position;
    ticks_publisher.publish(ticks);


    
    joint_state.header=ticks.header;
      
    joint_state.position[0]=uint16_to_radians*left_encoder_position;
    joint_state.position[1]=uint16_to_radians*right_encoder_position;
    joint_state.velocity[0]=uint16_to_radians*left_encoder_speed/robot.timerPeriod();
    joint_state.velocity[1]=uint16_to_radians*right_encoder_speed/robot.timerPeriod();
    joint_state_publisher.publish(joint_state);
   }
}
