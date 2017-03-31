#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <curses.h>
#include <linux/joystick.h>
#include "orazio_robot_connection.h"
#include <iostream>
#include <sstream>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

using namespace std;
using namespace srrg_orazio_core;

OrazioRobotConnection connection;
enum Mode {System=0x0, PWM=0x1, PID=0x2, DifferentialDrive=0x3};
const char *mode_strings[]={"System", "PWM", "PID", "DifferentialDrive"};
int joy_fd=-1;
int joy_tv_axis=1;
int joy_rv_axis=3;
float max_tv=0.5;
float max_rv=0.5;
float tvscale = max_tv/32767.0;
float rvscale = max_rv/32767.0;
  
void printResponsePacket(WINDOW* w, const ResponsePacket& p){
  ostringstream  os;
  os << "RESPONSE s: " << p.seq 
     << " id: " << (int) p.src_id 
     << " e: " << (int) p.error_code << endl;
  werase(w);
  waddstr(w, os.str().c_str());
}

void printSystemStatusPacket(WINDOW* w, const SystemStatusPacket& p){
  ostringstream  os;
  os << "SYSTEM_STATUS " << p.seq << endl;
  os << "  rx_packets: " << p.rx_packets << endl; 
  os << "  rx_errors: " << p.rx_packet_errors << endl; 
  os << "  tx_packets: " << p.tx_packets << endl; 
  os << "  watchdog:   " << p.watchdog_count << endl;
  os << "  battery: " << p.battery_level << endl; 
  os << endl;
  os << "CLIENT SERIAL:" << endl;
  os << "  rx_packets: " << connection.packetCount() << endl;
  os << "  rx_errors: " << connection.packetErrors() << endl;
  werase(w);
  waddstr(w, os.str().c_str());
}

void printSystemParamPacket(WINDOW* w, const SystemParamPacket& p){
  ostringstream  os;
  os << "SYSTEM_PARAM " << p.seq << endl;
  os << "  comm_speed: " << p.comm_speed << endl;
  os << "  comm_cycles: " << p.comm_cycles << endl;
  os << "  ws_cycles: " << p.watchdog_cycles << endl;
  os << "  motor_mode: " << (int)p.motor_mode << endl;
  os << "  timer_period: " << p.timer_period << endl;
  werase(w);
  waddstr(w, os.str().c_str());
}

void printJointStatusPacket(WINDOW* w, const JointStatusPacket& p){
  ostringstream  os;
  os << "JOINT_STATUS ";
  os << (int) p.seq << endl;
  for(int i=0; i<num_motors; i++){
    const JointInfo& joint=p.joints[i];
    os << "  Joint: " << i << endl;
    os << "    status: " << (int) joint.mode << endl; 
    os << "    position: " << (int) joint.encoder_position << endl; 
    os << "    encoder_speed: " << (int) joint.encoder_speed << endl; 
    os << "    desired_speed: " << (int) joint.desired_speed << endl; 
    os << "    pwm: " << (int) joint.pwm << endl; 
    os << "    sensed_current: " << (int) joint.sensed_current << endl; 
  }
  werase(w);
  waddstr(w, os.str().c_str());
}


void printJointParamPacket(WINDOW* w, const JointParamPacket& p){
  ostringstream  os;
  os << "JOINT_PARAMS " << p.seq << endl;
  for(int i=0; i<num_motors; i++){
    const JointParams& joint=p.params[i];
    os << "  Joint:   " << i <<endl;
    os << "    kp:    " << (int) joint.kp << endl;
    os << "    ki:    " << (int) joint.ki << endl;
    os << "    kd:    " << (int) joint.kd << endl;
    os << "    max_speed: " << (int) joint.max_speed << endl; 
    os << "    pwmmin:" << (int) joint.min_pwm << endl;
    os << "    pwmmax:" << (int) joint.max_pwm << endl;
    os << "    speed_slope: " << (int) joint.slope << endl; 
  }
  werase(w);
  waddstr(w, os.str().c_str());
}

void printKinematicsParamPacket(WINDOW* w, const DifferentialDriveParamPacket& p){
  ostringstream  os;
  os << "KINEMATIC_PARAMS " << p.seq << endl;
  os << "  baseline: " << p.baseline << endl;
  os << "  kr:       " << p.kr << endl;
  os << "  kl:       " << p.kl << endl;
  os << "  right_id: " << (int) p.right_joint_index << endl;
  os << "  left_id:  " << (int) p.left_joint_index << endl;
  werase(w);
  waddstr(w, os.str().c_str());
}

void printKinematicsStatus(WINDOW* w, const DifferentialDriveStatusPacket& p){
  ostringstream  os;
  os << "KINEMATIC_STATUS " << p.seq << endl;
  os << "  x:     " << p.odom_x << endl;
  os << "  y:     " << p.odom_y << endl;
  os << "  theta: " << p.odom_theta << endl;
  os << "  tv_meas:    " << p.translational_velocity_measured << endl;
  os << "  rv_meas:    " << p.rotational_velocity_measured << endl;
  os << "  tv_des:    " << p.translational_velocity_desired << endl;
  os << "  rv_des:    " << p.rotational_velocity_desired << endl;
  os << "  tv_adj:    " << p.translational_velocity_adjusted << endl;
  os << "  rv_adj:    " << p.rotational_velocity_adjusted << endl;
  werase(w);
  waddstr(w, os.str().c_str());
}



void handleSystemMode(int key){
}

void handlePWMMode(int key){
  int pwm_a=connection.jointStatus().joints[0].pwm;
  int pwm_b=connection.jointStatus().joints[1].pwm;
  switch(key){
  case KEY_UP:
    pwm_a++;
    break;
  case KEY_DOWN:
    pwm_a--;
    break;
  case KEY_LEFT:
    pwm_b--;
    break;
  case KEY_RIGHT:
    pwm_b++;
    break;
  default:;
  }
  connection.controlJoint(0,JointInfo::PWM, pwm_a);
  connection.controlJoint(1,JointInfo::PWM, pwm_b);

}

void handlePIDMode(int key){
  int pid_a=connection.jointStatus().joints[0].desired_speed;
  int pid_b=connection.jointStatus().joints[1].desired_speed;
  switch(key){
  case KEY_UP:
    pid_a++;
    break;
  case KEY_DOWN:
    pid_a--;
    break;
  case KEY_LEFT:
    pid_b++;
    break;
  case KEY_RIGHT:
    pid_b--;
    break;
  default:;
  }
  connection.controlJoint(0,JointInfo::PID, pid_a);
  connection.controlJoint(1,JointInfo::PID, pid_b);
}

void handleDifferentialDriveMode(int key){
  static float tv=0;
  static float rv=0;
  switch(key){
  case KEY_UP:
    tv+=0.05;
    break;
  case KEY_DOWN:
    tv-=0.05;
    break;
  case KEY_LEFT:
    rv+=0.05;
    break;
  case KEY_RIGHT:
    rv-=0.05;
    break;
  case ' ':
    rv=0;
    tv=0;
    break;
  default:;
  }
  connection.setBaseVelocities(tv,rv);
}

static float last_tv=0, last_rv=0;

void handleDifferentialDriveModeJoy(){
  struct js_event e;
  
  if (read (joy_fd, &e, sizeof(e)) > 0 ){
    int axis = e.number;
    int value = e.value;
    int type = e.type;

    float tv=last_tv,rv=last_rv;
    if (axis == joy_tv_axis && type == 2) {
      tv = -value * tvscale;
    }
    if (axis == joy_rv_axis && type == 2) {
      rv = -value *rvscale;
    }
    connection.setBaseVelocities(tv,rv);
    last_tv=tv; last_rv=rv;
  }
  else
    connection.setBaseVelocities(last_tv,last_rv);
}

void handleMode(Mode m, int key){
  if (joy_fd>0 && m==DifferentialDrive){
    handleDifferentialDriveModeJoy();
    return;
  }
  if (key<0)
    return;
  switch(m){
  case System:
    handleSystemMode(key);
    break;
  case PWM:
    handlePWMMode(key);
    break;
  case PID:
    handlePIDMode(key);
    break;
  case DifferentialDrive:
    handleDifferentialDriveMode(key);
    break;
  } 
}

const char* banner[]={
  "orazio_robot_monitor: controller of the robot",
  "usage:",
  "orazio_robot_monitor [params]",
  "params: ",
  " -serial-port <string>, default /dev/ttyACM0",
  " -joy-device <string>, default none",
  0
};

void printBanner(){
  const char** c=banner;
  while (*c){
    cerr << *c << endl;
    c++;
  }
}

int main(int argc, char** argv) {

  std::string serial_port="/dev/ttyACM0";
  std::string joy_device="";
  int c=1;
  while (c<argc){
    if (! strcmp(argv[c],"-h")){
      printBanner();
      return 0;
    }
    if (! strcmp(argv[c],"-serial-port")){
      c++;
      if (c<argc)
	serial_port=argv[c];
    }
    if (! strcmp(argv[c],"-joy-device")){
      c++;
      if (c<argc)
	joy_device=argv[c];
    }
    c++;
  }

  cerr << "running with params" << endl;
  cerr << "-serial-port: " << serial_port << endl;
  cerr << "-joy-device: " << joy_device << endl;

  if (joy_device.length()){
    joy_fd = open (joy_device.c_str(), O_RDONLY|O_NONBLOCK);
    if (joy_fd<0) {
      cerr << "no joy found in [" << joy_device << "]" <<endl;
    }
  }

  connection.connect(serial_port.c_str());
  if (! connection.isConnected()){
    cerr << "unable to open serial port [" << serial_port <<"]" << endl;
    return 0;
  }

  

  WINDOW * mainwin;
    
  if ( (mainwin = initscr()) == NULL ) {
    fprintf(stderr, "Error initialising ncurses.\n");
    exit(EXIT_FAILURE);
  }

  cbreak();
  nodelay(mainwin, true);
  keypad(mainwin, true);
  noecho();

  char message[80];
  WINDOW* message_win = newwin(1, 80,0, 0);
  box(message_win, 0 , 0);

  WINDOW* system_status_win = newwin(20, 40, 2, 0);
  wrefresh(system_status_win);	
  WINDOW* system_params_win = newwin(20, 40, 2, 40);
  wrefresh(system_params_win);	
  
  WINDOW* joint_status_win = newwin(20, 40, 2, 0);
  wrefresh(joint_status_win);	
  WINDOW* joint_params_win = newwin(20, 40, 2, 40);
  wrefresh(system_params_win);	

  WINDOW* kinematics_status_win = newwin(20, 40, 2, 40);
  wrefresh(kinematics_status_win);	
  WINDOW* kinematics_params_win = newwin(20, 40, 10, 40);
  wrefresh(kinematics_params_win);	


  WINDOW* response_win = newwin(1, 80, 1, 0);
  wrefresh(response_win);	
  
  int param_query_retries=10;
  // read the parameters from the base;
  for (int i=0; i< param_query_retries; i++){
    connection.controlParams(0,0);
    connection.spinOnce();
    connection.controlParams(0,1);
    connection.spinOnce();
    connection.controlParams(0,2);
    connection.spinOnce();
  }
  
  bool run=1;
  Mode mode=System;

 
  while(run) {
    connection.spinOnce();
    switch(mode){
    case System:
      printSystemStatusPacket(system_status_win, connection.systemStatus());
      wrefresh(system_status_win);		
      printSystemParamPacket(system_params_win, connection.systemParams());
      wrefresh(system_params_win);		
      break;
    case PWM:
    case PID:
      printJointStatusPacket(joint_status_win, connection.jointStatus());
      wrefresh(joint_status_win);		
      printJointParamPacket(joint_params_win, connection.jointParams());
      wrefresh(joint_params_win);		
      break;
    case DifferentialDrive:
      printJointStatusPacket(joint_status_win, connection.jointStatus());
      wrefresh(joint_status_win);		
      printKinematicsStatus(kinematics_status_win, connection.kinematicsStatus());
      wrefresh(kinematics_status_win);
      printKinematicsParamPacket(kinematics_params_win, connection.kinematicsParams());
      wrefresh(kinematics_params_win);
      break;
    default:;
    }
    printResponsePacket(response_win, connection.lastResponse());
    wrefresh(response_win);
    refresh();
 
    int key=getch();
    switch (key){
    case 'q': 
      run=false;
      break;
    case '0': 
      connection.controlJoint(0,JointInfo::Disabled,0);
      connection.controlJoint(1,JointInfo::Disabled,0);
      mode=System;
      break;
    case '1': 
      mode=PWM;
      break;
    case '2': 
      mode=PID;
      break;
    case '3': 
      mode=DifferentialDrive;
      break;
    default:
      handleMode(mode, key);
    }


    werase(message_win);
    sprintf (message, "MODE: %s  \n", mode_strings[mode]);
    waddstr(message_win, message);
    wrefresh(message_win);

  }


  /*  Clean up after ourselves  */
  delwin(system_status_win);
  delwin(joint_status_win);
  
  delwin(system_params_win);
  delwin(joint_params_win);
  
  endwin();

  return EXIT_SUCCESS;
}
