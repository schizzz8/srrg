#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <curses.h>
#include "orazio_robot_connection.h"
#include <iostream>
#include <sstream>
#include <map>
#include <pthread.h>
#include "command_parser.h"

using namespace std;
using namespace srrg_orazio_core;

OrazioRobotConnection connection;

void printSystemParamPacket(const SystemParamPacket& p){
  cout << "SYSTEM_PARAM " << p.seq << endl;
  cout << "  comm_speed: " << p.comm_speed << endl;
  cout << "  comm_cycles: " << p.comm_cycles << endl;
  cout << "  ws_cycles: " << p.watchdog_cycles << endl;
  cout << "  timer_period: " << p.timer_period << endl;
  cout << "  motor_mode: " << (int) p.motor_mode << endl;
}

void printJointParamPacket(const JointParamPacket& p){
  cout << "JOINT_PARAMS " << p.seq << endl;
  for(int i=0; i<num_motors; i++){
    const JointParams& joint=p.params[i];
    cout << "  Joint:   " << i <<endl;
    cout << "    kp:    " << (int) joint.kp << endl;
    cout << "    ki:    " << (int) joint.ki << endl;
    cout << "    kd:    " << (int) joint.kd << endl;
    cout << "    max_speed: " << (int) joint.max_speed << endl; 
    cout << "    pwmmin:" << (int) joint.min_pwm << endl;
    cout << "    pwmmin:" << (int) joint.max_pwm << endl;
    cout << "    speed_slope: " << (int) joint.slope << endl; 
  }
}

void printKinematicsParamPacket(const DifferentialDriveParamPacket& p){
  cout << "KINEMATIC_PARAMS " << p.seq << endl;
  cout << "  baseline: " << p.baseline << endl;
  cout << "  kr:       " << p.kr << endl;
  cout << "  kl:       " << p.kl << endl;
  cout << "  right_id: " << (int) p.right_joint_index << endl;
  cout << "  left_id:  " << (int) p.left_joint_index << endl;
}

class PrintParamCommandHandler: public BaseCommandHandler{
public:
  PrintParamCommandHandler(OrazioRobotConnection* c):
    BaseCommandHandler("printParams", c){}
  virtual bool handleCommand(istream& is) { 
    int value;
    is >> value;
    if (! is)
      return false;
    switch(value){
    case 0: 
      printSystemParamPacket(_robot_connection->systemParams());
      break;
    case 1: 
      printJointParamPacket(_robot_connection->jointParams());
      break;
    case 2: 
      printKinematicsParamPacket(_robot_connection->kinematicsParams());
      break;
    default:
      return false;
    }
    return true;
  }
protected:
  bool* _run;
};

bool run;
void* connectionThread(void*){
  while(run){
    connection.spinOnce();
  }
  return 0;
}


int main(int argc, char** argv) {
  bool run =true;
  initCommandMap(&connection, &run);
  addCommand(new PrintParamCommandHandler(&connection));
  connection.connect(argv[1]);
  if (! connection.isConnected()){
    return 0;
  }
  bool result;
  cout << "querying system params ... ";
  result=connection.queryParams(0);
  cout << result << endl;

  cout << "querying joint params ... ";
  result=connection.queryParams(1);
  cout << result << endl;

  cout << "querying kinematic params ... ";
  result=connection.queryParams(2);
  cout << result << endl;

  
  cerr <<  endl;
  cout << "READY" << endl;
  
  run = true;
  pthread_t runner;
  pthread_create(&runner, 0, &connectionThread, 0);
  while (run) {
    cout << "abot> ";
    char buf[1024];
    cin.getline(buf, 1024);
    istringstream is(buf);
    std::string tag;
    is >> tag;
    bool result = callCommand(tag, is);
    if (! result){
      cout << "error" << endl;
    } else 
      cout << "ok" << endl;
  }
  void* retval;
  pthread_join(runner, &retval);
  return 0;
}
