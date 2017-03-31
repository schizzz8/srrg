#include "orazio_robot_connection.h"

#include <string>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

using namespace std;
using namespace srrg_orazio_core;

OrazioRobotConnection robot;
string serial_port = "/dev/ttyACM0";
bool run = 1;
int which_program=1;
double speed_tv=0, speed_rv=0;


bool connect() {
  cout << "Connecting to serial port: " << serial_port << " ... " << endl;
  robot.connect(serial_port.c_str());
  if (! robot.isConnected()){
    cerr << "ERROR: unable to open serial port [" << serial_port <<"]" << endl;
    return 0;
  }
  cout << "Connected." << endl;
  return 1;
}

void setSpeed(double tv, double rv) {
    speed_tv=tv; speed_rv=rv;
}

void wait(double sec) {
    usleep(sec*1e6);
}


void robot_control_program_1(); 
void robot_control_program_2();
void robot_control_program_3();
void robot_control_program_4();
void robot_control_program_5();
void robot_control_program_6();
void robot_control_program_7();
void robot_control_program_8();
void robot_control_program_9();


void* run_thread(void *arg) {
    cout << "Robot running" << endl;
    while(run) {
        robot.setBaseVelocities(speed_tv,speed_rv);
        robot.spinOnce();
        int ss = robot.systemSeq();
        if (ss%10==0)
        cout << "DEBUG: " << ss << " " << robot.x() << " " << robot.y() << " " << robot.theta() << endl;
    }
}



void read_params(int argc, char** argv) {
    if (argc>1)
        which_program=atoi(argv[1]);
}

int main(int argc, char** argv) {

    if (connect()) {
        run = 1;

        read_params(argc,argv);

        // run thread
        pthread_t trun;
        int r = pthread_create(&trun,NULL,run_thread, NULL);
        if (r!=0) {
            cerr << "ERROR: cannot create thread" << endl;
            run = 0;
        }

        if (run) {
            // execute robot program
            cout << "Executing robot program " << which_program << endl;

            switch (which_program) {
                case 1:
                    robot_control_program_1(); break;
                case 2:
                    robot_control_program_2(); break;
                case 3:
                    robot_control_program_3(); break;
                case 4:
                    robot_control_program_4(); break;
                case 5:
                    robot_control_program_5(); break;
                case 6:
                    robot_control_program_6(); break;
                case 7:
                    robot_control_program_7(); break;
                case 8:
                    robot_control_program_8(); break;
                case 9:
                    robot_control_program_9(); break;
            }
        }

        robot.setBaseVelocities(0,0);
        robot.spinOnce();

    }


    cout << "Quit" << endl;

}


/*
 *   USER ROBOT PROGRAMS 1..9
 *   RUN EACH PROGRAM <i> WITH THE COMMAND
 *     ./robot_program <i>
 */


void robot_control_program_1() {
    setSpeed(0.25,0.0);
    wait(4);
    setSpeed(0.0,0.0);
}


void robot_control_program_2() {

    setSpeed(0.25, 0.0);
    wait(4);
    setSpeed(0.0, 1);
    wait(M_PI/2);

    setSpeed(0.25,0.0);
    wait(4);
    setSpeed(0.0,1);
    wait(M_PI/2);

    setSpeed(0.25,0.0);
    wait(4);
    setSpeed(0.0,1);
    wait(M_PI/2);

    setSpeed(0.25,0.0);
    wait(4);
    setSpeed(0.0,1);
    wait(M_PI/2);

    setSpeed(0.0,0.0);
}


void robot_control_program_3() {

    for (int i=0; i<4; i++) {
        setSpeed(0.25,0.0);
        wait(4);
        setSpeed(0.0,1);
        wait(M_PI/2);
    }

    setSpeed(0.0,0.0);
}

void robot_control_program_4() {
    setSpeed(M_PI/30,M_PI/30);
    wait(60);
    setSpeed(0.0,0.0);
}

void robot_control_program_5() {

}

void robot_control_program_6() {

}

void robot_control_program_7() {

}

void robot_control_program_8() {

}

void robot_control_program_9() {

}











