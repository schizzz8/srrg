#pragma once
#include "motor.h"
#include "encoder.h"
#include "joint_controller.h"
#include "packets.h"
#include "differential_drive_kinematics.h"

/**
   @file globals.h
   contains platform specific implementations 
   and initializations for our platform
*/


/****************** MOTOR  SETUP AND GLOBALS *******************/

extern Motor motors[num_motors];


/****************** JOINT AND KINEMATICS *******************/


//! these are the PID+all controllers for the joints
extern JointController joint_controllers[num_motors];

//! robot kinematics
extern DifferentialDriveKinematics kinematics;


//! status of the system (battery level, tx rx packets)
extern SystemStatusPacket system_status;

//! parameters of the system (comm speed, algorithm configuration, timers and so on)
extern SystemParamPacket system_params;

//! global variable used to keep the status of each joint
extern JointStatusPacket joints_status;

//! parameters of each joint
extern JointParamPacket  joints_params;

extern DifferentialDriveStatusPacket kinematics_status;

extern DifferentialDriveParamPacket kinematics_params;

//! initializes the system parameters
void initSystemParams();
void updateStatusParams();
void updateSystemStatus();

//! initializes joints and controllers
void initJoints();
void updateJointsParams();
void updateJointsStatus();


void initKinematics();
void updateKinematicsParams();
void updateKinematicsStatus();

//! this handles all the burden for a joint
void handleJointsISR();

//! this handles the base velocity profile for a differential drive
//! to be called in the main loop
void handleBaseVelocities();


