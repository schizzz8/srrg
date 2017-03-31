#include <stdint.h>
#include <stdio.h>
#include "globals.h"
#include "comm_globals.h"
#include "eeprom.h"
#include <avr/sleep.h>

//================================
// TIMER 4 is for the PID loop, or basically the scheduler routine
//================================

bool handle_communications=false;

uint16_t comm_counter=0;


void initScheduler() { //10ms
  cli();
  TCCR4A = 0;
  TCCR4B = 0;
  OCR4A = 156;
  TCCR4B |= (1 << WGM42);
  TCCR4B |= (1 << CS40) | (1 << CS42);
  TIMSK4 = (1 << OCIE4A);
  const uint8_t myEraser = 0x7;             // this is 111 in binary and is used as an eraser
  TCCR1B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0
  TCCR3B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0
  const uint8_t myPrescaler = 0x1;         // this could be a number in [1 , 6]. In this case, 3 corresponds in binary to 011.   
  TCCR1B |= myPrescaler;  //this operation (OR), replaces the last three bits in TCCR2B with our new value 011
  TCCR3B |= myPrescaler;  //this operation (OR), replaces the last three bits in TCCR2B with our new value 011
  handle_communications=false;
  comm_counter=system_params.comm_cycles;
  sei();
}

ISR(TIMER4_COMPA_vect) {
  TCNT4 = 0;
  handleJointsISR();
  --comm_counter;
  if (comm_counter==0){
    comm_counter=system_params.comm_cycles;
    handle_communications=true;
  }
}

void initMotors() {
  if (system_params.motor_mode==0){
    motors[0].init(3,12,9,A0, system_params.motor_mode);
    motors[1].init(11,13,8,A1, system_params.motor_mode);
  }
  if (system_params.motor_mode==1){
    motors[0].init(2,3,-1,-1, system_params.motor_mode);
    motors[1].init(4,5,-1,-1, system_params.motor_mode);
  }
  /*if (system_params.motor_mode==2){
    motors[0].init(2,3,-1,-1, system_params.motor_mode);
    motors[1].init(4,5,-1,-1, system_params.motor_mode);
  }*/
}


void setup(){
  //initialize the globals and the parameters
  initEEPROM();
  initSystemParams();
  initKinematics();
  initEncoders();
  initJoints();
  loadParamsFromEEPROM(0);
  loadParamsFromEEPROM(1);
  loadParamsFromEEPROM(2);
  initMotors();
  initCommunication();
  initScheduler();
}

void loop() {
  if (handle_communications){
    handle_communications=false;
    updateSystemStatus();
    updateJointsStatus();
    updateKinematicsStatus();
    packet_encoder.putPacket(system_status);
    packet_encoder.putPacket(joints_status);
    packet_encoder.putPacket(kinematics_status);
         
    if (handleCommunication())
      system_status.watchdog_count=system_params.watchdog_cycles;

    if (system_status.last_packet_id==DifferentialDriveControlPacket::default_id)
      handleBaseVelocities();

    if (system_status.last_packet_id== SystemParamPacket::default_id)
      initMotors();

    system_status.idle_cycles=0;
  }
  flushCommunication();
  system_status.idle_cycles++;
}
