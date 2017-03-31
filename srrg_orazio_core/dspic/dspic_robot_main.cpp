#include <stdint.h>
#include <stdio.h>
#include "globals.h"
#include "comm_globals.h"
#include "eeprom.h"
#include "dspic_settings.h"
#include "Arduino.h"
#include <string.h>

//================================
// TIMER 4 is for the PID loop, or basically the scheduler routine
//================================

bool handle_communications=false;

uint16_t comm_counter=1;

bool timer_started=false;
void initScheduler() { //10ms
  timer_started=true;
}

void initMotors() {
  motors[0].init(3,12,9,A0);
  motors[1].init(11,13,8,A1);
}

void __attribute__((__interrupt__, __no_auto_psv__)) _T1Interrupt(void) {
  if (! timer_started)
    return;

  //handleJointsISR();
  --comm_counter;
  if (comm_counter==0){
    comm_counter=10;
    handle_communications=true;
  }
}


void setup(){
  //initialize the globals and the parameters
  initArduinoEmulator();
  /*
  initEEPROM();
  initSystemParams();
  initEncoders();
  initMotors();
  initJoints();
  initKinematics();
  loadParamsFromEEPROM(0);
  loadParamsFromEEPROM(1);
  loadParamsFromEEPROM(2);
  initCommunication();
  */
  initScheduler();
}

const char* s="il cazzo\n";

void loop() {
  /*
  if (handle_communications){
    handle_communications=false;
    updateSystemStatus();
    updateJointsStatus();
    packet_encoder.putPacket(system_status);
    packet_encoder.putPacket(joints_status);
    if (kinematics_params.use_on_board_odometry){
      updateKinematicsStatus();
      packet_encoder.putPacket(kinematics_status);
    }
    if (handleCommunication())
      system_status.watchdog_count=system_params.watchdog_cycles;
  }
  */

    int l=strlen(s);
    for (int i=0; i<l; i++){
      Serial.write(s[i]);
    }
    Serial.flush();

}

int main() {
  setup();
  while(1)
    loop();
}
