#include "EEPROM.h"
#include "eeprom.h"
#include  "globals.h"

template <typename T>
void getEEPROM(int address, T& value) {
  char* c=reinterpret_cast<char*>(&value);
  int a=address;
  for (int i=0; i<sizeof(T); i++){
    *c=EEPROM.read(a);
    c++;
    a++;
  }
}

template <typename T>
void putEEPROM(int address, const T& value) {
  const char* c=reinterpret_cast<const char*>(&value);
  int a=address;
  for (int i=0; i<sizeof(T); i++){
    EEPROM.write(a, *c);
    c++;
    a++;
  }
}

template <typename T>
void updateEEPROM(int address, const T& value) {
  const char* c=reinterpret_cast<const char*>(&value);
  int a=address;
  for (int i=0; i<sizeof(T); i++){
    char new_char=EEPROM.read(a);
    if (new_char!=*c)
      EEPROM.write(a, *c);
    c++;
    a++;
  }
}


void initEEPROM(){
  // read the version from eeprom
  // if it matches, do nothing
  // otherwise write it on eeprom, and write once the parameters
  uint32_t version;
  getEEPROM(eeprom_version_address, version);
  if (version==firmware_version) {
    loadParamsFromEEPROM(0);
    loadParamsFromEEPROM(1);
    loadParamsFromEEPROM(2);
    return;
  }
  putEEPROM(eeprom_version_address, firmware_version);
  system_params=SystemParamPacket();
  system_params.seq=0;
  system_params.id=SystemParamPacket::default_id;
  writeParamsToEEPROM(0);

  joints_params=JointParamPacket();
  joints_params.id=JointParamPacket::default_id;
  joints_params.seq=0;
  writeParamsToEEPROM(1);

  kinematics_params=DifferentialDriveParamPacket();
  kinematics_params.seq=0;
  kinematics_params.id=DifferentialDriveParamPacket::default_id;
  writeParamsToEEPROM(2);
}

void loadParamsFromEEPROM(uint8_t param_type){
  switch(param_type){
  case 0: 
    getEEPROM(eeprom_system_param_address, system_params);
    system_params.timer_period=0.01;
    break;
  case 1:
    getEEPROM(eeprom_joint_param_address, joints_params);
    updateJointsParams();
    break;
  case 2:
    getEEPROM(eeprom_kinematics_param_address, kinematics_params);
    updateKinematicsParams();
    break;
  }
}

void writeParamsToEEPROM(uint8_t param_type){
  switch(param_type){
  case 0: 
    system_params.seq=0;
    putEEPROM(eeprom_system_param_address, system_params);
    break;
  case 1:
    joints_params.seq=0;
    putEEPROM(eeprom_joint_param_address, joints_params);
    break;
  case 2:
    kinematics_params.seq=0;
    putEEPROM(eeprom_kinematics_param_address, kinematics_params);
    break;
  }
}
