#pragma once
#include "packets.h"

using namespace srrg_orazio_core;
const uint32_t firmware_version=0xC4220;

const int eeprom_version_address=0x0000;
const int eeprom_system_param_address=0x0010;
const int eeprom_joint_param_address=eeprom_system_param_address + sizeof(SystemParamPacket);
const int eeprom_kinematics_param_address=eeprom_joint_param_address + sizeof(JointParamPacket);


//! initializes the eeprom with firmware version
//! if it is the first time this function is called, it writes once the stuff in EEPROM  
void initEEPROM();

//! reads the default params from eeprom
void loadParamsFromEEPROM(uint8_t param_type);

//! writes the default params to eeprom
void writeParamsToEEPROM(uint8_t param_type);
