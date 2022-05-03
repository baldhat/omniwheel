
#include <EEPROM.h>

#include "Arduino.h"

#define VELO_ADDR 0
#define ACCL_ADDR 4
#define STEP_ADDR 8

/*
  Load the max wheel accelleration value from the EEPROM
*/
float getAcceleration() {
  float acceleration;
  EEPROM.get(ACCL_ADDR, acceleration);
  return acceleration;
}

/*
  Load the max wheel velocity value from the EEPROM
*/
float getVelocity() {
  float velocity;
  EEPROM.get(VELO_ADDR, velocity);
  return velocity;
}

/*
  Set the max wheel velocity value in the EEPROM
*/
void setVelocity(float velocity) {
  EEPROM.put(VELO_ADDR, velocity);
}


/*
  Set the max wheel acceleration in the EEPROM
*/
void setAcceleration(float acceleration) {
  EEPROM.put(ACCL_ADDR, acceleration);
}


/*
  Set the micro step setting the the EEPROM
*/
void setMicrosteps(int micro_steps) {
  EEPROM.put(STEP_ADDR, micro_steps);
}


/*
  Load the number of micro steps to be used  from the EEPROM
*/
int getMicrosteps() {
  int micro_steps;
  EEPROM.get(STEP_ADDR, micro_steps);
  return micro_steps;
}
