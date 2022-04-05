
#include <EEPROM.h>

#include "Arduino.h"

#define VELO_ADDR 0
#define ACCL_ADDR 4
#define STEP_ADDR 8

float getAcceleration() {
  float acceleration;
  EEPROM.get(ACCL_ADDR, acceleration);
  return acceleration;
}

float getVelocity() {
  float velocity;
  EEPROM.get(VELO_ADDR, velocity);
  return velocity;
}

void setVelocity(float velocity) {
  EEPROM.put(VELO_ADDR, velocity);
}

void setAcceleration(float acceleration) {
  EEPROM.put(ACCL_ADDR, acceleration);
}

void setMicrosteps(int micro_steps) {
  EEPROM.put(STEP_ADDR, micro_steps);
}

int getMicrosteps() {
  int micro_steps;
  EEPROM.get(STEP_ADDR, micro_steps);
  return micro_steps;
}
