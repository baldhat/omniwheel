
#include <EEPROM.h>

#include "Arduino.h"

#define VELO_ADDR 0
#define ACCL_ADDR 4
#define STEP_ADDR 8

float getAccelleration() {
  float accelleration;
  EEPROM.get(ACCL_ADDR, accelleration);
  return accelleration;
}

float getVelocity() {
  float velocity;
  EEPROM.get(VELO_ADDR, velocity);
  return velocity;
}

void setVelocity(float velocity) {
  EEPROM.put(VELO_ADDR, velocity);
}

void setAccelleration(float accelleration) {
  EEPROM.put(ACCL_ADDR, accelleration);
}

void setMicrosteps(int micro_steps) {
  EEPROM.put(STEP_ADDR, micro_steps);
}

int getMicrosteps() {
  int micro_steps;
  EEPROM.get(STEP_ADDR, micro_steps);
  return micro_steps;
}
