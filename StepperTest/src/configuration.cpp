
#include <EEPROM.h>

#include "Arduino.h"

#define VELO_ADDR 0
#define ACCL_ADDR 4

void printVelocity() {
  float velocity;
  EEPROM.get(VELO_ADDR, velocity);
  Serial.println(velocity);
}

void printAccelleration() {
  float accelleration;
  EEPROM.get(ACCL_ADDR, accelleration);
  Serial.println(accelleration);
}

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
