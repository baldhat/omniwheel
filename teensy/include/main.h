#include "Arduino.h"

#ifndef main_h
#define main_h
#include "parser.h"

float velocityToPWMPeriod(float velocity_wheel1);
void runCommand(Command command);
float PWMPeriodToVelocity(float pwmPeriod);
void setMicrostepPins(int micro_steps);
void interactiveDriving();
void runInteractiveCommand(Command command);
void update(Command command);

#endif
