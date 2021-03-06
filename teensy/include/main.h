#include "Arduino.h"

#ifndef main_h
#define main_h
#include "parser.h"

void runCommand(Command command);
float velocityPWMConversion(float value);
void setMicrostepPins(int micro_steps);
void drivingLoop();
void runInteractiveCommand(Command command);
void updateTargets(Command command);
void setDirectionPins();
void sendSteps(long steps[3]);
void updateVelocities(float abs_spike_periods[3]);
void loadDefaultValues();
float getBatteryVoltage();
void changeMicroSteps(Command command);

#endif
