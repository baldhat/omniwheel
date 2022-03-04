#include "Arduino.h"

#ifndef main_h
#define main_h
#include "parser.h"

void rotate(float relativeAngle, float angularVelocity);
float velocityToPWMPeriod(float velocity_wheel1);
void runCommand(Command command);
float PWMPeriodToVelocity(float pwmPeriod);
void driveDirectLine(float direction, float distance, float velocity, float accelleration);

#endif
