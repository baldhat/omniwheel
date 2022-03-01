
#include "Arduino.h"

void rotate(float relativeAngle, float angularVelocity);
void moveDirectLine(float relativeDirection, float velocity, float distance);
float velocityToPWMPeriod(float velocity_wheel1);
void receiveSerialData();
int parseCommand();
void runCommand();

// Macros for faster digital output
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define SWT(x,y) (x^=(1<<y))

#define RADIUS 0.12

// Motor 1
#define dirPin1 4
#define stepPin1 3
#define enablePin1 2

// Motor 2
#define dirPin2 7
#define stepPin2 6
#define enablePin2 5

// Motor 3
#define dirPin3 10
#define stepPin3 9
#define enablePin3 8

int dirPins[] = {dirPin1, dirPin2, dirPin3};
int enablePins[] = {enablePin1, enablePin2, enablePin3};
int stepPins[] = {stepPin1, stepPin2, stepPin3};

const int STEPS_PER_REVOLUTION = 200;
const int MICRO_STEPS = 32;
const int STEPS_PER_REV_WITH_MICRO_STEPPING = STEPS_PER_REVOLUTION * MICRO_STEPS;
const float MAX_RPM = 7.8125;
const float MAX_VELOCITY = 0.08; // [m/s]
const float MIN_PERIOD = 20; // [µs]

char serialString[50];
int nextWritePosition = 0;
boolean isNewFullCommand = false;
char command;
float parameters[3];

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(stepPins[i], OUTPUT);
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], HIGH); // Disable motors until needed (LOW = ENABLED)
  }
}

void loop() {
  receiveSerialData();
  if (isNewFullCommand == true) {
    if (parseCommand() >= 0)
      runCommand();
    else Serial.println("Could not parse the command");
  }
  delay(5);
}

void runCommand() {
  switch(command) {
    case 'R':
      rotate(parameters[0], parameters[1]);
      break;
    case 'D':
      moveDirectLine(parameters[0], parameters[1], parameters[2]);
      break;
  }
}

/**
  Drive each motor by sending the specified spike_periods until `duration` has passed.
  @param spike_period[1-3]:     A float specifiying HALF the time between two HIGHs sent to the corresponding motor, in microseconds [µs]
  @param duration:              An unsigned long specifying the duration of the spike pulses, in microseconds [µs]
*/
void driveMotors(float spike_period1, float spike_period2, float spike_period3, unsigned long duration) {
  unsigned long last_state_change1 = 0;
  unsigned long last_state_change2 = 0;
  unsigned long last_state_change3 = 0;
  // boolean state1 = LOW;
  // boolean state2 = LOW;
  // boolean state3 = LOW;

  for (int i = 0; i < 3; i++) {
    digitalWrite(enablePins[i], LOW);
  }

  digitalWrite(dirPin1, spike_period1 >= 0 ? HIGH : LOW);
  digitalWrite(dirPin2, spike_period2 >= 0 ? HIGH : LOW);
  digitalWrite(dirPin3, spike_period3 >= 0 ? HIGH : LOW);

  // writing to dirPins and enablePins should happen at least 650 ns before first step pulse
  delayMicroseconds(1);

  spike_period1 = abs(spike_period1);
  spike_period2 = abs(spike_period2);
  spike_period3 = abs(spike_period3);

  long t_0 = micros();
  long loop_counter = 0;
  while (micros() - t_0 < duration) {
    if (micros() - last_state_change1 >= spike_period1) {
      SWT(PORTD, 3);
      last_state_change1 = micros();
    }
    if (micros() - last_state_change2 >= spike_period2) {
      SWT(PORTD, 6);
      last_state_change2 = micros();
    }
    if (micros() - last_state_change3 >= spike_period3) {
      SWT(PORTB, 1);
      last_state_change3 = micros();
    }
    loop_counter++;
  }

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], HIGH);

  Serial.print("Mean loop duration: ");
  Serial.println(duration * 1.0 / loop_counter);
}

/**
  Move in a direct line towards the given direction.
  @param relativeDirection: The relative radians angle towards which to move in a direct line
  @param distance:          How far the robot should move, in meters [m]
*/
void moveDirectLine(float relativeDirection, float velocity, float distance) {
  float velocity_wheel1 = cos(2.61799 - relativeDirection) * velocity;   // 150°
  float velocity_wheel2 = cos(0.523599 - relativeDirection) * velocity;  //  30°
  float velocity_wheel3 = cos(4.71239 - relativeDirection) * velocity;   // 270°

  float period_wheel1 = velocityToPWMPeriod(velocity_wheel1);
  float period_wheel2 = velocityToPWMPeriod(velocity_wheel2);
  float period_wheel3 = velocityToPWMPeriod(velocity_wheel3);

  Serial.print("wheel1 [µs]: ");
  Serial.println(period_wheel1);
  Serial.print("wheel2 [µs]: ");
  Serial.println(period_wheel2);
  Serial.print("wheel3 [µs]: ");
  Serial.println(period_wheel3);

  Serial.println((distance / velocity) * 1000000);
  unsigned long duration = (unsigned long) (distance / velocity) * 1000000; // [µs]

  // Serial.print("Duration [µs]: ");
  // Serial.println(duration);

  driveMotors(period_wheel1, period_wheel2, period_wheel3, duration);
}

void rotate(float relativeAngle, float angularVelocity) {
  float velocity = RADIUS * angularVelocity;
  float wheel_velocity = relativeAngle < 0 ? -velocity : velocity;
  float period = velocityToPWMPeriod(wheel_velocity);

  unsigned long duration = (unsigned long) (abs(relativeAngle) / angularVelocity) * 1000000; // [µs]

  driveMotors(period, period, period, duration);
}

/**
  Convert wheel velocity to PWM period in microseconds [µs]
*/
float velocityToPWMPeriod(float velocity_wheel) {
  if (velocity_wheel == 0) return 0;
  else if (velocity_wheel < 0) return -(1 / (abs(velocity_wheel) / MAX_VELOCITY)) * MIN_PERIOD;
  else return (1 / (velocity_wheel / MAX_VELOCITY)) * MIN_PERIOD;
}

void receiveSerialData() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '{') {
      nextWritePosition = 0;
      isNewFullCommand = false;
    } else if (c == '}')
      isNewFullCommand = true;
    else {
      if (nextWritePosition >= 50) {
        Serial.println("Command too long!");
        break;
      }
      serialString[nextWritePosition++] = c;
    }
  }
}

int parseCommand() {
  Serial.print("Parsing "); Serial.println(serialString);
  command = serialString[0];
  strtok(serialString, ";"); // throw away

  for (int i = 0; i < 3; i++) {
    char *param = strtok(NULL, ";");
    parameters[i] = atof(param);
  }

  isNewFullCommand = false;
  return 0;
}
