
#include "Arduino.h"

void rotate(unsigned int spike_period1, unsigned int spike_period2, unsigned int spike_period3);
void moveDirectLine(float relativeDirection, float distance);
float speedToPWMPeriod(float speed_wheel1);

// Macros for faster digital output
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

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
const float MAX_SPEED = 0.08; // [m/s]
const float MIN_PERIOD = 20; // [µs]

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
  Serial.println("Enter direction");
  while (Serial.available() <= 0) {
    delay(10);
  }
  float direction = Serial.parseFloat();
  while (Serial.available() > 0) Serial.read(); // flush

  // 0.0174533: Conversion factor from degrees to radians
  moveDirectLine(direction * 0.0174533, 0.32); // always move 32cm
}

/**
  Rotate each motor by sending the specified spike_periods until `duration` has passed.
  @param spike_period[1-3]:     A float specifiying the time between two HIGHs sent to the corresponding motor, in microseconds [µs]
  @param duration:              An unsigned long specifying the duration of the spike pulses, in microseconds [µs]
*/
void rotate(float spike_period1, float spike_period2, float spike_period3, unsigned long duration) {
  unsigned int num_steps1 = 0;
  unsigned int num_steps2 = 0;
  unsigned int num_steps3 = 0;
  unsigned long last_state_change1 = 0;
  unsigned long last_state_change2 = 0;
  unsigned long last_state_change3 = 0;
  boolean state1 = LOW;
  boolean state2 = LOW;
  boolean state3 = LOW;

  for (int i = 0; i < 3; i++) {
    digitalWrite(enablePins[i], LOW);
  }
  delayMicroseconds(2);

  digitalWrite(dirPin1, spike_period1 >= 0 ? HIGH : LOW);
  digitalWrite(dirPin2, spike_period2 >= 0 ? HIGH : LOW);
  digitalWrite(dirPin3, spike_period3 >= 0 ? HIGH : LOW);

  spike_period1 = abs(spike_period1);
  spike_period2 = abs(spike_period2);
  spike_period3 = abs(spike_period3);

  long t_0 = micros();
  while (micros() - t_0 < duration) {
    if (state1 != HIGH && micros() - last_state_change1 >= spike_period1) {
      SET(PORTD, 3);
      last_state_change1 = micros();
      state1 = HIGH;
      num_steps1++;
    } else if (state1 == HIGH && micros() - last_state_change1 >= spike_period1) {
      CLR(PORTD, 3);
      last_state_change1 = micros();
      state1 = LOW;
    }
    if (state2 != HIGH && micros() - last_state_change2 >= spike_period2) {
      SET(PORTD, 6);
      last_state_change2 = micros();
      state2 = HIGH;
      num_steps2++;
    } else if (state2 == HIGH && micros() - last_state_change2 >= spike_period2) {
      CLR(PORTD, 6);
      last_state_change2 = micros();
      state2 = LOW;
    }
    if (state3 != HIGH && micros() - last_state_change3 >= spike_period3) {
      SET(PORTB, 1);
      last_state_change3 = micros();
      state3 = HIGH;
      num_steps3++;
    } else if (state3 == HIGH && micros() - last_state_change3 >= spike_period3) {
      CLR(PORTB, 1);
      last_state_change3 = micros();
      state3 = LOW;
    }
  }

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], HIGH);
}

/**
  Move in a direct line towards the given direction.
  @param relativeDirection: The relative radians angle towards which to move in a direct line
  @param distance:          How far the robot should move, in meters [m]
*/
void moveDirectLine(float relativeDirection, float distance) {
  float speed_wheel1 = cos(2.61799 - relativeDirection) * MAX_SPEED;   // 150°
  float speed_wheel2 = cos(0.523599 - relativeDirection) * MAX_SPEED;  //  30°
  float speed_wheel3 = cos(4.71239 - relativeDirection) * MAX_SPEED;   // 270°

  float period_wheel1 = speedToPWMPeriod(speed_wheel1);
  float period_wheel2 = speedToPWMPeriod(speed_wheel2);
  float period_wheel3 = speedToPWMPeriod(speed_wheel3);

  Serial.print("wheel1 [µs]: ");
  Serial.println(period_wheel1);
  Serial.print("wheel2 [µs]: ");
  Serial.println(period_wheel2);
  Serial.print("wheel3 [µs]: ");
  Serial.println(period_wheel3);

  const unsigned long duration = (unsigned long) (distance / MAX_SPEED) * 1000000; // [µs]

  Serial.print("Duration [µs]:");
  Serial.println(duration);

  rotate(period_wheel1, period_wheel2, period_wheel3, duration);
}

/**
  Convert wheel speed to PWM period in microseconds [µs]
*/
float speedToPWMPeriod(float speed_wheel) {
  if (speed_wheel == 0) return 0;
  else if (speed_wheel < 0) return -(1 / (abs(speed_wheel) / MAX_SPEED)) * MIN_PERIOD;
  else return (1 / (speed_wheel / MAX_SPEED)) * MIN_PERIOD;
}
