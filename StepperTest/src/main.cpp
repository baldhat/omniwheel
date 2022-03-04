
#include "Arduino.h"

#include "main.h"
#include "parser.h"
#include "configuration.h"
#include "helper.h"

#define RADIUS 0.15
#define REVS_PER_METER 4
#define MOTOR_REVS_PER_WHEEL_REV 7 // grob
#define DIST_PER_WHEEL_REV 0.25  //m
#define MOTOR_REVS_PER_SECOND_BY_METERS_PER_SECOND 28 // grob

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

byte diffs[100000];

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(stepPins[i], OUTPUT);
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], HIGH); // Disable motors until needed (LOW = ENABLED)
    digitalWrite(stepPins[i], LOW);
  }
}

void runCommand(Command command) {
  switch(command.type) {
    case 'R':
      rotate(command.parameters[0], command.parameters[1]);
      break;
    case 'D':
      driveMotorsAccled(command.parameters[0], command.parameters[1], command.parameters[2], getAccelleration());
      break;
    case 's':
      printVelocity();
      break;
    case 'a':
      printAccelleration();
      break;
    case 'S':
      setVelocity(command.parameters[0]);
      break;
    case 'A':
      setAccelleration(command.parameters[0]);
      break;
  }
}

void driveDirectLine(float direction, float distance, float velocity, float accelleration) {

  if (velocity == 0) velocity = getVelocity();
  if (accelleration == 0) accelleration = getAccelleration();
  //
  // println("Using velocity: ", velocity);
  // println("Using accelleration: ", accelleration);

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], LOW);

  long last_state_change1 = 0;
  long last_state_change2 = 0;
  long last_state_change3 = 0;

  long steps1 = 0;
  long steps2 = 0;
  long steps3 = 0;

  float revs = REVS_PER_METER * MOTOR_REVS_PER_WHEEL_REV * distance;
  long num_steps1 = abs(cos(2.61799 - direction)) * STEPS_PER_REV_WITH_MICRO_STEPPING * revs * 2;
  long num_steps2 = abs(cos(0.523599 - direction)) * STEPS_PER_REV_WITH_MICRO_STEPPING * revs * 2;
  long num_steps3 = abs(cos(4.71239 - direction)) * STEPS_PER_REV_WITH_MICRO_STEPPING * revs * 2;

  float wheel1_target_velocity = cos(2.61799 - direction) * velocity;
  float abs_wheel1_target_velocity = abs(wheel1_target_velocity);
  digitalWrite(dirPin1, wheel1_target_velocity > 0);
  float wheel1_velocity = 0.001;
  float abs_spike_period1 = velocityToPWMPeriod(wheel1_velocity) / 2;

  float wheel2_target_velocity = cos(0.523599 - direction) * velocity;
  float abs_wheel2_target_velocity = abs(wheel2_target_velocity);
  digitalWrite(dirPin2, wheel2_target_velocity > 0);
  float wheel2_velocity = 0.001;
  float abs_spike_period2 = velocityToPWMPeriod(wheel2_velocity) / 2;

  float wheel3_target_velocity = cos(4.71239 - direction) * velocity;
  float abs_wheel3_target_velocity = abs(wheel3_target_velocity);
  digitalWrite(dirPin3, wheel3_target_velocity > 0);
  float wheel3_velocity = 0.001;
  float abs_spike_period3 = velocityToPWMPeriod(wheel3_velocity) / 2;

  long ramp_down_begin_step1 = -1;
  long ramp_down_begin_step2 = -1;
  long ramp_down_begin_step3 = -1;

  // println("Starting velocity1: ", wheel1_velocity);
  // println("Starting spike period1: ", abs_spike_period1);
  // println("Target velocity1: ", abs_wheel1_target_velocity);
  // println("Starting velocity2: ", wheel2_velocity);
  // println("Starting spike period2: ", abs_spike_period2);
  // println("Target velocity2: ", abs_wheel2_target_velocity);
  // println("Starting velocity3: ", wheel3_velocity);
  // println("Starting spike period3: ", abs_spike_period3);
  // println("Target velocity3: ", abs_wheel3_target_velocity);

  long loop_counter = 0;
  long t_0 = millis();
  while (steps1 < num_steps1) {

    long time_since_last_change1 = micros() - last_state_change1;
    if (time_since_last_change1 >= abs_spike_period1) {
      steps1++;
      GPIOA_PDOR ^= 1 << 12; // A12 = 3
      last_state_change1 = micros();

      if (ramp_down_begin_step1 == steps1 || (ramp_down_begin_step1 == -1 && steps1 == num_steps1 / 2))
        abs_wheel1_target_velocity = 0.0;

      if (wheel1_velocity < abs_wheel1_target_velocity && abs_spike_period1 != 0) {
        wheel1_velocity += (accelleration * abs(cos(2.61799 - direction))) / (1000000 / abs_spike_period1);
        if (wheel1_velocity > abs_wheel1_target_velocity) {
          wheel1_velocity = abs_wheel1_target_velocity;
          ramp_down_begin_step1 = num_steps1 - steps1; // We have reach cruising speed, it will take just as many steps to decellerate
        }
        abs_spike_period1 = abs(velocityToPWMPeriod(wheel1_velocity)) / 2;
      } else if (wheel1_velocity > abs_wheel1_target_velocity) {
        wheel1_velocity -= (accelleration * abs(cos(2.61799 - direction))) / (1000000 / abs_spike_period1);
        if (wheel1_velocity < abs_wheel1_target_velocity) wheel1_velocity = abs_wheel1_target_velocity;
        abs_spike_period1 = abs(velocityToPWMPeriod(wheel1_velocity)) / 2;
      }
    }

    long time_since_last_change2 = micros() - last_state_change2;
    if (time_since_last_change2 >= abs_spike_period2 && abs_spike_period2 != 0) {
      steps2++;
      GPIOD_PDOR ^= 1 << 4; // D4 = 6
      last_state_change2 = micros();

      if (ramp_down_begin_step2 == steps2 || (ramp_down_begin_step2 == -1 && steps2 == num_steps2 / 2))
          abs_wheel2_target_velocity = 0.0;

      if (wheel2_velocity < abs_wheel2_target_velocity) {
        wheel2_velocity += (accelleration * abs(cos(0.523599 - direction))) / (1000000 / abs_spike_period2);
        if (wheel2_velocity > abs_wheel2_target_velocity) {
          wheel2_velocity = abs_wheel2_target_velocity;
          ramp_down_begin_step2 = num_steps2 - steps2; // We have reach cruising speed, it will take just as many steps to decellerate
        }
        abs_spike_period2 = abs(velocityToPWMPeriod(wheel2_velocity)) / 2;
      } else if (wheel2_velocity > abs_wheel2_target_velocity) {
        wheel2_velocity -= (accelleration * abs(cos(0.523599 - direction))) / (1000000 / abs_spike_period2);
        if (wheel2_velocity < abs_wheel2_target_velocity) wheel2_velocity = abs_wheel2_target_velocity;
        abs_spike_period2 = abs(velocityToPWMPeriod(wheel2_velocity)) / 2;
      }
    }

    long time_since_last_change3 = micros() - last_state_change3;
    if (time_since_last_change3 >= abs_spike_period3 && abs_spike_period3 != 0) {
      steps3++;
      GPIOC_PDOR ^= 1 << 3; // C3 = 9
      last_state_change3 = micros();

      if (ramp_down_begin_step3 == steps3 || (ramp_down_begin_step3 == -1 && steps3 == num_steps3 / 2))
          abs_wheel3_target_velocity = 0.0;

      if (wheel3_velocity < abs_wheel3_target_velocity) {
        wheel3_velocity += (accelleration * abs(cos(4.71239 - direction))) / (1000000 / abs_spike_period3);
        if (wheel3_velocity > abs_wheel3_target_velocity) {
          wheel3_velocity = abs_wheel3_target_velocity;
          ramp_down_begin_step3 = num_steps3 - steps3; // We have reach cruising speed, it will take just as many steps to decellerate
        }
        abs_spike_period3 = abs(velocityToPWMPeriod(wheel3_velocity)) / 2;
      } else if (wheel3_velocity > abs_wheel3_target_velocity) {
        wheel3_velocity -= (accelleration * abs(cos(4.71239 - direction))) / (1000000 / abs_spike_period3);
        if (wheel3_velocity < abs_wheel3_target_velocity) wheel3_velocity = abs_wheel3_target_velocity;
        abs_spike_period3 = abs(velocityToPWMPeriod(wheel3_velocity)) / 2;
      }
    }
    loop_counter++;
  }

  // long duration = millis() - t_0;
  // println("Duration: ", duration);
  //
  // println("Final velocity1: ", wheel1_velocity);
  // println("Final velocity2: ", wheel2_velocity);
  // println("Final velocity3: ", wheel3_velocity);
  // println("Mean loop duration: ", duration * 1000.0 / loop_counter);
  //
  // println("steps 1: ", steps1);
  // println("Wanted steps: ", num_steps1);
  // println("steps 2: ", steps2);
  // println("Wanted steps: ", num_steps2);
  // println("steps 3: ", steps3);
  // println("Wanted steps: ", num_steps3);

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], HIGH);
}

void rotate(float relativeAngle, float angularVelocity) {
  float velocity = RADIUS * angularVelocity;
  float wheel_velocity = relativeAngle < 0 ? -velocity : velocity;
  float period = velocityToPWMPeriod(wheel_velocity);

  unsigned long duration = (unsigned long) (abs(relativeAngle) / angularVelocity) * 1000000; // [µs]

  //driveMotors(period, period, period, duration);
}

/**
  Convert wheel velocity to PWM period in microseconds [µs]
*/
float velocityToPWMPeriod(float velocity_wheel) {
  if (velocity_wheel == 0) return 0;
  else return 1000000 / (velocity_wheel * MOTOR_REVS_PER_SECOND_BY_METERS_PER_SECOND * STEPS_PER_REV_WITH_MICRO_STEPPING);
}

float PWMPeriodToVelocity(float pwmPeriod) {
  if (pwmPeriod == 0) return 0;
  else return 1000000 / (pwmPeriod * MOTOR_REVS_PER_SECOND_BY_METERS_PER_SECOND * STEPS_PER_REV_WITH_MICRO_STEPPING);
}

void loop() {
  boolean fullCommandReceived = receiveSerialData();
  if (fullCommandReceived) {
    Command command = parseCommand();
    if (command.valid)
      runCommand(command);
    else Serial.println("Could not parse the command");
  }
  delay(5);
}
