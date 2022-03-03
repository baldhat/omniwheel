
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

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(stepPins[i], OUTPUT);
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], HIGH); // Disable motors until needed (LOW = ENABLED)
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

/**
  Drive each motor by sending the specified spike_periods until `duration` has passed.
  @param spike_period[1-3]:     A float specifiying HALF the time between two HIGHs sent to the corresponding motor, in microseconds [µs]
  @param duration:              An unsigned long specifying the duration of the spike pulses, in microseconds [µs]
*/
void driveMotors(float spike_period1, float spike_period2, float spike_period3, unsigned long duration) {
  digitalWrite(dirPin1, spike_period1 < 0 ? LOW : HIGH );
  digitalWrite(dirPin2, spike_period2 < 0 ? LOW : HIGH );
  digitalWrite(dirPin3, spike_period3 < 0 ? LOW : HIGH );

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], LOW);

  float abs_spike_period1 = abs(spike_period1);
  float abs_spike_period2 = abs(spike_period2);
  float abs_spike_period3 = abs(spike_period3);

  long last_state_change1 = 0;
  long last_state_change2 = 0;
  long last_state_change3 = 0;

  boolean state1 = false;
  boolean state2 = false;
  boolean state3 =false;

  Serial.print("Duration: "); Serial.println(duration);

  long steps1 = 0;
  long steps2 = 0;
  long steps3 = 0;

  long t_0 = micros();
  long loop_counter = 0;
  while (micros() - t_0 < duration) {
    if (micros() - last_state_change1 >= abs_spike_period1) {
      steps1++;
      state1 = !state1;
      digitalWrite(stepPin1, state1);
      last_state_change1 = micros();
    }
    if (micros() - last_state_change2 >= abs_spike_period2) {
      steps2++;
      state2 = !state2;
      digitalWrite(stepPin2, state2);
      last_state_change2 = micros();
    }
    if (micros() - last_state_change3 >= abs_spike_period3) {
      steps3++;
      state3 = !state3;
      digitalWrite(stepPin3, state3);
      last_state_change3 = micros();
    }
    loop_counter++;
  }

  println("Mean loop time: ", duration * 1.0 / loop_counter);
  println("Steps motor 1: ", steps1 / 2);
  println("Steps motor 2: ", steps2 / 2);
  println("Steps motor 3: ", steps3 / 2);

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], HIGH);
}

void driveMotorsAccled(float direction, float distance, float velocity, float accelleration) {

  if (velocity == 0) velocity = getVelocity();
  if (accelleration == 0) accelleration = getAccelleration();

  println("Using velocity: ", velocity);
  println("Using accelleration: ", accelleration);

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], LOW);

  long last_state_change1 = 0;
  long last_state_change2 = 0;
  long last_state_change3 = 0;

  boolean state1 = false;
  boolean state2 = false;
  boolean state3 = false;

  long steps1 = 0;
  long steps2 = 0;
  long steps3 = 0;

  float revs = REVS_PER_METER * distance;
  long num_steps1 = abs(cos(2.61799 - direction)) * STEPS_PER_REV_WITH_MICRO_STEPPING * revs;
  long num_steps2 = abs(cos(0.523599 - direction)) * STEPS_PER_REV_WITH_MICRO_STEPPING * revs;
  long num_steps3 = abs(cos(4.71239 - direction)) * STEPS_PER_REV_WITH_MICRO_STEPPING * revs;

  float wheel1_target_velocity = cos(2.61799 - direction) * velocity;
  digitalWrite(dirPin1, wheel1_target_velocity > 0);
  float wheel1_velocity = PWMPeriodToVelocity(1000);
  float abs_spike_period1 = 1000;
  float abs_wheel1_target_velocity = abs(wheel1_target_velocity);

  float wheel2_target_velocity = cos(0.523599 - direction) * velocity;
  digitalWrite(dirPin2, wheel2_target_velocity > 0);
  float wheel2_velocity = PWMPeriodToVelocity(1000);
  float abs_spike_period2 = 1000;
  float abs_wheel2_target_velocity = abs(wheel2_target_velocity);

  float wheel3_target_velocity = cos(4.71239 - direction) * velocity;
  digitalWrite(dirPin3, wheel3_target_velocity > 0);
  float wheel3_velocity = PWMPeriodToVelocity(1000);
  float abs_spike_period3 = 1000;
  float abs_wheel3_target_velocity = abs(wheel3_target_velocity);

  println("Starting velocity1: ", wheel1_velocity);
  println("Starting spike period1: ", abs_spike_period1);
  println("Target velocity1: ", wheel1_target_velocity);
  println("Starting velocity2: ", wheel2_velocity);
  println("Starting spike period2: ", abs_spike_period2);
  println("Target velocity2: ", wheel2_target_velocity);
  println("Starting velocity3: ", wheel3_velocity);
  println("Starting spike period3: ", abs_spike_period3);
  println("Target velocity3: ", wheel3_target_velocity);

  long loop_counter = 0;
  long t_0 = millis();
  while (steps1 < num_steps1) {

    long time_since_last_change1 = micros() - last_state_change1;
    if (time_since_last_change1 >= abs_spike_period1) {
      steps1++;
      state1 = !state1;
      digitalWrite(stepPin1, state1);
      last_state_change1 = micros() - (time_since_last_change1 - abs_spike_period1);

      if (wheel1_velocity < abs_wheel1_target_velocity) {
        wheel1_velocity += (accelleration * abs(cos(2.61799 - direction))) / (1000000 / abs_spike_period1);
        if (wheel1_velocity > abs_wheel1_target_velocity) wheel1_velocity = abs_wheel1_target_velocity;
        abs_spike_period1 = abs(velocityToPWMPeriod(wheel1_velocity));
      }
    }

    long time_since_last_change2 = micros() - last_state_change2;
    if (time_since_last_change2 >= abs_spike_period2) {
      steps2++;
      state2 = !state2;
      digitalWrite(stepPin2, state2);
      last_state_change2 = micros() - (time_since_last_change2 - abs_spike_period2);

      if (wheel2_velocity < abs_wheel2_target_velocity) {
        wheel2_velocity += (accelleration * abs(cos(2.61799 - direction))) / (1000000 / abs_spike_period2);
        if (wheel2_velocity > abs_wheel2_target_velocity) wheel2_velocity = abs_wheel2_target_velocity;
        abs_spike_period2 = abs(velocityToPWMPeriod(wheel2_velocity));
      }
    }

    long time_since_last_change3 = micros() - last_state_change3;
    if (time_since_last_change3 >= abs_spike_period3) {
      steps3++;
      state3 = !state3;
      digitalWrite(stepPin3, state3);
      last_state_change3 = micros() - (time_since_last_change3 - abs_spike_period3);

      if (wheel3_velocity < abs_wheel3_target_velocity) {
        wheel3_velocity += (accelleration * abs(cos(2.61799 - direction))) / (1000000 / abs_spike_period3);
        if (wheel3_velocity > abs_wheel3_target_velocity) wheel3_velocity = abs_wheel3_target_velocity;
        abs_spike_period3 = abs(velocityToPWMPeriod(wheel3_velocity));
      }
    }
    loop_counter++;
  }

  long duration = millis() - t_0;
  println("Duration: ", duration);

  println("Final velocity: ", wheel1_velocity);
  println("Final spike period ", abs_spike_period1);
  println("Mean loop duration: ", duration * 1000.0 / loop_counter);

  println("steps 1: ", steps1);
  println("Wanted steps: ", num_steps1);
  println("steps 2: ", steps2);
  println("Wanted steps: ", num_steps2);
  println("steps 3: ", steps3);
  println("Wanted steps: ", num_steps3);

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], HIGH);
}

/**
  Move in a direct line towards the given direction.
  @param relativeDirection: The relative radians angle towards which to move in a direct line
  @param distance:          How far the robot should move, in meters [m]
*/
void moveDirectLine(float relativeDirection, float distance, float velocity) {
  if (velocity == 0)
    velocity = getVelocity();

  float velocity_wheel1 = cos(2.61799 - relativeDirection) * velocity;   // 150°
  float velocity_wheel2 = cos(0.523599 - relativeDirection) * velocity;  //  30°
  float velocity_wheel3 = cos(4.71239 - relativeDirection) * velocity;   // 270°

  float period_wheel1 = velocityToPWMPeriod(velocity_wheel1);
  float period_wheel2 = velocityToPWMPeriod(velocity_wheel2);
  float period_wheel3 = velocityToPWMPeriod(velocity_wheel3);

  Serial.println((distance / velocity) * 1000000);
  unsigned long duration = (unsigned long) (distance / velocity) * 1000000; // [µs]

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
