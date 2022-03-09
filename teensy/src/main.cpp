
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
#define ROTATION_PART 0.3

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

// Micro Step Define Pins
#define MS1 26
#define MS2 25
#define MS3 24
#define LED 27

int dirPins[] = {dirPin1, dirPin2, dirPin3};
int enablePins[] = {enablePin1, enablePin2, enablePin3};
int stepPins[] = {stepPin1, stepPin2, stepPin3};

int micro_steps;
float default_velocity;
float default_accelleration;
int steps_per_rev_with_micro_stepping;

boolean interactiveModeEnabled;

float velocity_wheel1;
float target_velocity_wheel1;
float velocity_wheel2;
float target_velocity_wheel2;
float velocity_wheel3;
float target_velocity_wheel3;

float acc_wheel1;
float acc_wheel2;
float acc_wheel3;

const int STEPS_PER_REVOLUTION = 200;

void setup() {
  Serial.begin(4000000);

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(LED, OUTPUT);

  for (int i = 0; i < 3; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(stepPins[i], OUTPUT);
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], HIGH); // Disable motors until needed (LOW = ENABLED)
    digitalWrite(stepPins[i], LOW);
  }

  // Load default values from EEPROM
  micro_steps = getMicrosteps();
  default_velocity = getVelocity();
  default_accelleration = getAccelleration();
  steps_per_rev_with_micro_stepping = STEPS_PER_REVOLUTION * micro_steps;
  setMicrostepPins(micro_steps);
}

void runCommand(Command command) {
  switch(command.type) {
    case 's':
      println(getVelocity());
      break;
    case 'a':
      println(getAccelleration());
      break;
    case 'S':
      setVelocity(command.parameters[0]);
      default_velocity = command.parameters[0];
      break;
    case 'A':
      setAccelleration(command.parameters[0]);
      default_accelleration = command.parameters[0];
      break;
    case 'M':
      setMicrosteps((int) command.parameters[0]);
      setMicrostepPins((int) command.parameters[0]);
      micro_steps = (int) command.parameters[0];
      steps_per_rev_with_micro_stepping = STEPS_PER_REVOLUTION * micro_steps;
      break;
    case 'm':
      println(getMicrosteps());
      break;
    case 'I':
      interactiveDriving();
      break;
  }
}

void runInteractiveCommand(Command command) {
  switch (command.type) {
    case 'I':
      update(command); break;
    case 'E':
      interactiveModeEnabled = false;
      break;
  }
}

void update(Command command) {
  float velocityParam = command.parameters[1];
  float angularVelocityParam = command.parameters[2];

  if (velocityParam > 1) velocityParam = 1;
  if (velocityParam < -1) velocityParam = -1;
  if (angularVelocityParam > 1) angularVelocityParam = 1;
  if (angularVelocityParam < -1) angularVelocityParam = -1;

  float direction = command.parameters[0];
  float velocity = velocityParam * default_velocity * (1 - ROTATION_PART);
  float angularVelocity = angularVelocityParam * default_velocity * ROTATION_PART;

  target_velocity_wheel1 = cos(2.61799 - direction) * velocity + angularVelocity;
  target_velocity_wheel2 = cos(0.523599 - direction) * velocity + angularVelocity;
  target_velocity_wheel3 = cos(4.71239 - direction) * velocity + angularVelocity;

  float delta_v_wheel1 = abs(target_velocity_wheel1 - velocity_wheel1);
  float delta_v_wheel2 = abs(target_velocity_wheel2 - velocity_wheel2);
  float delta_v_wheel3 = abs(target_velocity_wheel3 - velocity_wheel3);

  acc_wheel1 = default_accelleration;
  acc_wheel2 = default_accelleration;
  acc_wheel3 = default_accelleration;

  if (delta_v_wheel1 != 0 && delta_v_wheel1 >= delta_v_wheel2 && delta_v_wheel1 >= delta_v_wheel3) {
    acc_wheel2 = default_accelleration * (delta_v_wheel2 / delta_v_wheel1);
    acc_wheel3 = default_accelleration * (delta_v_wheel3 / delta_v_wheel1);
  } else if (delta_v_wheel2 != 0 && delta_v_wheel2 >= delta_v_wheel1 && delta_v_wheel2 >= delta_v_wheel3) {
    acc_wheel1 = default_accelleration * (delta_v_wheel1 / delta_v_wheel2);
    acc_wheel3 = default_accelleration * (delta_v_wheel3 / delta_v_wheel2);
  } else if (delta_v_wheel3 != 0) {
    acc_wheel1 = default_accelleration * (delta_v_wheel1 / delta_v_wheel3);
    acc_wheel2 = default_accelleration * (delta_v_wheel2 / delta_v_wheel3);
  }
}

void handleSerial() {
  if (Serial.available() > 0) {
    boolean fullCommandReceived = receiveSerialData();
    if (fullCommandReceived) {
      Command command = parseCommand();
      if (command.valid)
        runInteractiveCommand(command);
      else Serial.println("Could not parse the command");
    }
  }
}

void interactiveDriving() {
  Serial.println("Entered interactive mode");
  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], LOW);

  interactiveModeEnabled = true;

  long last_state_change1 = 0;
  velocity_wheel1 = 0;
  target_velocity_wheel1 = 0;
  float abs_spike_period_wheel1 = velocityToPWMPeriod(velocity_wheel1) / 2;

  long last_state_change2 = 0;
  velocity_wheel2 = 0;
  target_velocity_wheel2 = 0;
  float abs_spike_period_wheel2 = velocityToPWMPeriod(velocity_wheel2) / 2;

  long last_state_change3 = 0;
  velocity_wheel3 = 0;
  target_velocity_wheel3 = 0;
  float abs_spike_period_wheel3 = velocityToPWMPeriod(velocity_wheel3) / 2;

  long t_0 = micros();
  long last_velocity_update = micros();
  unsigned long loop_counter = 0;
  while (interactiveModeEnabled) {
    handleSerial();
    loop_counter++;

    if (micros() - last_state_change1 >= abs_spike_period_wheel1) {
      if (velocity_wheel1 != 0) {
        GPIOA_PDOR ^= 1 << 12; // A12 = Pin 3
        last_state_change1 = micros();
      }
    }

    if (micros() - last_state_change2 >= abs_spike_period_wheel2) {
      if (velocity_wheel2 != 0) {
        GPIOD_PDOR ^= 1 << 4;
        last_state_change2 = micros();
      }
    }

    if (micros() - last_state_change3 >= abs_spike_period_wheel3) {
      if (velocity_wheel3 != 0) {
        GPIOC_PDOR ^= 1 << 3;
        last_state_change3 = micros();
      }
    }

    // update the velocities every 50ms
    if (micros() - last_velocity_update > 50000) {
      last_velocity_update = micros();

      if (velocity_wheel1 < target_velocity_wheel1) {
        velocity_wheel1 += acc_wheel1 / 20;
        if (velocity_wheel1 > target_velocity_wheel1) {
          velocity_wheel1 = target_velocity_wheel1;
        }
        abs_spike_period_wheel1 = abs(velocityToPWMPeriod(velocity_wheel1)) / 2;
      } else if (velocity_wheel1 > target_velocity_wheel1) {
        velocity_wheel1 -= acc_wheel1 / 20;
        if (velocity_wheel1 < target_velocity_wheel1) velocity_wheel1 = target_velocity_wheel1;
        abs_spike_period_wheel1 = abs(velocityToPWMPeriod(velocity_wheel1)) / 2;
      }

      if (velocity_wheel1 > 0) GPIOA_PDOR |= 1 << 13; // A13 = Pin 4
      else GPIOA_PDOR &= ~(1 << 13); // A13 = Pin 4


      if (velocity_wheel2 < target_velocity_wheel2) {
        velocity_wheel2 += acc_wheel2 / 20;
        if (velocity_wheel2 > target_velocity_wheel2) {
          velocity_wheel2 = target_velocity_wheel2;
        }
        abs_spike_period_wheel2 = abs(velocityToPWMPeriod(velocity_wheel2)) / 2;
      } else if (velocity_wheel2 > target_velocity_wheel2) {
        velocity_wheel2 -= acc_wheel2 / 20;
        if (velocity_wheel2 < target_velocity_wheel2) velocity_wheel2 = target_velocity_wheel2;
        abs_spike_period_wheel2 = abs(velocityToPWMPeriod(velocity_wheel2)) / 2;
      }

      if (velocity_wheel2 > 0) GPIOD_PDOR |= 1 << 2;
      else GPIOD_PDOR &= ~(1 << 2);


      if (velocity_wheel3 < target_velocity_wheel3) {
        velocity_wheel3 += acc_wheel3 / 20;
        if (velocity_wheel3 > target_velocity_wheel3) {
          velocity_wheel3 = target_velocity_wheel3;
        }
        abs_spike_period_wheel3 = abs(velocityToPWMPeriod(velocity_wheel3)) / 2;
      } else if (velocity_wheel3 > target_velocity_wheel3) {
        velocity_wheel3 -= acc_wheel3 / 20;
        if (velocity_wheel3 < target_velocity_wheel3) velocity_wheel3 = target_velocity_wheel3;
        abs_spike_period_wheel3 = abs(velocityToPWMPeriod(velocity_wheel3)) / 2;
      }

      if (velocity_wheel3 > 0) GPIOC_PDOR |= 1 << 4;
      else GPIOC_PDOR &= ~(1 << 4);
    }
  }

  Serial.println("Leaving interactive mode");
  println("mean loop time[µs]: ", (micros() - t_0 * 1.0) / loop_counter);

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], HIGH);
}

/**
  Convert wheel velocity to PWM period in microseconds [µs]
*/
float velocityToPWMPeriod(float velocity_wheel) {
  if (velocity_wheel == 0) return 0;
  else return 1000000 / (velocity_wheel * MOTOR_REVS_PER_SECOND_BY_METERS_PER_SECOND * steps_per_rev_with_micro_stepping);
}

float PWMPeriodToVelocity(float pwmPeriod) {
  if (pwmPeriod == 0) return 0;
  else return 1000000 / (pwmPeriod * MOTOR_REVS_PER_SECOND_BY_METERS_PER_SECOND * steps_per_rev_with_micro_stepping);
}

void setMicrostepPins(int micro_steps) {
  switch (micro_steps) {
    case 1:
      digitalWrite(MS1, LOW); digitalWrite(MS2, LOW); digitalWrite(MS3, LOW); break;
    case 2:
      digitalWrite(MS1, HIGH); digitalWrite(MS2, LOW); digitalWrite(MS3, LOW); break;
    case 4:
      digitalWrite(MS1, LOW); digitalWrite(MS2, HIGH); digitalWrite(MS3, LOW); break;
    case 8:
      digitalWrite(MS1, HIGH); digitalWrite(MS2, HIGH); digitalWrite(MS3, LOW); break;
    case 16:
      digitalWrite(MS1, LOW); digitalWrite(MS2, LOW); digitalWrite(MS3, HIGH); break;
    case 32:
      digitalWrite(MS1, HIGH); digitalWrite(MS2, HIGH); digitalWrite(MS3, HIGH); break;
    default:
      println("Invalid micro steps: ", micro_steps);
  }
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
