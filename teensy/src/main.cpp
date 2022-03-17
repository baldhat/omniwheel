
#include "Arduino.h"

#include "main.h"
#include "parser.h"
#include "configuration.h"
#include "helper.h"

#define MOTOR_REVS_PER_SECOND_BY_METERS_PER_SECOND 48 // grob
#define ROTATION_PART 0.3
#define STEPS_PER_REVOLUTION 200

// Micro Step Define Pins
#define MS1 26
#define MS2 25
#define MS3 24

int enablePins[] = {2, 5, 8};

int micro_steps;
float default_velocity;
float default_accelleration;
int steps_per_rev_with_micro_stepping;

boolean interactiveModeEnabled;

float wheel_velocities[3];
float target_velocities[3];
float accellerations[3];
float wheel_phases[3] = {2.61799, 0.523599, 4.71239};

void setup() {
  Serial.begin(4000000);

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  for (int i = 2; i < 11; i++) pinMode(i, OUTPUT); // Set all driver pins to OUTPUT
  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], HIGH); // Disable all motors

  loadDefaultValues();
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
      updateTargets(command); break;
    case 'E':
      interactiveModeEnabled = false;
      break;
  }
}

void updateTargets(Command command) {
  float velocityParam = command.parameters[1];
  float angularVelocityParam = command.parameters[2];

  if (velocityParam > 1) velocityParam = 1;
  if (velocityParam < -1) velocityParam = -1;
  if (angularVelocityParam > 1) angularVelocityParam = 1;
  if (angularVelocityParam < -1) angularVelocityParam = -1;

  float direction = command.parameters[0];
  float velocity = velocityParam * default_velocity * (1 - ROTATION_PART);
  float angularVelocity = angularVelocityParam * default_velocity * ROTATION_PART;

  float delta_vs[3];

  for (int i = 0; i < 3; i++) {
    target_velocities[i] = cos(wheel_phases[i] - direction) * velocity + angularVelocity;
    delta_vs[i] = abs(target_velocities[i] - wheel_velocities[i]);
    accellerations[i] = default_accelleration;
  }

  if (delta_vs[0] != 0 && delta_vs[0] >= delta_vs[1] && delta_vs[0] >= delta_vs[2]) {
    accellerations[1] = default_accelleration * (delta_vs[1] / delta_vs[0]);
    accellerations[2] = default_accelleration * (delta_vs[2] / delta_vs[0]);
  } else if (delta_vs[1] != 0 && delta_vs[1] >= delta_vs[0] && delta_vs[1] >= delta_vs[2]) {
    accellerations[0] = default_accelleration * (delta_vs[0] / delta_vs[1]);
    accellerations[2] = default_accelleration * (delta_vs[2] / delta_vs[1]);
  } else if (delta_vs[2] != 0) {
    accellerations[0] = default_accelleration * (delta_vs[0] / delta_vs[2]);
    accellerations[1] = default_accelleration * (delta_vs[1] / delta_vs[2]);
  }
}

void handleSerial() {
  boolean fullCommandReceived = receiveSerialData();
  if (fullCommandReceived) {
    Command command = parseCommand();
    if (command.valid)
      runInteractiveCommand(command);
    else Serial.println("Could not parse the command");
  }
}

void interactiveDriving() {
  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], LOW);

  interactiveModeEnabled = true;

  long last_state_changes[3];
  float abs_spike_periods[3];
  long steps[3];

  for (int i = 0; i < 3; i++) {
    last_state_changes[i] = 0;
    wheel_velocities[i] = 0;
    target_velocities[i] = 0;
    steps[i] = 0;
    abs_spike_periods[i] = velocityPWMConversion(wheel_velocities[i]) / 2;
  }

  long last_velocity_update = micros();
  unsigned long loop_counter = 0;

  while (interactiveModeEnabled) {
    if (Serial.available() > 0) {
      handleSerial();
    }
    loop_counter++;

    if (micros() - last_state_changes[0] >= abs_spike_periods[0] && wheel_velocities[0] != 0) {
      GPIO9_DR_TOGGLE ^= 1 << 5; // Pin 3
      last_state_changes[0] = micros();
      if (wheel_velocities[0] > 0) steps[0]++; else steps[0]--;
    }

    if (micros() - last_state_changes[1] >= abs_spike_periods[1] && wheel_velocities[1] != 0) {
      GPIO7_DR_TOGGLE ^= 1 << 10; // Pin 6
      last_state_changes[1] = micros();
      if (wheel_velocities[1] > 0) steps[1]++; else steps[1]--;
    }

    if (micros() - last_state_changes[2] >= abs_spike_periods[2] && wheel_velocities[2] != 0) {
      GPIO7_DR_TOGGLE ^= 1 << 11; // Pin 9
      last_state_changes[2] = micros();
      if (wheel_velocities[2] > 0) steps[2]++; else steps[2]--;
    }

    if (micros() - last_velocity_update > 50000) {
      last_velocity_update = micros();
      updateVelocities(abs_spike_periods);
      setDirectionPins();
      sendSteps(steps);
      for (int i = 0; i < 3; i++) steps[i] = 0;
    }
  }

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], HIGH);
}

void updateVelocities(float abs_spike_periods[3]) {
  for (int i = 0; i < 3;  i++) {
    if (wheel_velocities[i] < target_velocities[i]) {
      wheel_velocities[i] += accellerations[i] / 20;
      if (wheel_velocities[i] > target_velocities[i]) {
        wheel_velocities[i] = target_velocities[i];
      }
      abs_spike_periods[i] = abs(velocityPWMConversion(wheel_velocities[i])) / 2;
    } else if (wheel_velocities[i] > target_velocities[i]) {
      wheel_velocities[i] -= accellerations[i] / 20;
      if (wheel_velocities[i] < target_velocities[i]) wheel_velocities[i] = target_velocities[i];
      abs_spike_periods[i] = abs(velocityPWMConversion(wheel_velocities[i])) / 2;
    }
  }
}

void setDirectionPins() {
  // if (wheel_velocities[0] > 0) GPIO9_DR_TOGGLE |= 1 << 6; // Pin 4
  // else GPIO9_DR_TOGGLE &= ~(1 << 6);
  //
  // if (wheel_velocities[1] > 0) GPIO7_DR_TOGGLE |= 1 << 17; // Pin 7
  // else GPIO7_DR_TOGGLE &= ~(1 << 17);
  //
  // if (wheel_velocities[2] > 0) GPIO7_DR_TOGGLE |= 1; // Pin 10
  // else GPIO7_DR_TOGGLE &= ~1;
  digitalWriteFast(4, wheel_velocities[0] > 0);
  digitalWriteFast(7, wheel_velocities[1] > 0);
  digitalWriteFast(10, wheel_velocities[2] > 0);
}

void sendSteps(long steps[3]) {
  Serial.print("{");
  Serial.print(steps[0]); Serial.print(";");
  Serial.print(steps[1]); Serial.print(";");
  Serial.print(steps[2]);
  Serial.println("}");
}

/**
  Convert wheel velocity to PWM period in microseconds [µs] and reverse
*/
float velocityPWMConversion(float value) {
  if (value == 0) return 0;
  else return 1000000 / (value * MOTOR_REVS_PER_SECOND_BY_METERS_PER_SECOND * steps_per_rev_with_micro_stepping);
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

void loadDefaultValues() {
  micro_steps = getMicrosteps();
  default_velocity = getVelocity();
  default_accelleration = getAccelleration();
  steps_per_rev_with_micro_stepping = STEPS_PER_REVOLUTION * micro_steps;
  setMicrostepPins(micro_steps);
}

void loop() {
  boolean fullCommandReceived = receiveSerialData();
  if (fullCommandReceived) {
    Command command = parseCommand();
    if (command.valid) {
      runCommand(command);
    } else Serial.println("Could not parse the command");
  }
  delay(1000);
}
