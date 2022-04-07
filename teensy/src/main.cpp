
#include "Arduino.h"

#include "main.h"
#include "parser.h"
#include "configuration.h"
#include "helper.h"

#define MOTOR_REVS_PER_SECOND_BY_METERS_PER_SECOND 48  // roughly
#define ROTATION_PART 0.3  // how much of the maximum wheel speed is reserved for rotation
#define STEPS_PER_REVOLUTION 200  // number of full steps for one motor revolution

#define VELOCITY_UPDATE_PERIOD_MICROSECONDS 50000  // 50 ms

// Pin numbers of the micro step selection pins 
#define MS1 26
#define MS2 25
#define MS3 24

// Analog input pin for the battery voltage divider.
#define VBAT 9
/*
  2.988V = 925 = 24.24V
  900 = 23.66V
*/

int enablePins[] = {2, 5, 8}; // Pin numbers of the enable pins of the stepper motor drivers

int micro_steps; // Current number of micro steps for the stepper motor (drivers)
float default_velocity; // maximum velocity a wheel can be driven at
float default_acceleration; // Acceleration of the robot
int steps_per_rev_with_micro_stepping; // number of steps per motor revolution

// Flag stating whether the robot is in the mode where it can receive drive commands
boolean drivingMode;

float wheel_velocities[3]; // Current velocities of the wheels
float target_velocities[3]; // Target velocities of the wheels, towards which the robot accelerates
float accelerations[3]; // Current accelerations of the wheels
float WHEEL_PHASES[3] = {2.61799, 0.523599, 4.71239};

void setup() {
  // The Teensy USB Serial ignores this number and just uses the maximum
  // available USB speed:
  Serial.begin(4000000);

  pinMode(MS1, OUTPUT); pinMode(MS2, OUTPUT); pinMode(MS3, OUTPUT);

  for (int i = 2; i < 11; i++) pinMode(i, OUTPUT); // Set all driver pins to OUTPUT
  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], HIGH); // Disable all motors

  loadDefaultValues();
}


/**
 * Execute a command. The meaning of the parameters depends upon the command and can be looked up in the wiki.
 */
void runCommand(Command command) {
  switch(command.type) {
    case 's':
      println(getVelocity()); break;
    case 'a':
      println(getAcceleration()); break;
    case 'S':
      setVelocity(command.parameters[0]);
      default_velocity = command.parameters[0];
      break;
    case 'A':
      setAcceleration(command.parameters[0]);
      default_acceleration = command.parameters[0];
      break;
    case 'M':
      changeMicroSteps(command);
      break;
    case 'm':
      println(getMicrosteps()); break;
    case 'b':
      println(getBatteryVoltage()); break;
    case 'I':
      drivingLoop(); break;

  }
}

/**
 * In driving mode only drive commands and battery level commands are allowed.
 */
void runInteractiveCommand(Command command) {
  switch (command.type) {
    case 'I':
      updateTargets(command); break;
    case 'E':
      drivingMode = false;
      break;
    case 'b':
      println(getBatteryVoltage()); break;
  }
}


/**
 * Change the current driving target_velocities and accelerations based on 
 * the given command.
 */
void updateTargets(Command command) {
  float direction = command.parameters[0];
  float velocityParam = command.parameters[1];
  float angularVelocityParam = command.parameters[2];

  // Clamp the velocities
  if (velocityParam > 1) velocityParam = 1;
  if (velocityParam < -1) velocityParam = -1;
  if (angularVelocityParam > 1) angularVelocityParam = 1;
  if (angularVelocityParam < -1) angularVelocityParam = -1;

  // Convert relative velocities to the actual velocity values
  float velocity = velocityParam * default_velocity * (1 - ROTATION_PART);
  float angularVelocity = angularVelocityParam * default_velocity * ROTATION_PART;

  // By how much do the new target velocities differ from the current wheel velocities
  float delta_vs[3];

  // Calculate new target velocities
  for (int i = 0; i < 3; i++) {
    target_velocities[i] = cos(WHEEL_PHASES[i] - direction) * velocity + angularVelocity;
    delta_vs[i] = abs(target_velocities[i] - wheel_velocities[i]);
    accelerations[i] = default_acceleration;
  }

  // The greatest delta_v gets the maximum acceleration, the rest get accelerations
  // proportional to their delta_v / max_delta_v. This ensures all the wheels finishing
  // acceleration at the same time. 
  if (delta_vs[0] != 0 && delta_vs[0] >= delta_vs[1] && delta_vs[0] >= delta_vs[2]) {
    accelerations[1] = default_acceleration * (delta_vs[1] / delta_vs[0]);
    accelerations[2] = default_acceleration * (delta_vs[2] / delta_vs[0]);
  } else if (delta_vs[1] != 0 && delta_vs[1] >= delta_vs[0] && delta_vs[1] >= delta_vs[2]) {
    accelerations[0] = default_acceleration * (delta_vs[0] / delta_vs[1]);
    accelerations[2] = default_acceleration * (delta_vs[2] / delta_vs[1]);
  } else if (delta_vs[2] != 0) {
    accelerations[0] = default_acceleration * (delta_vs[0] / delta_vs[2]);
    accelerations[1] = default_acceleration * (delta_vs[1] / delta_vs[2]);
  }
}

/**
 * Handle incoming serial messages. If they are a valid command, run them.
 */
void handleSerial() {
  boolean fullCommandReceived = receiveSerialData();
  if (fullCommandReceived) {
    Command command = parseCommand();
    if (command.valid)
      runInteractiveCommand(command);
    else Serial.println("Could not parse the command");
  }
}

/**
 * Drive the motors until the drivingMode is disabled. Update the wheel velocities according
 * to the commands read via serial.
 */
void drivingLoop() {
  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], LOW); // Enable motors

  drivingMode = true;

  long last_state_changes[3];
  float abs_spike_periods[3];
  long steps[3];

  // Initialize variables with appropriate values.
  for (int i = 0; i < 3; i++) {
    last_state_changes[i] = 0;
    wheel_velocities[i] = 0;
    target_velocities[i] = 0;
    steps[i] = 0;
    abs_spike_periods[i] = velocityPWMConversion(wheel_velocities[i]) / 2;
  }

  long last_velocity_update = micros();

  while (drivingMode) {
    if (Serial.available() > 0) {
      handleSerial();
    }

    /* For each motor:
     * Check if the time has passed to flip the step pin. This time is determined by the 
     * absolute spike periods. The spike period isn't defined for zero velocities, so we don't 
     * flip the pin.
     * Increase/decrease the motor steps on every pin flip. (So we actually log half steps)
     */
    if (micros() - last_state_changes[0] >= abs_spike_periods[0] && wheel_velocities[0] != 0) {
      GPIO9_DR_TOGGLE ^= 1 << 5; // Teensy Pin 3
      last_state_changes[0] = micros();
      if (wheel_velocities[0] > 0) steps[0]++; else steps[0]--;
    }

    if (micros() - last_state_changes[1] >= abs_spike_periods[1] && wheel_velocities[1] != 0) {
      GPIO7_DR_TOGGLE ^= 1 << 10; // Teensy Pin 6
      last_state_changes[1] = micros();
      if (wheel_velocities[1] > 0) steps[1]++; else steps[1]--;
    }

    if (micros() - last_state_changes[2] >= abs_spike_periods[2] && wheel_velocities[2] != 0) {
      GPIO7_DR_TOGGLE ^= 1 << 11; // Teensy Pin 9
      last_state_changes[2] = micros();
      if (wheel_velocities[2] > 0) steps[2]++; else steps[2]--;
    }

    // After VELOCITY_UPDATE_PERIOD_MICROSECONDS have passed, update the velocities and send 
    // the recorded (half)steps.
    if (micros() - last_velocity_update > VELOCITY_UPDATE_PERIOD_MICROSECONDS) {
      last_velocity_update = micros();
      updateVelocities(abs_spike_periods);
      setDirectionPins();
      sendSteps(steps);
      for (int i = 0; i < 3; i++) steps[i] = 0;
    }
  }

  // Send the steps not yet sent
  sendSteps(steps);

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], HIGH); // When leaving, disable motors

  // Send another step update with all zeros, so the node knows we have come to a halt
  for (int i = 0; i < 3; i++) steps[i] = 0;
  sendSteps(steps);
}

/**
 * Accelerate by adding the needed velocity to each wheel and recalculating
 * the spike_periods for the motor drivers.
 * Handles acceleration (wheel_velocity < target_velocity) and deceleration.
 * If the target velocity gets overshot, the wheel_velocity gets hard-set to the target velocity.
 */
void updateVelocities(float abs_spike_periods[3]) {
  float acceleration_fraction = (float) 1000000 / VELOCITY_UPDATE_PERIOD_MICROSECONDS;
  for (int i = 0; i < 3;  i++) {
    if (wheel_velocities[i] < target_velocities[i]) {
      wheel_velocities[i] += accelerations[i] / acceleration_fraction;
      if (wheel_velocities[i] > target_velocities[i]) {
        wheel_velocities[i] = target_velocities[i];
      }
      abs_spike_periods[i] = abs(velocityPWMConversion(wheel_velocities[i])) / 2;
    } else if (wheel_velocities[i] > target_velocities[i]) {
      wheel_velocities[i] -= accelerations[i] / acceleration_fraction;
      if (wheel_velocities[i] < target_velocities[i]) wheel_velocities[i] = target_velocities[i];
      abs_spike_periods[i] = abs(velocityPWMConversion(wheel_velocities[i])) / 2;
    }
  }
}

/**
 * Sets the output of the stepper driver direction pins according to the sign 
 * of the wheel velocities.
 */
void setDirectionPins() {
  digitalWriteFast(4, wheel_velocities[0] > 0);
  digitalWriteFast(7, wheel_velocities[1] > 0);
  digitalWriteFast(10, wheel_velocities[2] > 0);
}

/**
 * Send the steps of each motor over serial for odometry.
 * Since the steps are updated on every pin flip, they are actually half steps,
 * which must be considered when evaluating them.
 */
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

/**
 * Set the micro step configuration pins of the drivers according to the current micro steps.
 * The table describing the possible combinations can be found in the wiki.
 */ 
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


/**
 * Load the default values from the EEPROM and set the corresponding variables and pins.
 */
void loadDefaultValues() {
  micro_steps = getMicrosteps();
  default_velocity = getVelocity();
  default_acceleration = getAcceleration();
  steps_per_rev_with_micro_stepping = STEPS_PER_REVOLUTION * micro_steps;
  setMicrostepPins(micro_steps);
}

/**
 * Return the current battery voltage approximated by a line.
 */
float getBatteryVoltage() {
  return analogRead(VBAT) * 0.0232 + 2.78;
}

void changeMicroSteps(Command command) {
  setMicrosteps((int) command.parameters[0]);
  setMicrostepPins((int) command.parameters[0]);
  micro_steps = (int) command.parameters[0];
  steps_per_rev_with_micro_stepping = STEPS_PER_REVOLUTION * micro_steps;
}

/**
 * Wait for full commands and execute them.
 */
void loop() {
  boolean fullCommandReceived = receiveSerialData();
  if (fullCommandReceived) {
    Command command = parseCommand();
    if (command.valid) {
      runCommand(command);
    } else Serial.println("Could not parse the command");
  }
  delay(1);
}
