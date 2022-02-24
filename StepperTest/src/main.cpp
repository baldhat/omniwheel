
#include "Arduino.h"

void rotate(unsigned int spike_periods[]);

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

const int STEPS_PER_REVOLUTION = 6400; // 200 * 32 (micro-stepping)

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing drivers...");
  for (int i = 0; i < 3; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(stepPins[i], OUTPUT);
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], HIGH);
  }
}

void loop() {
  Serial.println("Enter delay in microseconds");
  while (Serial.available() <= 0) {
    delay(10);
  }
  unsigned int dbs = (unsigned int) Serial.parseInt();
  while (Serial.available() > 0) Serial.read(); // flush

  unsigned int spike_periods[] = {dbs, dbs, dbs};

  if (dbs >= 1)
    rotate(spike_periods);
}

void rotate(unsigned int spike_periods[]) {
  unsigned int num_steps[] = {0, 0, 0};
  unsigned long last_state_changes[] = {0, 0, 0};
  boolean states[] = {LOW, LOW, LOW};

  Serial.println(spike_periods[0]);

  for (int i = 0; i < 3; i++) {
    digitalWrite(enablePins[i], LOW);
    digitalWrite(dirPins[i], HIGH);
  }
  delayMicroseconds(2);

  while (num_steps[0] <= STEPS_PER_REVOLUTION) {
    for (int i = 0; i < 3; i++) {
      if (states[i] != HIGH && micros() - last_state_changes[i] >= spike_periods[i]) {
        digitalWrite(stepPins[i], HIGH);
        last_state_changes[i] = micros();
        states[i] = HIGH;
        num_steps[i]++;
      } else if (states[i] == HIGH && micros() - last_state_changes[i] >= spike_periods[i]) {
        digitalWrite(stepPins[i], LOW);
        last_state_changes[i] = micros();
        states[i] = LOW;
      }
    }
    delayMicroseconds(1);
  }

  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], HIGH);
}
