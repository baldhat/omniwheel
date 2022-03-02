
#include "Arduino.h"

void rotate(float relativeAngle, float angularVelocity);
void moveDirectLine(float relativeDirection, float velocity, float distance);
float velocityToPWMPeriod(float velocity_wheel1);
void receiveSerialData();
int parseCommand();
void runCommand();

// Macro for faster digital output
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

char serialString[100];
int nextWritePosition = 0;
boolean isNewFullCommand = false;
char command;
float parameters[3];

long wheel1_counter = 0;
long wheel2_counter = 0;
long wheel3_counter = 0;

long wheel1_num_steps = 0;
long wheel2_num_steps = 0;
long wheel3_num_steps = 0;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(dirPins[i], OUTPUT);
    pinMode(stepPins[i], OUTPUT);
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], HIGH); // Disable motors until needed (LOW = ENABLED)
  }

  cli();
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0B |= B00000010; // 500ns per tick
  TIMSK0 |= B00000010; // compare match A
  OCR0A = 255;

  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= B00000010; // 500ns per tick
  TIMSK1 |= B00000010; // compare match A
  OCR1A = 255;

  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2B |= B00000010; // 500ns per tick
  TIMSK2 |= B00000010; // compare match A
  OCR2A = 255;
  sei();
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
  digitalWrite(dirPin1, spike_period1 < 0 ? LOW : HIGH );
  digitalWrite(dirPin2, spike_period2 < 0 ? LOW : HIGH );
  digitalWrite(dirPin3, spike_period3 < 0 ? LOW : HIGH );

  spike_period1 = abs(spike_period1);
  spike_period2 = abs(spike_period2);
  spike_period3 = abs(spike_period3);

  cli();
  for (int i = 0; i < 3; i++) digitalWrite(enablePins[i], LOW);

  wheel1_num_steps = duration / spike_period1;
  wheel2_num_steps = duration / spike_period2;
  wheel3_num_steps = duration / spike_period3;

  Serial.print("Wheel1_num_steps: "); Serial.println(wheel1_num_steps);
  Serial.print("Wheel2_num_steps: "); Serial.println(wheel2_num_steps);
  Serial.print("Wheel3_num_steps: "); Serial.println(wheel3_num_steps);

  wheel1_counter = 0;
  wheel2_counter = 0;
  wheel3_counter = 0;

  if (spike_period1 < 127) {
    TCCR0B |= B00000010; // 8 scaler
    OCR0A = lowByte((int) round(spike_period1) * 2);
  } else if (spike_period1 >= 128 && spike_period1 < 4096) {
    TCCR0B |= B00000100; // 256 scaler
    OCR0A = lowByte((int) round(spike_period1) / 16);
  } else {
    digitalWrite(enablePin1, HIGH);
  }
  if (spike_period2 < 127) {
    TCCR1B |= B00000010; // 8 scaler
    OCR1A = lowByte((int) round(spike_period2) * 2);
  } else if (spike_period2 >= 128 && spike_period2 < 4096) {
    TCCR1B |= B00000100; // 256 scaler
    OCR1A = lowByte((int) round(spike_period2) / 16);
  } else {
    digitalWrite(enablePin2, HIGH);
  }
  if (spike_period3 < 127) {
    TCCR2B |= B00000010; // 8 scaler
    OCR2A = lowByte((int) round(spike_period3) * 2);
  } else if (spike_period3 >= 128 && spike_period3 < 4096) {
    TCCR2B |= B00000100; // 256 scaler
    OCR2A = lowByte((int) round(spike_period3) / 16);
  } else {
    digitalWrite(enablePin3, HIGH);
  }
  sei();
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
      if (nextWritePosition >= 100) {
        Serial.println("Command too long!");
        break;
      }
      serialString[nextWritePosition++] = c;
    }
  }
}

int parseCommand() {
  command = serialString[0];
  if (command != 'D' && command != 'R') return -1;
  strtok(serialString, ";"); // throw away

  for (int i = 0; i < 3; i++) {
    char *param = strtok(NULL, ";");
    parameters[i] = atof(param);
  }

  isNewFullCommand = false;
  return 0;
}

// Interrupts:
ISR(TIMER0_COMPA_vect){
  TCNT0  = 0;
  SWT(PORTD, 3);
  wheel1_counter++;
  if (wheel1_counter >= wheel1_num_steps) {
    digitalWrite(enablePins[0], HIGH);
  }
}

ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;
  SWT(PORTD, 6);
  wheel2_counter++;
  if (wheel2_counter >= wheel2_num_steps) {
    digitalWrite(enablePins[1], HIGH);
  }
}

ISR(TIMER2_COMPA_vect){
  TCNT2  = 0;
  SWT(PORTB, 1);
  wheel3_counter++;
  if (wheel3_counter >= wheel3_num_steps) {
    digitalWrite(enablePins[2], HIGH);
  }
}
