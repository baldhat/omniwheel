#include "Arduino.h"

/**
  Contains a number of helper methods for easier printing to the serial connection.
*/

void println(String message, float value) {
  Serial.print(message); Serial.println(value, 5);
}

void println(String message, long value) {
    Serial.print(message); Serial.println(value);
}

void println(String message, int value) {
  Serial.print(message); Serial.println(value);
}

void println(int value) {
  Serial.println(value);
}

void println(float value) {
  Serial.println(value);
}

void println(long value) {
  Serial.println(value);
}
