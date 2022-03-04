#include "Arduino.h"

void println(String message, float value) {
  Serial.print(message); Serial.println(value, 5);
}

void println(String message, long value) {
    Serial.print(message); Serial.println(value);
}
