
#include "Arduino.h"
#include "parser.h"

#define NUM_VALID_COMMANDS 8

char serialString[100];
int nextWritePosition = 0;
boolean isNewFullCommand = false;
char validCommands[NUM_VALID_COMMANDS + 1] = "RDAaSsMm";

boolean receiveSerialData() {
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
  return isNewFullCommand;
}

boolean isValidCommand(char command) {
  for (int i = 0; i < NUM_VALID_COMMANDS; i++) {
    if (command == validCommands[i]) return true;
  }
  return false;
}

Command parseCommand() {
  Command command;
  command.type = serialString[0];
  if (!isValidCommand(command.type)) command.valid = false;
  strtok(serialString, ";"); // throw away

  for (int i = 0; i < 3; i++) {
    char *param = strtok(NULL, ";");
    command.parameters[i] = atof(param);
  }

  isNewFullCommand = false;
  command.valid = true;
  return command;
}
