
#include "Arduino.h"
#include "parser.h"

#define NUM_VALID_COMMANDS 9

char serialString[50]; // Char buffer for the message being read from serial
int nextWritePosition = 0; // Pointer to the next position in the serialString to write to
boolean isNewFullCommand = false; // Flag showing if a command has been read fully
char validCommands[NUM_VALID_COMMANDS + 1] = "AaSsMmIEb"; // Char array of all allowed command chars

/*
  If available, read data from the serial connection. Reset the nextWritePosition
  to the start of the buffer if the beginning of a new command is detected, and
  set the full command flag to true if a command has been received.
*/
boolean receiveSerialData() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '{') {
      nextWritePosition = 0;
      isNewFullCommand = false;
      memset(serialString, '\0', sizeof(serialString)); // Clear the serialString array
    } else if (c == '}')
      isNewFullCommand = true;
    else {
      if (nextWritePosition >= 50) {
        Serial.println("Command too long!");
        break;
      }
      serialString[nextWritePosition++] = c;
    }
  }
  return isNewFullCommand;
}

/*
  Determine whether a command is valid. It is valid if the command char is a valid command char.
*/
boolean isValidCommand(char command) {
  for (int i = 0; i < NUM_VALID_COMMANDS; i++) {
    if (command == validCommands[i]) return true;
  }
  return false;
}

/*
  Create a command object from the read serial command string and return it.
  Always returns a command, but provides flag if the command is invalid.
*/
Command parseCommand() {
  Command command;
  command.type = serialString[0];
  if (!isValidCommand(command.type)) {
    command.valid = false;
    return command;
  }
  strtok(serialString, ";"); // throw away

  for (int i = 0; i < 3; i++) {
    char *param = strtok(NULL, ";");
    if (param != NULL)
      command.parameters[i] = atof(param);
  }

  isNewFullCommand = false;
  command.valid = true;
  return command;
}
