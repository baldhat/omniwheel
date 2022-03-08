#include "Arduino.h"

#ifndef parser_h
#define parser_h

class Command {
  public:
    char type;
    float parameters[3];
    boolean valid;
};

boolean receiveSerialData();
Command parseCommand();
boolean isValidCommand(char command);

#endif
