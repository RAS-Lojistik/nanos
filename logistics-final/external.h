#ifndef EXTERNAL_F
#define EXTERNAL_F

#include "config.h"

#include <string.h>

uint16_t getCommandsWithNRF(char** commandsPtr) {
    *(commandsPtr) = new char[6];
    char buffer[] = "ffrffrfrffls";
    memcpy(*(commandsPtr), buffer, 13);  
    return 13;
}

#endif