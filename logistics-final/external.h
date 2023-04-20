#ifndef EXTERNAL_F
#define EXTERNAL_F

#include "config.h"

#include <string.h>

uint16_t GetCommandsWithNRF(char** commands) {
    *(commands) = new char[6];
    char buffer[] = "ffrffrfrffls";
    memcpy(*(commands), buffer, 13);  
    return 13;
}

#endif