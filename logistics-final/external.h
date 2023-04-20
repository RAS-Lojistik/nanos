#ifndef EXTERNAL_F
#define EXTERNAL_F

#include "config.h"

#include <string.h>

uint16_t GetCommandsWithNRF(char** commands) {
    *(commands) = new char[6];
    char buffer[] = "fffffs";
    memcpy(*(commands), buffer, 6);  
    return 6;
}

#endif