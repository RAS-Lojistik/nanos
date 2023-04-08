#ifndef EXTERN
#define EXTERN

#include "config.h"

uint16_t GetCommandsWithNRF(char** commands) {
    *(commands) = new char[500];
    return 500;
}

#endif