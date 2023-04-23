#ifndef EXTERNAL_F
#define EXTERNAL_F

#include "config.h"

#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <string.h>

uint16_t getCommandsWithNRF(char** commandsPtr) {
  #ifndef NRF_ACTIVE
    char buffer[] = "frffrfrflfrfrflffffrffffflffffrffrflfflffffrrfflfrflfrflflfrflfrfrrffrflfrfrflflfrfflffrfls";
    const uint8_t bufferSize = 92;

    char* commandsArray = new char[bufferSize];
    memcpy(commandsArray, buffer, bufferSize);
    *(commandsPtr) = commandsArray;
    return bufferSize;
  #else
    RF24 radio(cfg::pins::nrfEnable, cfg::pins::nrfSS);

    const byte address[6] = "ras56";
    char dataChunk[32];

    char commands[7]="fslrpd";
    char commandCodes[216][3];

    short int i = 0;
    short int j = 0;
    short int l = 0;
    short int k = 0;
    short int kj = 0;

    short int sizeOfSignal = 0;
    short int pathLength = 0;

    for(i=0; i<=5; i++) {
      for(j=0; j<=5; j++) {
        for(k=0; k<=5; k++) {
          commandCodes[l][0]=commands[i];
          commandCodes[l][1]=commands[j];
          commandCodes[l][2]=commands[k];
          l++;
        }
      }
    }

    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();

    while(1) {
      if (radio.available()) {
        radio.read(&sizeOfSignal, sizeof(sizeOfSignal));
        break;
      }
      delay(15);
    }

    unsigned char signalReceived[sizeOfSignal];

    while(kj < sizeOfSignal - 2) {
      while(1) {
        if(radio.available()) {
          radio.read(&dataChunk, sizeof(dataChunk));
          delay(5);
          break;
        }
      }

      for(i = 0; i < 31; i++) {
        signalReceived[kj] = dataChunk[i];
        kj++;
      }
    }

    for(i = 0; signalReceived[i] < 216 && i < sizeOfSignal - 1; i++) {
      pathLength++;
    }

    pathLength = pathLength * 3;
    pathLength = pathLength + sizeOfSignal - i;

    char path[pathLength];

    kj = 0;
    for(i = 0; signalReceived[i] < 216 && i<sizeOfSignal-1; i++){
      path[kj] = commandCodes[signalReceived[i]][0]; 
      kj++;
      path[kj] = commandCodes[signalReceived[i]][1];
      kj++;
      path[kj] = commandCodes[signalReceived[i]][2];
      kj++;
    }
      
    for(; kj < pathLength - 1; kj++){
      switch(signalReceived[i]) {
        case 216:
          path[kj] = 'f';
          i++;
          break;
        case 217:
          path[kj] = 's';
          i++;
          break;
        case 218:
          path[kj] = 'l';
          i++;
          break;
        case 219:
          path[kj] = 'r';
          i++;
          break;
        case 220:
          path[kj] = 'p';
          i++;
          break;
        case 221:
          path[kj] = 'd';
          i++;
          break;
        case 255:
          path[kj] = 'e';
          i++;
          break;
      }
    }

    char* commandsArray = new char[sizeof(path)];
    memcpy(commandsArray, path, sizeof(path));  
    *(commandsPtr) = commandsArray;
    return sizeof(path);
  #endif
}

#endif