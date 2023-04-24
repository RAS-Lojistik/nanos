#ifndef EXTERNAL_F
#define EXTERNAL_F

#include "config.h"

#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <string.h>

uint16_t getCommandsWithNRF(char** pathPtr) {
  #ifndef NRF_ACTIVE
    char buffer[] = "frffrfrflfrfrflffffrffffflffffrffrflfflffffrrfflfrflfrflflfrflfrfrrffrflfrfrflflfrfflffrfls";
    const uint8_t bufferSize = 92;

    char* commandsArray = new char[bufferSize];
    memcpy(commandsArray, buffer, bufferSize);
    *(pathPtr) = commandsArray;
    return bufferSize;
  #else
    RF24 radio(A2, 4);
    const uint8_t address[6] = "ras56";
    
    char dataChunk[32]; 
    uint8_t pathSize = 0;

    radio.begin();
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();

    #ifdef DEBUG_SERIAL
      Serial.begin(cfg::k_serialBaudRate);  
    #endif
      while(!radio.available()){
    #ifdef DEBUG_SERIAL
        Serial.println("waiting for signal...");
    #endif
        delay(5);
      }

    radio.read(&pathSize, 1);

    char* path = new char[pathSize];
    *(pathPtr) = path;

    uint8_t cursor = 0;
    for(; cursor < pathSize; cursor += 31) {
      while(!radio.available()) { }
      if(pathSize - cursor >= 31) {
        radio.read(dataChunk, 31);
        memcpy(&path[cursor], dataChunk, 31);
      }
      else {
        radio.read(dataChunk, pathSize - cursor);
        memcpy(&path[cursor], dataChunk, pathSize - cursor);      
      }
      delay(5);
    }

    #ifdef DEBUG_SERIAL
      for(uint8_t i = 0; i < pathSize; i++) {
        Serial.print(path[i]);
      }
    #endif

    return pathSize;
  #endif
}

#endif