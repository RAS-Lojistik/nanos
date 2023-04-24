#ifndef EXTERNAL_F
#define EXTERNAL_F

#include "config.h"

#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>

#include <string.h>

//SEND COMMAND TO PI TO START IMAGE RECOGNITION; STALL UNTIL MOVE COMMAND IS RECEIVED
void serialCom() {
  Serial.write(100);
  
  uint8_t responded = 0;
  while(!responded) {
    if(Serial.available()) {
      String data = Serial.readStringUntil('\n');
      if(data == "move") {
        responded = 1;
        Serial.write(45);
      }
    }
  }
}

//GET THE PATH FROM PI; WRITE IT TO path & RETURN PATH LENGTH
uint16_t getPathFromPi(char** pathPtr) {
  uint16_t bufferSize = 256;
  char buffer[bufferSize];
  uint16_t count = 0;
  while (!Serial.available()) {}
  if (Serial.available() > 0) {
    uint16_t count = Serial.readBytesUntil('\n', buffer, bufferSize);
  }

  *(pathPtr) = new char[count];
  memcpy(*(pathPtr), buffer, count);  
  return count;
}

//SEND PATH DATA TO THE OTHER NANO
void sendCommandsWithNRF(char* path, uint8_t pathSize){
  RF24 radio(cfg::pins::nrfEnable, cfg::pins::nrfSS);

  const uint8_t address[6] = "ras56";
  uint8_t dataChunk[32];

  radio.begin(); 
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  radio.write(&pathSize, 1);
  delay(30);

  uint8_t cursor = 0;
  for(; cursor < pathSize; cursor += 31) {
    if(pathSize - cursor >= 31) {
      memcpy(dataChunk, &path[cursor], 31);
      radio.write(dataChunk, 31);
    }
    else {
      memcpy(dataChunk, &path[cursor], pathSize - cursor);
      radio.write(dataChunk, pathSize - cursor);
    }
    delay(30);
  }
}

#endif