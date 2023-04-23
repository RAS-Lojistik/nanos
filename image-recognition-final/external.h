#ifndef EXTERNAL_F
#define EXTERNAL_F

#include "config.h"

#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>

#include <string.h>

//SEND COMMAND TO PI TO START IMAGE RECOGNITION; STALL UNTIL MOVE COMMAND IS RECEIVED
void OCRPi() {
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
  uint16_t bufferSize = 250;
  char buffer[bufferSize];
  uint16_t count = 0;
  while (!Serial.available()) {}
  if (Serial.available() > 0) {
    uint16_t count = Serial.readBytesUntil('\n', buffer, bufferSize);
    buffer[count] = '\0';
    count++;
  }

  *(pathPtr) = new char[count];
  memcpy(*(pathPtr), buffer, count);  
  return count;
}

//SEND PATH DATA TO THE OTHER NANO
void sendCommandsWithNRF(char* path, int16_t pathSize) {
  RF24 radio(cfg::pins::nrfEnable, cfg::pins::nrfSS);

  char commands[7]="fslrpd";
  short int i=0;
  short int j=0;
  short int k=0;
  short int l=0;
  char commandCodes[216][3];

  const byte address[6] = "ras56"; //this will help us communicate with our module and our module only
  unsigned char dataChunk[32];

  short int pointOfSingleCommands=0;
  short int jk=0;

  short int sizeOfPathMinusTwo=pathSize-2;

  unsigned char signalToSend[((pathSize-1)-((pathSize-1)%3))/3+((pathSize-1)%3)+1];
  
  l=0;
  for(i=0; i<=5; i++){
    for(j=0; j<=5; j++){
      for(k=0; k<=5; k++){
        commandCodes[l][0]=commands[i];
        commandCodes[l][1]=commands[j];
        commandCodes[l][2]=commands[k];
        l++;
      }
    }
  }
  pointOfSingleCommands=l;
  
  l=0;
  for(i=0; (i+2)<=sizeOfPathMinusTwo ;i=i+3){
    for(j=0; j<=215; j++){
      if(commandCodes[j][0]==path[i] && commandCodes[j][1]==path[i+1] && commandCodes[j][2]==path[i+2]){
        signalToSend[l]=j;
        l++;
      }
    }
  }

  for(jk=0; i<=sizeOfPathMinusTwo ;i++){
    switch (path[i]){
      case 'f':
        signalToSend[l]=pointOfSingleCommands;
        l++; 
        break;

      case 's':
        signalToSend[l]=pointOfSingleCommands+1;
        l++;
        break;

      case 'l':
        signalToSend[l]=pointOfSingleCommands+2;
        l++;
        break;

      case 'r':
        signalToSend[l]=pointOfSingleCommands+3;
        l++;
        break;

      case 'p':
        signalToSend[l]=pointOfSingleCommands+4;
        l++;
        break;

      case 'd':
        signalToSend[l]=pointOfSingleCommands+5;
        l++;
        break;
    }
  }
      
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN); //set the power level depending on how far away the modules are
  radio.stopListening();

  short int signalMagnitude = (sizeof(signalToSend) / (sizeof(signalToSend[0]))) - 2;
  short int sigMagPTwo=signalMagnitude+2;
  radio.write(&sigMagPTwo, sizeof(sigMagPTwo));
  delay(10);

  l=0;
  for(jk=0; jk<=signalMagnitude ;jk=jk+32) {
    for(j=0; j<32 ;j++) {
      if(l<=signalMagnitude) {
        dataChunk[j]=signalToSend[l];
        l++;
      }
      else{
        dataChunk[j]= 255;
        l++;
      }
    }
    radio.write(&dataChunk, sizeof(dataChunk));
    delay(30);
  }
}

#endif