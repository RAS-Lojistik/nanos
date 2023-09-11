#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

void setup() {
  char* path;
  getCommandsWithNRF(&path);
}

void loop() { }

uint16_t getCommandsWithNRF(char** pathPtr) { 
  RF24 radio(A2, 4);
  const uint8_t address[6] = "ras56";
  
  char dataChunk[32]; 
  uint8_t pathSize = 0;

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  Serial.begin(57600);  
  while(!radio.available()){
    Serial.println("waiting for signal...");
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

  for(uint8_t i = 0; i < pathSize; i++) {
    Serial.print(path[i]);
  }

  return pathSize;
}