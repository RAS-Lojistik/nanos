#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

void setup() { }

void loop() {
  const char pathStr[] = "frfflflffrfrfflflfflffflflfffrfrfffrffls";     //JUNCTION TEST: "frfflflffrfrfflflfflffflflfffrfrfffrffls"
  char path[sizeof(pathStr) - 1];
  memcpy(path, pathStr, sizeof(pathStr) - 1);
  uint8_t pathSize = sizeof(path);
  sendCommandsWithNRF(path, pathSize);
  delay(5000);
}

void sendCommandsWithNRF(char* path, uint8_t pathSize){
  RF24 radio(A2, 4);

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