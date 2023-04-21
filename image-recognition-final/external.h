#ifndef EXTERNAL
#define EXTERNAL

//SEND COMMAND TO PI TO START IMAGE RECOGNITION; STALL UNTIL MOVE COMMAND IS RECEIVED
void OCRPi() {
  return 1;   
}

//GET THE PATH FROM PI; WRITE IT TO path & RETURN PATH LENGTH
uint8_t getPathFromPi(char** pathPtr) {
  *(pathPtr) = new char[13];
  char buffer[] = "ffrffrfrffls";
  memcpy(*(pathPtr), buffer, 13);  
  return 13;
}

//SEND PATH DATA TO THE OTHER NANO
void sendCommandsWithNRF(char* path, size_t pathSize) {
  return;
}

#endif