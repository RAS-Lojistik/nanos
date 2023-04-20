#ifndef EXTERNAL
#define EXTERNAL

//SEND COMMAND TO PI TO START IMAGE RECOGNITION; STALL UNTIL MOVE COMMAND IS RECEIVED
void recognize() {
  return 1;   
}

//GET THE PATH FROM PI; WRITE IT TO pathBuffer & RETURN PATH LENGTH
uint8_t getPath(char** pathBufferPtr) {
  pathBufferPtr = new char[26];
  *(*(pathBufferPtr)) = "frlflfrlflfrlflfrlflfrlfl";
  return 26;
}

//SEND PATH DATA TO THE OTHER NANO
void txPath(char* pathBuffer, size_t pathSize) {
  return;
}

#endif