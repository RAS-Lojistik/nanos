#include <avr/io.h>

const uint8_t k_armPickUpWord = 31;
const uint8_t k_armDropWord = 37;
volatile uint8_t SPIReceive = 255;

uint8_t armState = 0; //0 for lowered, 1 for up

ISR(SPI_STC_vect) {
  SPIReceive = SPDR;
}

void setup() {
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, OUTPUT);
  pinMode(13, INPUT);
  SPCR = 0b11000011;
}

void loop() {
  if((SPIReceive == k_armPickUpWord) && !armState){
    cli();
    armState = 1;
      //TO DO: move arm up code        
    sei();
  }
  else if((SPIReceive == k_armDropWord) && armState) {
    cli();
    armState = 0;
      //TO DO: drop code        
    sei();
  }
}
