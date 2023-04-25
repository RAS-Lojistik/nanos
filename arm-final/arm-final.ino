#include <Servo.h>
#include <Stepper.h>

#include <avr/io.h>

const uint8_t k_armPickUpWord = 0b01011110;
const uint8_t k_armDropWord = 0b01010010;
volatile uint8_t SPIReceive = 255;

uint8_t armState = 0; //0 for lowered, 1 for up

Servo myServo;
Stepper stepper(2038, 5, 7, 6, 8);

ISR(SPI_STC_vect) {
  SPIReceive = SPDR;
}

void setup() {
  pinMode(10, INPUT);
  pinMode(11, INPUT);
  pinMode(12, OUTPUT);
  pinMode(13, INPUT);
  SPCR = 0b11000011;
 
  pinMode(2, OUTPUT);

  myServo.attach(2);
  stepper.setSpeed(15);

  myServo.write(35);
  stepper.step(750);
  delay(1000);
}

void loop() {
  if((SPIReceive == k_armPickUpWord) && !armState){
    grab_and_up();  
    armState = 1;
  }
  else if((SPIReceive == k_armDropWord) && armState) {
    down_and_loose();
    armState = 0;
  }
}

void grab_and_up() {
  stepper.step(-450);
  delay(1000);
  myServo.write(115);
  delay(1000);
  stepper.step(750);
  delay(1000);
}

void down_and_loose() {
  stepper.step(-450);
  delay(1000);
  myServo.write(35);
  delay(1000);
  stepper.step(750);
  delay(1000);
}