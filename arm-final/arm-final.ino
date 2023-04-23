#include <Servo.h>
#include <Stepper.h>

#include <avr/io.h>

const uint8_t k_armPickUpWord = 0b01011110;
const uint8_t k_armDropWord = 0b01010010;
volatile uint8_t SPIReceive = 255;

uint8_t armState = 0; //0 for lowered, 1 for up

Servo myservo;
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

  myservo.attach(2);
  stepper.setSpeed(15);

  stepper.step(400);
  delay(1500);
  myservo.write(40);
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
  stepper.step(-400);
  delay(800);
  myservo.write(120);
  delay(800);
  stepper.step(400);
  delay(800);
}

void down_and_loose() {
  stepper.step(-400);
  delay(800);
  myservo.write(40);
  delay(800);
  stepper.step(400);
  delay(800);
}