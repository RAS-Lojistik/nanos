#define MASTER

#include <SPI.h> 
#include <avr/io.h>

#ifdef TEST
  uint8_t dataArray[] = {0b10000000, 0b01000000, 0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010, 0b00000001};
  volatile uint8_t SPIReceive = 0b10011111;
  uint8_t i = 0;

  void setup() {
    Serial.begin(9600);

    DDRB |= (1 << DDB4) | (1 << DDB3) | (1 << DDB2);  // SET SCK, MOSI & SS AS OUTPUT 
    PORTB |= (1 << DDB2);                             // SET SS HIGH
    SPCR = 0b11010011;                                // CONFIGURE SPI: ISR enabled, master, sample config. & 1/128 clock speed. 
  }

  void loop() {
    PORTB &= ~(1 << DDB2);
    delayMicroseconds(1);
    SPDR = dataArray[i++];
    delayMicroseconds(70);
    PORTB |= (1 << DDB2);

    i = i % 8;
    Serial.println(SPIReceive);
    delay(500);
  }

  ISR(SPI_STC_vect) {
    SPIReceive = SPDR;
  }
#endif

#ifdef MASTER
  uint8_t dataArray[] = {0b10000000, 0b01000000, 0b00100000, 0b00111000, 0b00001000, 0b00000100, 0b00000010, 0b00000001};
  volatile uint8_t SPIReceive = 0b00000000;
  uint8_t i = 0;

  ISR(SPI_STC_vect) {
    SPIReceive = SPDR;
  }

  void setup() {
    Serial.begin(9600);

    DDRB |= (1 << DDB5) | (1 << DDB3) | (1 << DDB2);  // SET SCK, MOSI & SS AS OUTPUT 
    PORTB |= (1 << DDB2);                             // SET SS HIGH
    SPCR = 0b11010011;                                // CONFIGURE SPI: ISR enabled, master, sample config. & 1/128 clock speed. 
  }

  void loop() {
    PORTB &= ~(1 << DDB2);
    delayMicroseconds(1);
    SPDR = dataArray[i++];
    delayMicroseconds(70);
    PORTB |= (1 << DDB2);

    i = i % 8;
    Serial.println(SPIReceive);
    delay(500);
  }
#endif

#ifdef SLAVE
uint8_t dataArray[] = {0b00011111, 0b00100101, 0b00111110};
volatile uint8_t SPIReceive = 0b00000000;
uint8_t i = 0;

ISR(SPI_STC_vect) {
  SPIReceive = SPDR;
  SPDR = dataArray[i++];
  i = i % 3;
}

void setup() {
  //SETUP LED PINS
  DDRD = 0b11111100;
  DDRB = 0b00000011;

  DDRB |= (1 << DDB4);  // SET MISO AS OUTPUT 
  SPCR = 0b11000011;    // CONFIGURE SPI: ISR enabled, master, sample config. & 1/128 clock speed. 
}

void loop() {
  PORTD = 0b00000000;
  PORTD |= (SPIReceive << 2);
  PORTB = 0b00000000;
  PORTB |= (SPIReceive >> 6);
  delay(1);
}
#endif