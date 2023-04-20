#ifndef SHIFT_REGISTER
#define SHIFT_REGISTER

class ShiftRegister {
private:
  const uint8_t _dataPin;
  const uint8_t _clockPin;
  const uint8_t _strobePin;
  
  inline void push() const {
    cli();
    for(uint8_t j = 0; j < 8; j++) {
      if((0b10000000 >> j) & this->byte) {
        digitalWrite(_dataPin, HIGH);
      } 
      else {
        digitalWrite(_dataPin, LOW);
      }
      delayMicroseconds(2);
      digitalWrite(_clockPin, HIGH);
      delayMicroseconds(2);
      digitalWrite(_clockPin, LOW);
      delayMicroseconds(2);
      digitalWrite(_dataPin, LOW);
      delayMicroseconds(2);
    }
    digitalWrite(_strobePin, HIGH);
    delayMicroseconds(2);
    digitalWrite(_strobePin, LOW);
    delayMicroseconds(2);
    sei();
  }

public:
  uint8_t byte;
  ShiftRegister(const uint8_t dataPin, const uint8_t clockPin, const uint8_t strobePin):
    _dataPin(dataPin), 
    _clockPin(clockPin), 
    _strobePin(strobePin),
    byte(0) {
  }

  inline void forcePush(const uint8_t byte) {
    this->byte = byte;
    this->push();
  }

  inline void setBit(const uint8_t bitOrder, const uint8_t state) {
    if(((1UL << bitOrder) & this->byte) ^ (state << bitOrder)) {
      this->byte ^= (1UL << bitOrder);
      this->push();
    }
  }
  
  inline void setBits(const uint8_t bitMask, const uint8_t setByte) {
    uint8_t tempByte = this->byte;
    for (uint8_t i = 0; i < 8; i++) {
      if((1UL << i) & bitMask) {
        if(((1UL << i) & tempByte) ^ ((1UL << i) & setByte)) {
          tempByte ^= (1UL << i);
        }
      }
    }
    if(tempByte != this->byte) {
      this->byte = tempByte;
      this->push();       
    }
  }

  inline void toggleBit(const uint8_t bitOrder) {
    this->byte ^= (1UL << bitOrder);
    this->push();
  }
};

#endif