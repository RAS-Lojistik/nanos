#ifndef SPI
#define SPI

#include "config.h"

class SPIA {
private:
  uint8_t _cfgByte;
  uint8_t _enabled;

public:
  SPIA(const uint8_t cfgByte):
    _cfgByte(cfgByte) {
  }
  ~SPIA() {
    this->disable();
  }

  void setCfgByte(uint8_t cfgByte) {
    _cfgByte = cfgByte;
    if(_enabled) {
      this->enable();
    }
  }

  inline void enable() {
    SPCR = _cfgByte;
    _enabled = 1;
  }
  inline void disable() {
    SPCR = 0UL;
    _enabled = 0;

  }

  inline void writeToSlave(const uint8_t byte, const uint8_t ssPin) {
    cli();
    digitalWrite(ssPin, HIGH);
    delayMicroseconds(1);
    SPDR = byte;
    delayMicroseconds(70);
    digitalWrite(ssPin, LOW);
    delayMicroseconds(1);
    sei();
  }
};

#endif