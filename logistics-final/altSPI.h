#ifndef SPI
#define SPI

#include "config.h"

class AltSPI {
private:
  uint8_t _cfgByte;
  uint8_t _enabled;

public:
  AltSPI(const uint8_t cfgByte):
    _cfgByte(cfgByte) {
  }
  ~AltSPI() {
    this->disable();
  }

  inline void setCfgByte(const uint8_t cfgByte) {
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

  inline void writeToSlave(const uint8_t byte, const uint8_t ssPin) const {
    cli();
    digitalWrite(ssPin, LOW);
    delayMicroseconds(1);
    SPDR = byte;
    delayMicroseconds(70);
    digitalWrite(ssPin, HIGH);
    delayMicroseconds(1);
    sei();
  }
};

#endif