#ifndef DEBUG_LEDS
#define DEBUG_LEDS

#include "shiftRegister.h"
#include "config.h"

class DebugLeds {
private:
  ShiftRegister* const _sr;
  const uint8_t _redSROrder;
  const uint8_t _greenSROrder;
  const uint8_t _blueSROrder;
  
public:
  DebugLeds(ShiftRegister* const sr, const uint8_t redSROrder, const uint8_t greenSROrder, const uint8_t blueSROrder):
    _sr(sr),
    _redSROrder(redSROrder),
    _greenSROrder(greenSROrder),
    _blueSROrder(blueSROrder) {
  }

  inline void redOn() {
    _sr->setBit(_redSROrder, 1UL);
  }
  inline void redOff() {
    _sr->setBit(_redSROrder, 0);
  }
  inline void redToggle() {
    _sr->toggleBit(_redSROrder);
  }

  inline void greenOn() {
    _sr->setBit(_greenSROrder, 1UL);
  }
  inline void greenOff() {
    _sr->setBit(_greenSROrder, 0);
  }
  inline void greenToggle() {
    _sr->toggleBit(_greenSROrder);
  }

  inline void blueOn() {
    _sr->setBit(_blueSROrder, 1UL);
  }
  inline void blueOff() {
    _sr->setBit(_blueSROrder, 0);
  }
  inline void blueToggle() {
    _sr->toggleBit(_blueSROrder);
  }
};

#endif