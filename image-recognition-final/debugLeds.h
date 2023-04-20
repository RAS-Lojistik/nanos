#ifndef DEBUG_LEDS
#define DEBUG_LEDS

#include "config.h"

class DebugLeds {
private:
  const uint8_t _redPin;
  const uint8_t _greenPin;
  const uint8_t _bluePin;

  uint8_t _redState = 0;
  uint8_t _greenState = 0;
  uint8_t _blueState = 0;

public:
  DebugLeds(const uint8_t redPin, const uint8_t greenPin, const uint8_t bluePin):
            _redPin(redPin), _greenPin(greenPin), _bluePin(bluePin) {
    digitalWrite(_redPin, LOW);
    digitalWrite(_greenPin, LOW); 
    digitalWrite(_bluePin, LOW); 
  }

  inline void redOn() {
    if(!_redState) {
      _redState = 1;
      digitalWrite(_redPin, HIGH);
    }
  }
  inline void redOff() {
    if(_redState) {
      _redState = 0;
      digitalWrite(_redPin, LOW);
    }
  }
  inline void redToggle() {
    _redState ^= 1UL;
    if(_redState) {
      digitalWrite(_redPin, LOW);
    }
    else {
      digitalWrite(_redPin, HIGH);
    }
  }

  inline void greenOn() {
    if(!_greenState) {
      _greenState = 1;
      digitalWrite(_greenPin, HIGH);
    }
  }
  inline void greenOff() {
    if(_greenState) {
      _greenState = 0;
      digitalWrite(_greenPin, LOW);
    }
  }
  inline void greenToggle() {
    _greenState ^= 1UL;
    if(_greenState) {
      digitalWrite(_greenPin, LOW);
    }
    else {
      digitalWrite(_greenPin, HIGH);
    }
  }

  inline void blueOn() {
    if(!_blueState) {
      _blueState = 1;
      digitalWrite(_bluePin, HIGH);
    }
  }
  inline void blueOff() {
    if(_blueState) {
      _blueState = 0;
      digitalWrite(_bluePin, LOW);
    }
  }
  inline void blueToggle() {
    _blueState ^= 1UL;
    if(_blueState) {
      digitalWrite(_bluePin, LOW);
    }
    else {
      digitalWrite(_bluePin, HIGH);
    }
  }
};

#endif