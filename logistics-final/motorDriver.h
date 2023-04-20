#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER

#include "shiftRegister.h"
#include "config.h"

class MotorDriver {
private:
  ShiftRegister* const _sr;
  const uint8_t _leftPWMPin;
  const uint8_t _leftIn1BitOrder;
  const uint8_t _leftIn2BitOrder;
  const uint8_t _rightPWMPin;
  const uint8_t _rightIn1BitOrder;
  const uint8_t _rightIn2BitOrder;
  uint8_t _srBitMask;
  uint8_t _srByte = 0UL;

  uint8_t _lastLeftDirection = 2; 
  uint8_t _lastRightDirection = 2;

public:
  MotorDriver(ShiftRegister* const sr, const uint8_t leftPWMPin, const uint8_t leftIn1BitOrder, const uint8_t leftIn2BitOrder,
                            const uint8_t rightPWMPin, const uint8_t rightIn1BitOrder, const uint8_t rightIn2BitOrder): 
                   _sr(sr), _leftPWMPin(leftPWMPin), _leftIn1BitOrder(leftIn1BitOrder), _leftIn2BitOrder(leftIn2BitOrder),
                            _rightPWMPin(rightPWMPin), _rightIn1BitOrder(rightIn1BitOrder), _rightIn2BitOrder(rightIn2BitOrder) {
    _srBitMask = (1UL << _leftIn1BitOrder) | (1UL << _leftIn2BitOrder) | (1UL << _rightIn1BitOrder) | (1UL << _rightIn2BitOrder);
  }

  inline void drive(int16_t leftSpeed, int16_t rightSpeed) {
    static uint8_t _srChangeFlag = 0;
    if(leftSpeed >= 0) {
      if(_lastLeftDirection != 1) {
        _lastLeftDirection = 1;
        _srChangeFlag = 1;
        _srByte |= (1UL << _leftIn1BitOrder);
        _srByte &= ~(1UL << _leftIn2BitOrder);
      }
    }
    else {
      leftSpeed = -leftSpeed;
      if(_lastLeftDirection != 0) {
        _lastLeftDirection = 0;
        _srChangeFlag = 1;
        _srByte &= ~(1UL << _leftIn1BitOrder);
        _srByte |= (1UL << _leftIn2BitOrder);
      }
    }

    if(rightSpeed >= 0) {
      if(_lastRightDirection != 1) {
        _lastRightDirection = 1;
        _srChangeFlag = 1;
        _srByte |= (1UL << _rightIn1BitOrder);
        _srByte &= ~(1UL << _rightIn2BitOrder);
      }
    }
    else {
      rightSpeed = -rightSpeed;
      if(_lastRightDirection != 0) {
        _lastRightDirection = 0;
        _srChangeFlag = 1;
        _srByte &= ~(1UL << _rightIn1BitOrder);
        _srByte |= (1UL << _rightIn2BitOrder);
      }
    }    

    if(leftSpeed > 250) {
      leftSpeed = 250;
    }
    if(rightSpeed > 250) {
      rightSpeed = 250;
    }

    if(_srChangeFlag) {
      _sr->setBits(_srBitMask, _srByte);
      _srChangeFlag = 0;
    }  
    analogWrite(_leftPWMPin, leftSpeed);
    analogWrite(_rightPWMPin, rightSpeed);
  }
};

#endif