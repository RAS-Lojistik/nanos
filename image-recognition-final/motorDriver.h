#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER

#include "config.h"

class MotorDriver {
private:
  const uint8_t _left1Pin;
  const uint8_t _left2Pin;
  const uint8_t _leftPWMPin;
  const uint8_t _right1Pin;
  const uint8_t _right2Pin;
  const uint8_t _rightPWMPin;

public:
  MotorDriver(const uint8_t left1Pin, const uint8_t left2Pin, const uint8_t leftPWMPin, const uint8_t right1Pin, const uint8_t right2Pin, const uint8_t rightPWMPin): 
              _left1Pin(left1Pin), _left2Pin(left2Pin), _leftPWMPin(leftPWMPin), _right1Pin(right1Pin), _right2Pin(right2Pin), _rightPWMPin(rightPWMPin) {}

  inline void drive(int16_t leftSpeed, int16_t rightSpeed) {
    if(leftSpeed < 0) {
      leftSpeed = -leftSpeed;
      digitalWrite(_left1Pin, LOW);
      digitalWrite(_left2Pin, HIGH);
    } 
    else {
      digitalWrite(_left1Pin, HIGH);
      digitalWrite(_left2Pin, LOW);
    }

    if(rightSpeed < 0) {
      rightSpeed = -rightSpeed;
      digitalWrite(_right1Pin, LOW);
      digitalWrite(_right2Pin, HIGH);
    }
    else {
      digitalWrite(_right1Pin, HIGH);
      digitalWrite(_right2Pin, LOW);
    }

    if(leftSpeed > 255) {
      leftSpeed = 255;
    }
    if(rightSpeed > 255) {
      rightSpeed = 255;
    }

    analogWrite(_leftPWMPin, leftSpeed);
    analogWrite(_rightPWMPin, rightSpeed);
  }
};

#endif