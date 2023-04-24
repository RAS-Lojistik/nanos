#ifndef CONFIG
#define CONFIG

#include <QTRSensors.h>

#define aDEBUG_SERIAL
#define aLOGI_TEST

namespace cfg {
  static const int16_t k_base = 35;
  static const float k_p = 0.04;
  static const float k_d = 1200;

  static const float k_onLineThreshold = 0.4f;

  static const int16_t k_turnLeftSpeed = 35;
  static const int16_t k_forwardSpeed = 30;
  static const int16_t k_forwardDuration = 580;

  static const int16_t k_parkForwardDuration = 1500;

  static const uint8_t k_sensorCount = 6;
  static const QTRReadMode k_readMode = QTRReadMode::On;
  static const uint16_t k_calibrationMoveDuration = 950;
  static const uint16_t k_calibrationMoveSpeed = 35;
  static const uint8_t k_calibrationWiggleCount = 2;

  static const uint32_t k_serialBaudRate = 9600;

  namespace pins {
    static const uint8_t emitter = A7;
    static const uint8_t qtr[] = { A1, A2, A3, A4, A5, A6 };

    static const uint8_t button = 2;
    
    static const uint8_t leftMotor1 = 5;
    static const uint8_t leftMotor2 = 4;
    static const uint8_t leftMotorPWM = 6;
    static const uint8_t rightMotor1 = 7;
    static const uint8_t rightMotor2 = 8;
    static const uint8_t rightMotorPWM = 9;

    static const uint8_t nrfInterrupt = 3;
    static const uint8_t nrfEnable = A0;
    static const uint8_t nrfSS = 10;

    static const uint8_t SPIMOSI = 11;
    static const uint8_t SPIMISO = 12;
    static const uint8_t SPIClock = 13;
  }
}

#define CONFIG
#endif