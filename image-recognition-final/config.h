#ifndef CONFIG
#define CONFIG

#include <QTRSensors.h>

#define aDEBUG_SERIAL
#define aLOGI_TEST

namespace cfg {
  static const float k_base = 120;
  static const float k_p = 150;
  static const float k_d = 5;

  static const float k_onLineThreshold = 0.2;

  static const float k_turnLeftSpeed = 70;
  static const float k_forwardSpeed = 70;
  static const float k_forwardDuration = 150;

  static const uint8_t k_sensorCount = 6;
  static const QTRReadMode k_readMode = QTRReadMode::OnAndOff;
  /* TO DO
  static const uint16_t k_calibrationMoveDuration = 150;
  static const uint16_t k_calibrationMoveSpeed = 60;
  static const uint8_t k_calibrationWiggleCount = 3;
  */

  static const uint32_t k_serialBaudRate = 115200;

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

    static const uint8_t nrfEnable = A0;
    static const uint8_t nrfInterrupt = 3;
    static const uint8_t nrfSS = 10;
    static const uint8_t SPIMISO = 12;
    static const uint8_t SPIMOSI = 11;
    static const uint8_t SPIClock = 13;
  }
}

#define CONFIG
#endif