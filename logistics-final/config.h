#ifndef CONFIG
#define CONFIG

#include <QTRSensors.h>

#define aDEBUG_SERIAL
#define DEBUG_LED
#define aDEBUG_MOTOR
#define aLOGI_TEST

namespace cfg {
  static const int16_t k_base = 55;
  static const float k_p = 0.05;
  static const float k_d = 1200;

  static const float k_onLineThreshold = 0.4f;

  static const int16_t k_turnRightSpeed = 35; //DURATION 750 FOR 90 DEGREES
  static const int16_t k_turnLeftSpeed = 35;
  static const int16_t k_forwardSpeed = 40;
  static const int16_t k_forwardDuration = 580;

  static const uint8_t k_sensorCount = 6;
  static const QTRReadMode k_readMode = QTRReadMode::On;

  static const uint32_t k_lineColorTimeout = 250000;

  static const uint16_t k_calibrationMoveDuration = 1300;
  static const uint16_t k_calibrationMoveSpeed = 35;
  static const uint8_t k_calibrationWiggleCount = 2;

  static const uint32_t k_serialBaudRate = 115200; 

  static const uint8_t k_SPIConfigByte = 0b00000010; //00[DataOrder]0[POLARITY1][POLARITY2][CLOCK1][CLOCK2]
  static const uint8_t armPickUpWord = 31;
  static const uint8_t armDropWord = 37; 

  namespace srOrders {
    static const uint8_t left1 = 0;
    static const uint8_t left2 = 1;
    static const uint8_t right1 = 2;
    static const uint8_t right2 = 3;
    static const uint8_t LEDBlue = 4;
    static const uint8_t LEDGreen = 5;
    static const uint8_t LEDRed = 6;
  }

  namespace pins {
    static const uint8_t emitter = A7;
    static const uint8_t qtr[] = { A0, A1, A3, A4, A5, A6 };

    static const uint8_t SRClock = 7;
    static const uint8_t SRData = 8;
    static const uint8_t SRStrobe = 9;

    static const uint8_t rightMotorPWM = 5;
    static const uint8_t leftMotorPWM = 6;

    static const uint8_t nrfInterrupt = 3;
    static const uint8_t nrfSS = 4;

    static const uint8_t armUnoSS = 10;
    static const uint8_t SPIMISO = 11;
    static const uint8_t SPIMOSI = 12;
    static const uint8_t SPIClock = 13;
  }
}

#define CONFIG
#endif