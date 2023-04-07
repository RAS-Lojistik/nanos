#ifndef CONFIG
#define CONFIG

#include <QTRSensors.h>

#define NDEBUG

namespace cfg {
  static const float k_base = 120;
  static const float k_p = 150;
  static const float k_d = 5;

  static const uint8_t k_sensorCount = 6;
  static const QTRReadMode k_readMode = QTRReadMode::OnAndOff;
  static const uint8_t k_calibrationCount = 100;

  static const uint32_t k_serialBaudRate = 115200; 

  static const uint8_t k_SPIConfigByte = 0b00000010; //00[DataOrder]0[POLARITY1][POLARITY2][CLOCK1][CLOCK2]

  namespace pins {
    static const uint8_t emitter = 12;
    static const uint8_t qtrPins[] = {A0, A1, A2, A3, A4, A5};

    static const uint8_t SRClock = 4;
    static const uint8_t SRData = 5;
    static const uint8_t SRStrobe = 10;

    static const uint8_t rightMotorPWM = 9;
    static const uint8_t leftMotorPWM = 10;

    static const uint8_t armUnoSS = 10;
    static const uint8_t SPIMISO = 11;
    static const uint8_t SPIMOSI = 12;
    static const uint8_t SPIClock = 13;
  }
}

#define CONFIG
#endif