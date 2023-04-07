#ifndef CONFIG

#include <QTRSensors.h>

#define NDEBUG
namespace cfg{
  static const float k_base = 120;
  static const float k_iCap = 100;
  static const float k_p = 150;
  static const float k_i = 0;
  static const float k_d = 5;

  static const uint8_t k_sensorCount = 6;
  static const QTRReadMode k_readMode = QTRReadMode::OnAndOff;

  namespace pins{
    static const uint8_t emitter = 12;
    static const uint8_t qtr[] = {A0, A1, A2, A3, A4, A5};

    static const uint8_t leftMotor1 = 4;
    static const uint8_t leftMotor2 = 5;
    static const uint8_t leftMotorPWM = 10;
    static const uint8_t rightMotor1 = 3;
    static const uint8_t rightMotor2 = 2;
    static const uint8_t rightMotorPWM = 9; //right-A left-Bz
  }
}


#define CONFIG
#endif