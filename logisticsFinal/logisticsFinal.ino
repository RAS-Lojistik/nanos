#include <QTRSensors.h>

#include "motorDriver.h"
#include "config.h"
#include "shiftRegister.h"
#include "SPI.h"

QTRSensors qtr = QTRSensors();
uint16_t sensorValues[cfg::k_sensorCount] = { 0 };

SPIA spi = SPIA(cfg::k_SPIConfigByte);

SR sr = SR(cfg::pins::SRData, cfg::pins::SRClock, cfg::pins::SRStrobe);

MotorDriver driver = MotorDriver(&sr, cfg::pins::leftMotorPWM, 0, 1, 
                                      cfg::pins::rightMotorPWM, 2, 3);


namespace {
  uint32_t lastTime = 1;
  uint32_t currentTime = 2;
  float dT = 1;

  uint8_t lineColor = 0; //0 FOR BLACK, 1 FOR WHITE
  uint16_t position = 0;
  float lastError = 0;
  float currentError = 0;

  float speedModifier = 0.f;
}

void setup() {
  //SETUP PINS
    pinMode(cfg::pins::emitter, OUTPUT); 
    for(uint8_t i = 0; i < cfg::k_sensorCount; i++) {
      pinMode(cfg::pins::qtrPins[i], OUTPUT);
    } 

    pinMode(cfg::pins::SRClock, OUTPUT); 
    pinMode(cfg::pins::SRData, OUTPUT);
    pinMode(cfg::pins::SRStrobe, OUTPUT);

    pinMode(cfg::pins::leftMotorPWM, OUTPUT);
    pinMode(cfg::pins::rightMotorPWM, OUTPUT);

    pinMode(cfg::pins::armUnoSS, OUTPUT);
    pinMode(cfg::pins::SPIMISO, INPUT);
    pinMode(cfg::pins::SPIMOSI, OUTPUT);
    pinMode(cfg::pins::SPIClock, OUTPUT);

  //START SERIAL CONNECTION
    #ifdef DEBUG
      Serial.begin(cfg::k_serialBaudRate);
      Serial.println("Serial connected.");
    #endif

  //SETUP & CALIBRATE QTR
    #ifdef DEBUG
      Serial.println("QTR calibrating...");
    #endif
    qtr.setTypeAnalog();
    qtr.setEmitterPin(cfg::pins::emitter);
    qtr.setSensorPins(cfg::pins::qtrPins, cfg::k_sensorCount);
    for(uint8_t i = 0; i < cfg::k_calibrationCount; i++) {
      qtr.calibrate(cfg::k_readMode);
    }
    #ifdef DEBUG
      Serial.println("QTR calibrated.");
    #endif

  //SETUP SR
    sr.forcePush(0);
  //SETUP MOTOR DRIVER
    driver.drive(0, 0);
  //SETUP SPI 
    spi.enable();
}

void loop() {
  //TIME
    lastTime = currentTime;
    currentTime = micros();
    dT = static_cast<float>(currentTime - lastTime) / 1000000;

  //ERROR
    position = (lineColor) ? qtr.readLineWhite(sensorValues, cfg::k_readMode) : qtr.readLineBlack(sensorValues, cfg::k_readMode);
    lastError = currentError;
    currentError = static_cast<float>(position - (cfg::k_sensorCount * 500)) / (cfg::k_sensorCount * 500);

  //PID
    speedModifier = cfg::k_p * currentError + cfg::k_d * (currentError - lastError) / dT;
    driver.drive(cfg::k_base + speedModifier, cfg::k_base - speedModifier);
}
