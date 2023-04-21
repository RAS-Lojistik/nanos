#include "motorDriver.h"
#include "external.h"
#include "config.h"

#include <math.h>
#include <string.h>
#include <inttypes.h>

uint16_t sensorValues[cfg::k_sensorCount] = { 0 };
QTRSensors qtr = QTRSensors();

MotorDriver driver = MotorDriver(cfg::pins::leftMotor1, cfg::pins::leftMotor2, cfg::pins::leftMotorPWM, cfg::pins::rightMotor1, cfg::pins::rightMotor2, cfg::pins::rightMotorPWM);

namespace {
  uint32_t lastTime = 1;
  uint32_t currentTime = 2;
  uint32_t dT = 1;

  int16_t position = 0;
  int16_t lastError = 0;
  int16_t currentError = 0;

  int16_t PIDSpeedModifier = 0;
  #ifdef DEBUG_SERIAL
    char* serialPrintBuffer = new char[250];
  #endif
}

namespace {
  uint16_t numberOfCommands;
  char commands[15] = { 'f', 'c', 'f', 'c', 'f', 'c', 'f', 'c', 
                        'f', 'c', 'f', 'c', 'f', 'c', 'f', 'c', 'l',
                        'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 's' };
  uint16_t commandCounter = 0;

  volatile uint8_t lastLineColorFlag = 1;
  volatile uint8_t lineColorFlag = 1; //0 FOR BLACK, 1 FOR WHITE
  volatile uint32_t lineColorTime = 1;

  volatile uint8_t lastOnLineFlag = 1;
  volatile uint8_t onLineFlag = 1;

  uint8_t turnRunOnceFlag = 0;

  uint8_t junctionDoubleTestFlag = 0;
}

void setup() {
  //SETUP PINS
    pinMode(cfg::pins::emitter, OUTPUT);
    for(uint8_t i = 0; i < cfg::k_sensorCount; i++) {
      pinMode(cfg::pins::qtr[i], OUTPUT);
    }
    pinMode(cfg::pins::leftMotor1, OUTPUT);
    pinMode(cfg::pins::leftMotor2, OUTPUT);
    pinMode(cfg::pins::leftMotorPWM, OUTPUT);
    pinMode(cfg::pins::rightMotor1, OUTPUT);
    pinMode(cfg::pins::rightMotor2, OUTPUT);
    pinMode(cfg::pins::rightMotorPWM, OUTPUT);

    pinMode(cfg::pins::nrfInterrupt, INPUT);
    pinMode(cfg::pins::nrfInterrupt, INPUT);
    pinMode(cfg::pins::nrfSS, OUTPUT);
    pinMode(cfg::pins::SPIClock, OUTPUT);
    pinMode(cfg::pins::SPIMISO, INPUT);
    pinMode(cfg::pins::SPIMOSI, OUTPUT);

  //START SERIAL CONNECTION
    Serial.begin(cfg::k_serialBaudRate);
    while(!Serial) {}

  //SETUP MOTOR DRIVER
    driver.drive(0, 0);
    #ifdef DEBUG_MOTOR
      driver.drive(250, 250);
      delay(500);
      driver.drive(-250, -250);
      delay(500);
      driver.drive(0, 250);
      delay(500);
      driver.drive(250, 0);
      delay(500);
      driver.drive(0, 0);
    #endif

  //SETUP & CALIBRATE QTR
    #ifdef DEBUG_SERIAL
      Serial.println("QTR calibrating...");
    #endif
    qtr.setTypeAnalog();
    qtr.setEmitterPin(cfg::pins::emitter);
    qtr.setSensorPins(cfg::pins::qtr, cfg::k_sensorCount);
    for(uint8_t i = 0; i < cfg::k_calibrationWiggleCount; i++) {
      lastTime = millis();
      driver.drive(-cfg::k_calibrationMoveSpeed, cfg::k_calibrationMoveSpeed);
      while((millis() - lastTime) < cfg::k_calibrationMoveDuration / 2.f) {
        qtr.calibrate(cfg::k_readMode);
      }
      lastTime = millis();
      driver.drive(cfg::k_calibrationMoveSpeed, -cfg::k_calibrationMoveSpeed);
      while(millis() - lastTime < cfg::k_calibrationMoveDuration) {
        qtr.calibrate(cfg::k_readMode);
      }
      lastTime = millis();
      driver.drive(-cfg::k_calibrationMoveSpeed, cfg::k_calibrationMoveSpeed);
      while((millis() - lastTime) < cfg::k_calibrationMoveDuration / 2) {
        qtr.calibrate(cfg::k_readMode);
      }
      driver.drive(0, 0);
    }
    #ifdef DEBUG_SERIAL
      Serial.println("QTR calibrated.");
    #endif
}

void loop() {
  //TIME
    lastTime = currentTime;
    currentTime = micros();
    dT = currentTime - lastTime;

  //POSITION & LINE COLOR
    position = (lineColorFlag) ? qtr.readLineWhite(sensorValues, cfg::k_readMode) : qtr.readLineBlack(sensorValues, cfg::k_readMode);
    lastLineColorFlag = lineColorFlag;
    if(sensorValues[0] < 300 && ((sensorValues[1] < 300 && sensorValues[2] < 300 && sensorValues[3] > 700 && sensorValues[4] > 700) || 
                                 (sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] < 300 && sensorValues[4] < 300) || 
                                 (sensorValues[1] < 300 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[4] < 300 )) && sensorValues[5] < 300) {
      if(lastLineColorFlag) {
        lineColorFlag = 0;
        position = qtr.readLineBlack(sensorValues, cfg::k_readMode);
      }
    }
    else if(sensorValues[0] > 700 && ((sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] < 300 && sensorValues[4] < 300) || 
                                      (sensorValues[1] < 300 && sensorValues[2] < 300 && sensorValues[3] > 700 && sensorValues[4] > 700) || 
                                      (sensorValues[1] > 700 && sensorValues[2] < 300 && sensorValues[3] < 300 && sensorValues[4] > 700)) && sensorValues[5] > 700) {
      if(!lastLineColorFlag) {
        lineColorFlag = 1;
        position = qtr.readLineWhite(sensorValues, cfg::k_readMode);
      }
    }
    if(lineColorFlag) {
      for(uint8_t i = 0; i < cfg::k_sensorCount; i++) {
        sensorValues[i] = 1000 - sensorValues[i];
      }
    }
    if(lastLineColorFlag != lineColorFlag) {
      lineColorTime = micros() - 1;           
    }

  // ERROR & IS ON LINE
    lastError = currentError;
    currentError = position - ((cfg::k_sensorCount - 1) * 500);
    lastOnLineFlag = onLineFlag;
    if((sensorValues[2] > 500 && sensorValues[3] > 300) || (sensorValues[2] > 300 && sensorValues[3] > 500)) {
      onLineFlag = 1;
      leds.greenOn();
    }
    else {
      onLineFlag = 0;
      leds.greenOff();
    }
  //
}