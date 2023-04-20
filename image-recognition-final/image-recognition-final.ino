#include "motorDriver.h"
#include "debugLeds.h"
#include "external.h"
#include "config.h"

#include <math.h>
#include <string.h>
#include <inttypes.h>

QTRSensors qtr = QTRSensors();
uint16_t lastSensorValues[cfg::k_sensorCount] = { 0 };
uint16_t sensorValues[cfg::k_sensorCount] = { 0 };

MotorDriver driver = MotorDriver(cfg::pins::leftMotor1, cfg::pins::leftMotor2, cfg::pins::leftMotorPWM, cfg::pins::rightMotor1, cfg::pins::rightMotor2, cfg::pins::rightMotorPWM);

DebugLeds leds = DebugLeds(cfg::pins::redLED, cfg::pins::greenLED, cfg::pins::blueLED);

namespace {
  uint32_t lastTime = 1;
  uint32_t currentTime = 2;
  float dT = 1;

  uint16_t position = 0;
  float lastError = 0;
  float currentError = 0;

  float PIDSpeedModifier = 0.f;
  #ifdef DEBUG_SERIAL
    char* serialPrintBuffer = new char[100];
  #endif
}

namespace {
  uint16_t numberOfCommands;
  char* commands = nullptr;
  uint16_t commandCounter = 0;
  
  uint8_t lastLineColorFlag = 1;
  uint8_t lineColorFlag = 1; //0 FOR BLACK, 1 FOR WHITE

  uint8_t lastOnLineFlag = 1;
  uint8_t onLineFlag = 1;
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
    pinMode(cfg::pins::nrfSS, OUTPUT);
    pinMode(cfg::pins::SPIClock, OUTPUT);
    pinMode(cfg::pins::SPIMISO, INPUT);
    pinMode(cfg::pins::SPIMOSI, OUTPUT);

  //START SERIAL CONNECTION
    Serial.begin(cfg::k_serialBaudRate);

  //SETUP MOTOR DRIVER
    driver.drive(0, 0);
    #ifdef DEBUG_LED
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
    /* TO DO
    for(uint8_t i = 0; i < cfg::k_calibrationWiggleCount; i++) {
      lastTime = millis();
      driver.drive(-cfg::k_calibrationMoveSpeed, cfg::k_calibrationMoveSpeed);
      while((millis() - lastTime) / 1.5f < cfg::k_calibrationMoveDuration) {
        qtr.calibrate(cfg::k_readMode);
      }
      lastTime = millis();
      driver.drive(cfg::k_calibrationMoveSpeed, -cfg::k_calibrationMoveSpeed);
      while(millis() - lastTime < cfg::k_calibrationMoveDuration) {
        qtr.calibrate(cfg::k_readMode);
      }
      lastTime = millis();
      driver.drive(-cfg::k_calibrationMoveSpeed, cfg::k_calibrationMoveSpeed);
      while((millis() - lastTime) / 1.5f < cfg::k_calibrationMoveDuration) {
        qtr.calibrate(cfg::k_readMode);
      }
    }
    */
    for(uint8_t i = 0; i < 100; i++) {
      qtr.calibrate(cfg::k_readMode);
    }      
    #ifdef DEBUG_SERIAL
      Serial.println("QTR calibrated.");
    #endif
  

}

void loop() {
  //TIME
    lastTime = currentTime;
    currentTime = micros();
    dT = static_cast<float>(currentTime - lastTime) / 1000000;

  //POSITION & LINE COLOR
    memcpy(&lastSensorValues, &sensorValues, 2 * cfg::k_sensorCount);
    position = (lineColorFlag) ? qtr.readLineWhite(sensorValues, cfg::k_readMode) : qtr.readLineBlack(sensorValues, cfg::k_readMode);
    //ON BLACK LINE IF LAST 2 READINGS WERE BLACK ONLY IN THE MIDDLE && CONVERSE IS TRUE IF OTHER WAY AROUND
    if(lastSensorValues[0] + sensorValues[0] < 100 && lastSensorValues[1] + sensorValues[1] < 100 && lastSensorValues[4] + sensorValues[4] < 100 && lastSensorValues[5] + sensorValues[5] < 100 && sensorValues[2] > 800 && lastSensorValues[2] > 800 && sensorValues[3] > 800 && lastSensorValues[3] > 800) {
      if(lastLineColorFlag) {
        lineColorFlag = 0;
        position = (lineColorFlag) ? qtr.readLineWhite(sensorValues, cfg::k_readMode) : qtr.readLineBlack(sensorValues, cfg::k_readMode);
      }
    }
    else if (lastSensorValues[0] + sensorValues[0] > 1800 && lastSensorValues[1] + sensorValues[1] > 1800 && lastSensorValues[4] + sensorValues[4] > 1800 && lastSensorValues[5] + sensorValues[5] > 1800 && sensorValues[2] < 200 && lastSensorValues[2] < 200 && sensorValues[3] < 200 && lastSensorValues[3] < 200) {
      if(!lastLineColorFlag) {
        lineColorFlag = 1;
        position = (lineColorFlag) ? qtr.readLineWhite(sensorValues, cfg::k_readMode) : qtr.readLineBlack(sensorValues, cfg::k_readMode);
      }
    }
    #ifdef DEBUG_LED
      if(lastLineColorFlag != lineColorFlag) {
        leds.redToggle();              
      }
    #endif
    #ifdef DEBUG_SERIAL
      sprintf(serialPrintBuffer, "Sensor Values: %" PRIu16 "\t%" PRIu16 "\t%" PRIu16 "\t%" PRIu16 "\t%" PRIu16 "\t%" PRIu16 "\n", sensorValues[0], sensorValues[1], sensorValues[2],
                                                                                                                                  sensorValues[3], sensorValues[4], sensorValues[5]);
      Serial.print(serialPrintBuffer);
    #endif    

}