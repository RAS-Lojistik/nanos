#include <QTRSensors.h>

#include "motorDriver.h"
#include "shiftRegister.h"
#include "debugLeds.h"
#include "altSPI.h"
#include "extern.h"
#include "config.h"

QTRSensors qtr = QTRSensors();
uint16_t sensorValues[cfg::k_sensorCount] = { 0 };


ShiftRegister sr = ShiftRegister(cfg::pins::SRData, cfg::pins::SRClock, cfg::pins::SRStrobe);

MotorDriver driver = MotorDriver(&sr, cfg::pins::leftMotorPWM, cfg::srOrders::left1, cfg::srOrders::left2, 
                                      cfg::pins::rightMotorPWM, cfg::srOrders::right1, cfg::srOrders::right2);

DebugLeds leds = DebugLeds(&sr, cfg::srOrders::LEDRed, cfg::srOrders::LEDGreen, cfg::srOrders::LEDBlue);

AltSPI spi = AltSPI(cfg::k_SPIConfigByte);

namespace {
  uint32_t lastTime = 1;
  uint32_t currentTime = 2;
  float dT = 1;

  uint16_t position = 0;
  float lastError = 0;
  float currentError = 0;

  float speedModifier = 0.f;
}

namespace {
  uint16_t numberOfCommands;
  char* commands = nullptr;
  uint16_t commandCounter = 0;
  
  uint8_t lineColor = 0; //0 FOR BLACK, 1 FOR WHITE
}

void setup() {
  //SETUP
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
      digitalWrite(cfg::pins::armUnoSS, HIGH);
      pinMode(cfg::pins::SPIMISO, INPUT);
      pinMode(cfg::pins::SPIMOSI, OUTPUT);
      pinMode(cfg::pins::SPIClock, OUTPUT);

    //START SERIAL CONNECTION
      #ifdef DEBUG
        Serial.begin(cfg::k_serialBaudRate);
        Serial.println("Serial connected.");
      #endif

    //SETUP SR
      sr.forcePush(0);

    //SETUP MOTOR DRIVER
      driver.drive(0, 0);
      #ifdef DEBUG
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
      #ifdef DEBUG
        Serial.println("QTR calibrating...");
      #endif
      qtr.setTypeAnalog();
      qtr.setEmitterPin(cfg::pins::emitter);
      qtr.setSensorPins(cfg::pins::qtrPins, cfg::k_sensorCount);
      for(uint8_t i = 0; i < cfg::k_calibrationWiggleCount) {
        lastTime = millis();
        driver.drive(-cfg::k_calibrationMoveSpeed, cfg::k_calibrationMoveSpeed);
        while(millis() - lastTime/2 < cfg::k_calibrationMoveTime) {
          qtr.calibrate(cfg::k_readMode);
        }
        lastTime = millis();
        driver.drive(cfg::k_calibrationMoveSpeed, -cfg::k_calibrationMoveSpeed);
        while(millis() - lastTime < cfg::k_calibrationMoveTime) {
          qtr.calibrate(cfg::k_readMode);
        }
        lastTime = millis();
        driver.drive(-cfg::k_calibrationMoveSpeed, cfg::k_calibrationMoveSpeed);
        while(millis() - lastTime/2 < cfg::k_calibrationMoveTime) {
          qtr.calibrate(cfg::k_readMode);
        }
      }
      #ifdef DEBUG
        Serial.println("QTR calibrated.");
      #endif

    //SETUP DEBUG LEDS
      leds.redOff();
      leds.greenOff();
      leds.blueOff();
      #ifdef DEBUG
        leds.redOn();
        delay(333);
        leds.redOff();
        leds.greenOn();
        delay(333);
        leds.greenOff();
        leds.blueOn();
        delay(333);
        leds.blueOff();  
      #endif    
    //SETUP SPI 
      spi.enable();

  //GET COMMANDS
    numberOfCommands = GetCommandsWithNRF(&commands);
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

  //STATE MACHINE
    switch () {
      case 'r':
    }

  //PID
    speedModifier = cfg::k_p * currentError + cfg::k_d * (currentError - lastError) / dT;
    driver.drive(cfg::k_base + speedModifier, cfg::k_base - speedModifier);
}
