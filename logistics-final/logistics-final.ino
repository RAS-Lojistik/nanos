#include "motorDriver.h"
#include "shiftRegister.h"
#include "debugLeds.h"
#include "altSPI.h"
#include "extern.h"
#include "config.h"

#include <QTRSensors.h>

#include <math.h>
#include <string.h>
#include <inttypes.h>

QTRSensors qtr = QTRSensors();
uint16_t lastSensorValues[cfg::k_sensorCount] = { 0 };
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
      #ifdef DEBUG_SERIAL
        Serial.begin(cfg::k_serialBaudRate);
        Serial.println("Serial connected.");
      #endif

    //SETUP SR
      sr.forcePush(0);

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
      qtr.setSensorPins(cfg::pins::qtrPins, cfg::k_sensorCount);
      for(uint8_t i = 0; i < cfg::k_calibrationWiggleCount; i++) {
        lastTime = millis();
        driver.drive(-cfg::k_calibrationMoveSpeed, cfg::k_calibrationMoveSpeed);
        while(millis() - lastTime/2 < cfg::k_calibrationMoveDuration) {
          qtr.calibrate(cfg::k_readMode);
        }
        lastTime = millis();
        driver.drive(cfg::k_calibrationMoveSpeed, -cfg::k_calibrationMoveSpeed);
        while(millis() - lastTime < cfg::k_calibrationMoveDuration) {
          qtr.calibrate(cfg::k_readMode);
        }
        lastTime = millis();
        driver.drive(-cfg::k_calibrationMoveSpeed, cfg::k_calibrationMoveSpeed);
        while(millis() - lastTime/2 < cfg::k_calibrationMoveDuration) {
          qtr.calibrate(cfg::k_readMode);
        }
      }
      #ifdef DEBUG_SERIAL
        Serial.println("QTR calibrated.");
      #endif

    //SETUP DEBUG LEDS
      leds.redOff();
      leds.greenOff();
      leds.blueOff();
      #ifdef DEBUG_LED
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
  //WAIT FOR BUTTON
    //TO DO
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

  //ERROR & IS ON LINE
    lastError = currentError;
    currentError = static_cast<float>(position - (cfg::k_sensorCount * 500)) / (cfg::k_sensorCount * 500);
    lastOnLineFlag = onLineFlag;
    if(fabs(lastError) < cfg::k_onLineThreshold && fabs(currentError) < cfg::k_onLineThreshold) {
      onLineFlag = 1;
    }

  //STATE MACHINE
    switch (commands[commandCounter]) {
      case 'f': 
        if (!lineColorFlag) {
          if ((sensorValues[2] > 850 && sensorValues[3] > 850) && ((sensorValues[0] > 850 && sensorValues[1] > 850) || (sensorValues[4] > 850 && sensorValues[5] > 850))) {
            driver.drive(cfg::k_forwardSpeed, cfg::k_forwardSpeed);
            delay(cfg::k_forwardDuration);
            commandCounter++;
            #ifdef DEBUG_LED
              leds.blueToggle();            
            #endif
            break;
          }                        
        }
        else {
          if ((sensorValues[2] < 150 && sensorValues[3] < 150) && ((sensorValues[0] < 150 && sensorValues[1] < 150) || (sensorValues[4] < 150 && sensorValues[5] < 150))) {
            driver.drive(cfg::k_forwardSpeed, cfg::k_forwardSpeed);
            delay(cfg::k_forwardDuration);
            commandCounter++;
            #ifdef DEBUG_LED
              leds.blueToggle();
            #endif
            break;
          }
        }
        speedModifier = cfg::k_p * currentError + cfg::k_d * (currentError - lastError) / dT;
        driver.drive(cfg::k_base + speedModifier, cfg::k_base - speedModifier);
        #ifdef DEBUG_SERIAL
          sprintf(serialPrintBuffer, "Motor Speeds: %f\t%f\n", cfg::k_base + speedModifier, cfg::k_base - speedModifier);
          Serial.print()
        #endif
        break;
      case 'r':
        driver.drive(cfg::k_turnRightSpeed, -cfg::k_turnRightSpeed);
        if(!lastOnLineFlag && onLineFlag) {
          commandCounter++;
          #ifdef DEBUG_LED
              leds.blueToggle();
          #endif
        }
        break;
      case 'l':
        driver.drive(-cfg::k_turnLeftSpeed, -cfg::k_turnLeftSpeed);
        if(!lastOnLineFlag && onLineFlag) {
          commandCounter++;
          #ifdef DEBUG_LED
              leds.blueToggle();
          #endif
        }
        break;
      case 'p':
        break;
      case 'd':
         break;
    }
}
