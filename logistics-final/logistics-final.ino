#include "motorDriver.h"
#include "shiftRegister.h"
#include "debugLeds.h"
#include "external.h"
#include "altSPI.h"
#include "config.h"

#include <QTRSensors.h>

#include <math.h>
#include <string.h>
#include <inttypes.h>

uint16_t sensorValues[cfg::k_sensorCount] = { 0 };
QTRSensors qtr = QTRSensors();

ShiftRegister sr = ShiftRegister(cfg::pins::SRData, cfg::pins::SRClock, cfg::pins::SRStrobe);

MotorDriver driver = MotorDriver(&sr, cfg::pins::leftMotorPWM, cfg::srOrders::left1, cfg::srOrders::left2, 
                                      cfg::pins::rightMotorPWM, cfg::srOrders::right1, cfg::srOrders::right2);

DebugLeds leds = DebugLeds(&sr, cfg::srOrders::LEDRed, cfg::srOrders::LEDGreen, cfg::srOrders::LEDBlue);

AltSPI spi = AltSPI(cfg::k_SPIConfigByte);

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
    int16_t leftSpeed = 1;
    int16_t rightSpeed = 1;
  #endif
}

namespace {
  uint16_t numberOfCommands;
  char* commands = nullptr;
  uint16_t commandCounter = 0;

  volatile uint8_t lastLineColorFlag = 1;
  volatile uint8_t lineColorFlag = 1; //0 FOR BLACK, 1 FOR WHITE

  volatile uint8_t lastOnLineFlag = 1;
  volatile uint8_t onLineFlag = 1;

  uint8_t turnRunOnceFlag = 0;

  uint8_t junctionDoubleTestFlag = 0;
}

void setup() {
  //SETUP
    //SETUP PINS
      pinMode(cfg::pins::emitter, OUTPUT); 
      for(uint8_t i = 0; i < cfg::k_sensorCount; i++) {
        pinMode(cfg::pins::qtr[i], INPUT);
      } 

      pinMode(cfg::pins::SRClock, OUTPUT); 
      pinMode(cfg::pins::SRData, OUTPUT);
      pinMode(cfg::pins::SRStrobe, OUTPUT);

      pinMode(cfg::pins::leftMotorPWM, OUTPUT);
      pinMode(cfg::pins::rightMotorPWM, OUTPUT);

      pinMode(cfg::pins::nrfInterrupt, INPUT);
      pinMode(cfg::pins::nrfSS, OUTPUT);
      digitalWrite(cfg::pins::nrfSS, HIGH);

      pinMode(cfg::pins::armNanoSS, OUTPUT);
      digitalWrite(cfg::pins::armNanoSS, HIGH);

      pinMode(cfg::pins::SPIMISO, INPUT);
      pinMode(cfg::pins::SPIMOSI, OUTPUT);
      pinMode(cfg::pins::SPIClock, OUTPUT);

    //START SERIAL CONNECTION
      #ifdef DEBUG_SERIAL
        Serial.begin(cfg::k_serialBaudRate);
        while(!Serial) {}
        Serial.println("Serial connected.");
      #endif

    //SETUP SR
      sr.forcePush(0);

    //SETUP MOTOR DRIVER
      driver.drive(0, 0);
    
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

    //SETUP DEBUG LEDS
      leds.redOn();
      delay(333);
      leds.redOff();
      leds.greenOn();
      delay(333);
      leds.greenOff();
      leds.blueOn();
      delay(333);
      leds.blueOff();

  //GET COMMANDS
    leds.redOn();
    leds.greenOn();
    leds.blueOn();
    numberOfCommands = getCommandsWithNRF(&commands);
    leds.redOff();
    leds.greenOff();
    leds.blueOff();

  //SETUP SPI 
    spi.enable();
}

void loop() {
  //TIME
    lastTime = currentTime;
    currentTime = micros();
    dT = currentTime - lastTime;

  //POSITION & LINE COLOR  
    position = (lineColorFlag) ? qtr.readLineWhite(sensorValues, cfg::k_readMode) : qtr.readLineBlack(sensorValues, cfg::k_readMode);

    lastLineColorFlag = lineColorFlag;
    if(sensorValues[0] < 700 && ((sensorValues[1] < 300 && sensorValues[2] < 300 && sensorValues[3] > 700 && sensorValues[4] > 700) || 
                                 (sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] < 300 && sensorValues[4] < 300) ||
                                 (sensorValues[1] < 300 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[4] > 700) ||
                                 (sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[4] < 300) ||
                                 (sensorValues[1] < 300 && sensorValues[2] > 700 && sensorValues[3] > 700 && sensorValues[4] < 300 )) && sensorValues[5] < 700) {
      if(lastLineColorFlag) {
        lineColorFlag = 0;
        position = qtr.readLineBlack(sensorValues, cfg::k_readMode);
      }
    }
    else if(sensorValues[0] > 300 && ((sensorValues[1] > 700 && sensorValues[2] > 700 && sensorValues[3] < 300 && sensorValues[4] < 300) || 
                                      (sensorValues[1] < 300 && sensorValues[2] < 300 && sensorValues[3] > 700 && sensorValues[4] > 700) ||
                                      (sensorValues[1] > 700 && sensorValues[2] < 300 && sensorValues[3] < 300 && sensorValues[4] < 300) ||
                                      (sensorValues[1] < 300 && sensorValues[2] < 300 && sensorValues[3] < 300 && sensorValues[4] > 700) ||
                                      (sensorValues[1] > 700 && sensorValues[2] < 300 && sensorValues[3] < 300 && sensorValues[4] > 700)) && sensorValues[5] > 300) {
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
      leds.redToggle();
    }

  //ERROR & IS ON LINE
    lastError = currentError;
    currentError = position - ((cfg::k_sensorCount - 1) * 500);

    lastOnLineFlag = onLineFlag;
    if(abs(position - 2500) < 500) {
      onLineFlag = 1;
      leds.greenOn();
    }
    else {
      onLineFlag = 0;
      leds.greenOff();
    }

  //STATE MACHINE
    #ifdef LOGI_TEST
      spi.writeToSlave(cfg::k_armPickUpWord, cfg::pins::armNanoSS);
      delay(7000);
      spi.writeToSlave(cfg::k_armDropWord, cfg::pins::armNanoSS);
      delay(7000);
    #else
      switch (commands[commandCounter]) {                   
        case 'f': //FOLLOW LINE UNTIL SENSOR REACHES A CROSSROAD
          if((sensorValues[2] > 300 && sensorValues[3] > 300) && ((sensorValues[0] > 300 && sensorValues[1] > 300) || (sensorValues[4] > 300 && sensorValues[5] > 300))) {
            if(!junctionDoubleTestFlag) {
              junctionDoubleTestFlag = 1;
              driver.drive(cfg::k_base, cfg::k_base);
              delay(20);
            }
            else {
              if(commands[commandCounter + 1] == 'f') {
                driver.drive(cfg::k_base, cfg::k_base);
                delay(250);              
              }
              else {
                driver.drive(-200, -200);
                delay(35);
                driver.drive(0, 0);            
              }
              junctionDoubleTestFlag = 0;
              commandCounter++;
              leds.blueToggle();            
            }
          } 
          else {
            junctionDoubleTestFlag = 0;
            PIDSpeedModifier = cfg::k_p * currentError + cfg::k_d * (currentError - lastError) / dT;
            PIDSpeedModifier = constrain(PIDSpeedModifier, -35, 35);
            if(!(lastLineColorFlag == lineColorFlag)){
              driver.drive(cfg::k_base, cfg::k_base);
              if (lineColorFlag){
                position = qtr.readLineWhite(sensorValues, cfg::k_readMode);
                for(uint8_t i = 0; i < cfg::k_sensorCount; i++) {
                  sensorValues[i] = 1000 - sensorValues[i];
                }
                //delay(25);
                break;
              }
              else {
                position = qtr.readLineBlack(sensorValues, cfg::k_readMode);
              }
              //delay(25);
            }
            else {
            driver.drive(cfg::k_base + PIDSpeedModifier, cfg::k_base - PIDSpeedModifier);
            }
          }
          break;
        case 'r': //GO FORWARD FOR A WHILE. THEN TURN RIGHT UNTIL SENSOR'S ON LINE AGAIN
          if(!turnRunOnceFlag) {
            driver.drive(cfg::k_forwardSpeed, cfg::k_forwardSpeed);
            delay(cfg::k_forwardDuration);
            turnRunOnceFlag = 1;
            driver.drive(cfg::k_turnRightSpeed, -cfg::k_turnRightSpeed);
            delay(700);
          }
          driver.drive(cfg::k_turnRightSpeed, -cfg::k_turnRightSpeed);
          if((!lastOnLineFlag) && onLineFlag) {
            turnRunOnceFlag = 0;
            commandCounter++;
          }
          break;
        case 'l': //GO FORWARD FOR A WHILE. THEN TURN LEFT UNTIL SENSOR'S ON LINE AGAIN
          if(!turnRunOnceFlag) {
            driver.drive(cfg::k_forwardSpeed, cfg::k_forwardSpeed);
            delay(cfg::k_forwardDuration);
            turnRunOnceFlag = 1;
          }
          driver.drive(-cfg::k_turnLeftSpeed, cfg::k_turnLeftSpeed);
          if((!lastOnLineFlag) && onLineFlag) {
            turnRunOnceFlag = 0;
            commandCounter++;
          }
          break;
        case 'p': //MOVE FORWARD UNTIL LASER IS TRIGGERED. THEN PICKUP THE CUBE, MOVE BACK & TURN AROUND.
          spi.writeToSlave(cfg::k_armPickUpWord, cfg::pins::armNanoSS);
          delay(3000);
          break;
        case 'd':
          spi.writeToSlave(cfg::k_armDropWord, cfg::pins::armNanoSS);
          delay(3000);
          break;
        case 's':
          driver.drive(0, 0);
          delay(5000);
          commandCounter++;
          break;
      }
    #endif
    
    #ifdef DEBUG_SERIAL
      leftSpeed = cfg::k_base + PIDSpeedModifier;
      rightSpeed = cfg::k_base - PIDSpeedModifier;
      sprintf(serialPrintBuffer, "                                                                 Left: %" PRId16 "\n", leftSpeed);
      Serial.print(serialPrintBuffer);
      sprintf(serialPrintBuffer, "                                                                                                      Right: %" PRId16 "\n", rightSpeed);
      Serial.print(serialPrintBuffer);
      sprintf(serialPrintBuffer, "                                                                                                                          Error: %" PRId16 "\n", currentError);
      Serial.print(serialPrintBuffer);
      sprintf(serialPrintBuffer, "Sensor Values: %" PRIu16 "\t%" PRIu16 "\t%" PRIu16 "\t%" PRIu16 "\t%" PRIu16 "\t%" PRIu16 "\n", sensorValues[0], sensorValues[1], sensorValues[2],
                                                                                                                          sensorValues[3], sensorValues[4], sensorValues[5]);
      Serial.print(serialPrintBuffer);
    #endif
}