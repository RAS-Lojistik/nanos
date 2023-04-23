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
    int16_t leftSpeed = 1;
    int16_t rightSpeed = 1;
  #endif
}

namespace {
  uint16_t numberOfCommands = 25;
  char commands[] = { 'f', 'f','f','l', 's' };
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
      pinMode(cfg::pins::qtr[i], INPUT);
    }
    pinMode(cfg::pins::leftMotor1, OUTPUT);
    pinMode(cfg::pins::leftMotor2, OUTPUT);
    pinMode(cfg::pins::leftMotorPWM, OUTPUT);
    pinMode(cfg::pins::rightMotor1, OUTPUT);
    pinMode(cfg::pins::rightMotor2, OUTPUT);
    pinMode(cfg::pins::rightMotorPWM, OUTPUT);

    pinMode(cfg::pins::button, INPUT);

    pinMode(cfg::pins::nrfInterrupt, INPUT);
    pinMode(cfg::pins::nrfSS, OUTPUT);
    digitalWrite(cfg::pins::nrfSS, HIGH);

    pinMode(cfg::pins::SPIMISO, INPUT);
    pinMode(cfg::pins::SPIMOSI, OUTPUT);
    pinMode(cfg::pins::SPIClock, OUTPUT);

  //START SERIAL CONNECTION
    Serial.begin(cfg::k_serialBaudRate);
    while(!Serial) {}

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
      while((millis() - lastTime) < cfg::k_calibrationMoveDuration / 2) {
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

  //WAIT BUTTON PRESS
    while(digitalRead(cfg::pins::button)) { }
    delay(1500);
}

void loop() {
  int bdry_qtr = 300;
  //TIME
    lastTime = currentTime;
    currentTime = micros();
    dT = currentTime - lastTime;

  //POSITION & LINE COLOR
    position = (lineColorFlag) ? qtr.readLineWhite(sensorValues, cfg::k_readMode) : qtr.readLineBlack(sensorValues, cfg::k_readMode);

    lastLineColorFlag = lineColorFlag;
    // IF BLACK LINE (WHITE -> BLACK)
    if(sensorValues[0] < bdry_qtr && sensorValues[5] < bdry_qtr) {
      if(lastLineColorFlag) {
        lineColorFlag = 0;
        position = qtr.readLineBlack(sensorValues, cfg::k_readMode);
      }
    }
    // IF WHITE LINE (BLACK -> WHITE)
    else if(sensorValues[0] > 1000-bdry_qtr && sensorValues[5] > 1000-bdry_qtr) {
      if(!lastLineColorFlag) {
        lineColorFlag = 1;
        position = qtr.readLineWhite(sensorValues, cfg::k_readMode);
      }
    }

  // ERROR & IS ON LINE
    lastError = currentError;
    currentError = position - ((cfg::k_sensorCount - 1) * 500);
    lastOnLineFlag = onLineFlag;
    if(abs(position - 2500) < 500) {
      onLineFlag = 1;
    }
    else {
      onLineFlag = 0;
    }

  //STATE MACHINE
    #ifdef LOGI_TEST
      PIDSpeedModifier = cfg::k_p * currentError + cfg::k_d * (currentError - lastError) / dT;
      driver.drive(cfg::k_base, cfg::k_base);
      /*char path[] = "ffrfrllllffrlsffrfrllllffrlsffrfrllllffrlsffrfrllllffrlsffrfrllllffrlsffrfrllllffrlsffrfrllllffrlsffrfrllllffrlsffrfrllllffrlsffrfrllllffrlsffrfrllllffrlsffrfrllllffrlsffrfrllllffrls";
      uint16_t pathSize = sizeof(path);
      sendCommandsWithNRF(path, pathSize);*/
    #else
      switch (commands[commandCounter]) {                   
        case 'f': //FOLLOW LINE UNTIL SENSOR REACHES A CROSSROAD
          if(sensorValues[0] < bdry_qtr && sensorValues[4] > 1000-bdry_qtr && sensorValues[5] > 1000-bdry_qtr) {
            //if(!junctionDoubleTestFlag) {
              //junctionDoubleTestFlag = 1;
              driver.drive(cfg::k_base, cfg::k_base*2);
              delay(50);
            //}
            /*else {
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
            }*/
              driver.drive(0, 0);
              // WAIT FOR OCR
              serialCom();
              //while(1) { }
              commandCounter++;
              break;   
          } 
          else {
            //junctionDoubleTestFlag = 0;
            PIDSpeedModifier = cfg::k_p * currentError + cfg::k_d * (currentError - lastError) / dT;
            PIDSpeedModifier = constrain(PIDSpeedModifier, -15, 15);
            if(lastLineColorFlag != lineColorFlag){
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
        case 'l': //GO FORWARD FOR A WHILE THEN TURN LEFT UNTIL SENSOR'S ON LINE AGAIN
          driver.drive(-cfg::k_turnLeftSpeed, cfg::k_turnLeftSpeed);
          if((!lastOnLineFlag) && onLineFlag) {
            commandCounter++;
          }
          break;

        case 's':
          if(!lastLineColorFlag && lineColorFlag) {
              driver.drive(cfg::k_base, cfg::k_base);
              delay(1000);
              driver.drive(0, 0);
              while(1) { }
          } else {
          PIDSpeedModifier = cfg::k_p * currentError + cfg::k_d * (currentError - lastError) / dT;
          PIDSpeedModifier = constrain(PIDSpeedModifier, -15, 15);
          driver.drive(cfg::k_base + PIDSpeedModifier, cfg::k_base - PIDSpeedModifier);
          }
          /* char* path = nullptr;
          uint16_t pathSize = getPathFromPi(&path);
          sendCommandsWithNRF(path, pathSize);
          */
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

void serialCom() {
  Serial.write(100);
  
  uint8_t responded = 0;
  while(!responded) {
    if(Serial.available()) {
      String data = Serial.readStringUntil('\n');
      if(data == "move") {
        responded = 1;
        Serial.write(45);
      }
    }
  }
}