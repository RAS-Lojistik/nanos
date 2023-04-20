#include "config.h"

#include <QTRSensors.h>
#include <time.h>

QTRSensors qtr;

void setup() {
  pinMode(cfg::pins::leftMotor1, OUTPUT);
  pinMode(cfg::pins::leftMotor1, OUTPUT);
  pinMode(cfg::pins::leftMotor2, OUTPUT);
  pinMode(cfg::pins::leftMotorPWM, OUTPUT);
  pinMode(cfg::pins::rightMotor1, OUTPUT);
  pinMode(cfg::pins::rightMotor2, OUTPUT);
  pinMode(cfg::pins::rightMotorPWM, OUTPUT);
  pinMode(cfg::pins::emitter, OUTPUT);

  qtr.setTypeAnalog();
  qtr.setEmitterPin(cfg::pins::emitter);
  qtr.setSensorPins(cfg::pins::qtr, cfg::k_sensorCount);
  
#ifdef DEBUG
  Serial.begin(9600);
  Serial.println("Calibrating..."); 
#endif

  delay(3000);
  for (uint8_t i = 0; i < 120; i++) {
    qtr.calibrate(cfg::k_readMode);
  }  

#ifdef DEBUG
  Serial.println("Calibration complete!");
#endif
}

void loop()
{
  static uint16_t sensorValues[cfg::k_sensorCount] = {0};
  static uint32_t lastTime = -1;
  static float lastError = 0;
  static float iError = 0;

  float position = static_cast<float>(qtr.readLineBlack(sensorValues, cfg::k_readMode)) / ((cfg::k_sensorCount - 1) * 500);
  
  uint32_t currentTime = millis();
  float dT = (currentTime - lastTime) / 1000.0f;
  lastTime = currentTime;

  float error = 1 - position;
  float dError = ((error - lastError)) / dT;
  lastError = error;

  int16_t speedModifier = error * cfg::k_p + dError * cfg::k_d + iError * cfg::k_i;

#ifdef DEBUG
  for (int value : sensorValues)
  {
    Serial.print(value);
    Serial.print("\t");
  }
  Serial.print("error: "); 
  Serial.print(error);
  Serial.print("\tleft speed: ");
  Serial.print(cfg::k_base + speedModifier);
  Serial.print("\tright speed: ");
  Serial.println(cfg::k_base - speedModifier);
#endif
  
  driveMotors(cfg::k_base + speedModifier, cfg::k_base - speedModifier);
  delayMicroseconds(100);
}

void driveMotors(int16_t leftMotorSpeed, int16_t rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    digitalWrite(cfg::pins::leftMotor1, HIGH);
    digitalWrite(cfg::pins::leftMotor2, LOW);
  } 
  else {
    digitalWrite(cfg::pins::leftMotor1, LOW);
    digitalWrite(cfg::pins::leftMotor2, HIGH);
    leftMotorSpeed = -leftMotorSpeed;
  }
  
  if(rightMotorSpeed >= 0) {
    digitalWrite(cfg::pins::rightMotor1, HIGH);
    digitalWrite(cfg::pins::rightMotor2, LOW);
  } 
  else {
    digitalWrite(cfg::pins::rightMotor1, LOW);
    digitalWrite(cfg::pins::rightMotor2, HIGH);
    rightMotorSpeed = -rightMotorSpeed;
  }

  if(leftMotorSpeed > 255) leftMotorSpeed = 255;
  if(rightMotorSpeed > 255) rightMotorSpeed = 255;  
  analogWrite(cfg::pins::leftMotorPWM, leftMotorSpeed);
  analogWrite(cfg::pins::rightMotorPWM, rightMotorSpeed);
}
