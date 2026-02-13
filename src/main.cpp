#include <Arduino.h>
#include "BasicLinearAlgebra.h"
#include "L298N.h"

const unsigned char M1_PWM = 6, M1_C = 38, M1_D = 39;
const unsigned char M2_PWM = 7, M2_C = 40, M2_D = 41;
const unsigned char M3_PWM = 5, M3_C = 42, M3_D = 43;
const unsigned char rack_PWM = 46, rack_C = 47, rack_D = 48;
const unsigned char conveyor_PWM = 45, conveyor_C = 49, conveyor_D = 50;

bool motors_exist = false;
L298N* drive_motors[3];
L298N* conveyor;
L298N* rack;

enum State{driving,
  storing,
  dispensing,
  pressButton, 
  firing, 
  charging,
  waitingForBlock,
  movingRack,
  waitingForData,
  softwareStop};

const long XBEE_BAUD = 115200;
const long USB_BAUD = 9600;
State state = waitingForData;
State nextState = state;

void serialSetup() {
  Serial.begin(USB_BAUD);
  Serial3.begin(XBEE_BAUD);
}

void motorPinSetup() {
  if (!motors_exist) {
  drive_motors[0] = new L298N(M1_PWM, M1_C, M1_D);
  drive_motors[1] = new L298N(M2_PWM, M2_C, M2_D);
  drive_motors[2] = new L298N(M3_PWM, M3_C, M3_D);
  rack = new L298N(rack_PWM, rack_C, rack_D);
  conveyor = new L298N(conveyor_PWM, conveyor_C, conveyor_D);
  motors_exist = true;
  }

  for (int i = 0; i < 3; i++) {
    drive_motors[i]->init();
  }
  conveyor->init();
  rack->init();
}

void sensorPinSetup() {

}

BLA::Matrix<3,3> mapCenterOfRotation(float x, float y) {
  BLA::Matrix<3,3> J;
  J(0,0) = 1; J(0, 1) = 0; J(0, 2) = 0;
  J(1,0) = 0; J(1, 1) = 1; J(1, 2) = y;
  J(2,0) = 0; J(2, 1) = 0; J(2, 2) = 0;
  return J;
}

void setup() {
  // put your setup code here, to run once:
  serialSetup();
  motorPinSetup();
  sensorPinSetup();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // mapCenterOfRotation(0, 1);
  for (int i = 0; i<5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }
  Serial.println("Test");
  delay(1000);
}