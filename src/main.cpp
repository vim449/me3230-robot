#include <Arduino.h>
#include "BasicLinearAlgebra.h"

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
}

void loop() {
  // put your main code here, to run repeatedly:
  mapCenterOfRotation(0, 1);
}