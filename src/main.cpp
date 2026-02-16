#include "BasicLinearAlgebra.h"
#include "Encoder.h"
#include "HardwareSerial.h"
#include "L298N.h"
#include "PWMServo.h"
#include "QTRSensors.h"
#include "sending.h"
#include <Arduino.h>

// ALL PINS
const uint8_t M1_PWM = 6, M1_C = 38, M1_D = 39;
const uint8_t M2_PWM = 7, M2_C = 40, M2_D = 41;
const uint8_t M3_PWM = 5, M3_C = 42, M3_D = 43;
const uint8_t rack_PWM = 46, rack_C = 47, rack_D = 48;
const uint8_t conveyor_PWM = 45, conveyor_C = 49, conveyor_D = 50;
const uint8_t buttonServo_PWM = 11, discardServo_PWM = 12;
const uint8_t lineQtrPins[8] = {23, 25, 27, 29, 31, 33, 35, 37};
const uint8_t shovelQtrPins[1] = {34};
const uint8_t conveyorQtrPins[1] = {36};
const uint8_t limitPins[3] = {51, 52, 53};
const uint8_t encoderPins[6] = {2, 3, 18, 19, 20, 21};

// motors
bool motors_exist = false; // to ensure motor objects are only created once
L298N *drive_motors[3];
L298N *conveyor;
L298N *rack;
PWMServo buttonServo, discardServo;
double x_dot, y_dot, theta_dot;

// sensors
QTRSensors lineQtr;
QTRSensors shovelQtr;
QTRSensors conveyorQtr;
const uint8_t qtrSensorCount = 8;
uint16_t qtrSensorValues[qtrSensorCount];
bool limitStates[3] = {false, false, false};
Encoder *encoders[3];

// time variables
double t, t0;

// aliasing to xbee shield
// HardwareSerial* const xbee = &Serial3;
#define xbee Serial3

enum State {
  driving,
  storing,
  dispensing,
  pressButton,
  waitingForBlock,
  movingRack,
  waitingForData,
};

const long XBEE_BAUD = 115200;
const long USB_BAUD = 9600;
State state;
State nextState;

//game variables
enum BlockType {none, wood, stone, iron, diamond};
BlockType stored[3] = {none, none, none};
BlockType pick = {wood};
BlockType sword = {wood};
bool shield = false;

void serialSetup() {
  Serial.begin(USB_BAUD);
  Serial3.begin(XBEE_BAUD);
}

void controlMotors(double xdot, double ydot, double thetadot) {


}

void motorPinSetup() {
  if (!motors_exist) {
    drive_motors[0] = new L298N(M1_PWM, M1_C, M1_D);
    encoders[0] = new Encoder(encoderPins[0], encoderPins[1]);
    drive_motors[1] = new L298N(M2_PWM, M2_C, M2_D);
    encoders[1] = new Encoder(encoderPins[2], encoderPins[3]);
    drive_motors[2] = new L298N(M3_PWM, M3_C, M3_D);
    encoders[2] = new Encoder(encoderPins[4], encoderPins[5]);
    rack = new L298N(rack_PWM, rack_C, rack_D);
    conveyor = new L298N(conveyor_PWM, conveyor_C, conveyor_D);
    motors_exist = true;
  }

  for (auto &motor : drive_motors) {
    motor->init();
  }
  conveyor->init();
  rack->init();
  buttonServo.attach(buttonServo_PWM);
  discardServo.attach(discardServo_PWM);
}

void sensorPinSetup() {
  // set up for line following sensor
  lineQtr.setTypeRC();
  shovelQtr.setTypeRC();
  conveyorQtr.setTypeRC();
  lineQtr.setSensorPins(lineQtrPins, qtrSensorCount);
  shovelQtr.setSensorPins(shovelQtrPins, 1);
  conveyorQtr.setSensorPins(conveyorQtrPins, 1);

  // set up for limit switch
  for (auto &pin : limitPins) {
    pinMode(pin, INPUT_PULLUP);
  }
}

BLA::Matrix<3, 3> mapCenterOfRotation(float x, float y) {
  BLA::Matrix<3, 3> J;
  J(0, 0) = 1;
  J(0, 1) = 0;
  J(0, 2) = 0;
  J(1, 0) = 0;
  J(1, 1) = 1;
  J(1, 2) = y;
  J(2, 0) = 0;
  J(2, 1) = 0;
  J(2, 2) = 0;
  return J;
}

void reset() {
  motorPinSetup();
  sensorPinSetup();
  t0 = micros() / 1000000.;
  state = waitingForData;
  nextState = state;
}

void setup() {
  // put your setup code here, to run once:
  serialSetup();
  pinMode(LED_BUILTIN, OUTPUT);
  reset();
}

void loop() {
  t = micros() / 1000000. - t0;

  // check for software stop
  if (xbee.available() > 0) {
    if (xbee.read() == SOFTWARE_STOP) {
      // kill all motors
      for (auto &motor : drive_motors) {
        motor->setBrake(400);
      }
      conveyor->setBrake(400);
      rack->setBrake(400);
      // TODO reset encoders
      reset();
    }
  }

  switch (state) {
  case driving:
  case storing:
  case dispensing:
  case pressButton:
  case waitingForBlock:
  case movingRack:
  case waitingForData:
    break;
  }
  state = nextState;
}