#include "BasicLinearAlgebra.h"
#include "Encoder.h"
#include "HardwareSerial.h"
#include "L298N.h"
#include "PWMServo.h"
#include "QTRSensors.h"
#include "sending.h"
#include <Arduino.h>

// turn off when not connected to pc for performance
#define DEBUG
#ifdef DEBUG
bool shouldPrint = true;
#endif

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
const uint8_t limitPins[2] = {51, 52};
const uint8_t encoderPins[6] = {2, 3, 18, 19, 20, 21};
const uint8_t motorCurrentPins[3] = {A3, A4, A5};
const uint8_t hallEffectPin = A6;

// motors
bool motors_exist = false; // to ensure motor objects are only created once
L298N *drive_motors[3];
L298N conveyor = L298N(conveyor_PWM, conveyor_C, conveyor_D);
L298N *rack;
PWMServo buttonServo, discardServo;
// maps from x' y' theta' space to motor space, can be found from getJacobian.m
const BLA::Matrix<3, 3, double> motorJacobian = {0,
                                                 1000.0 / 48.0,
                                                 158.688 / 48.0,
                                                 sqrt(3) / 0.096,
                                                 -500.0 / 48.0,
                                                 158.688 / 48.0,
                                                 sqrt(3) / 0.096,
                                                 500.0 / 48.0,
                                                 -158.688 / 48.0};
#define DISCARD_ANGLE 180
#define DISCARD_STORE_ANGLE 0
#define PRESS_ANGLE 180
#define PRESS_STORE_ANGLE 0

// sensors
QTRSensors lineQtr;
QTRSensors shovelQtr;
QTRSensors conveyorQtr;
const uint8_t qtrSensorCount = 8;
uint16_t qtrSensorValues[qtrSensorCount];
bool limitStates[3] = {
    true, true, true}; // switches are high by default and low when triggered
Encoder *encoders[3];
double hallEffect = 0.0;

// time variables
double t, t0;

// aliasing to xbee shield
// HardwareSerial* const xbee = &Serial3;
#define xbee Serial3

const long XBEE_BAUD = 115200;
const long USB_BAUD = 9600;

// control variables
State state;
State nextState;
BLA::Matrix<3, 1, double> motorSpeeds = {0, 0, 0};
double x_dot, y_dot, theta_dot;
int targetRack = 0;
int currentRack = 0; // index of limit switch the rack is resting at
double timerTarget = 0;
bool servoTarget = false; // true if the servo should be extended
uint8_t targetPress = 0;
uint8_t numPressed = 0; // number of times button pressed

// game variables
enum BlockType { none, wood, stone, iron, diamond };
BlockType stored[3] = {none, none, none};
BlockType pick = {wood};
BlockType sword = {wood};
bool shield = false;
bool needsDiscard = false;

void serialSetup() {
  Serial.begin(USB_BAUD);
  Serial.println("Initialized XBEE");
  xbee.begin(XBEE_BAUD);
}

void controlMotors(double xdot, double ydot, double thetadot) {
  BLA::Matrix<3, 1, double> in = {xdot, ydot, theta_dot};
  motorSpeeds = motorJacobian * in;

  for (int i = 0; i < 3; i++) {
    drive_motors[i]->setSpeed(motorSpeeds(i));
  }
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
    motors_exist = true;
  }

  for (auto &motor : drive_motors) {
    motor->init();
  }
  conveyor.init();
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

void startConveyorService(bool forwards) {
  conveyor.setSpeed(forwards ? 400 : -400);
}

void moveRackService(uint8_t target) {
  targetRack = target;
  if (targetRack > currentRack) {
    // rack needs to drive forwards
    // Serial.println("Rack driving forwards");
    rack->setSpeed(400); // TODO, determine if this is a reasonable speed
  } else if (targetRack < currentRack) {
    // rack needs to drive backwards
    // Serial.println("Rack driving backwards");
    rack->setSpeed(-400);
  } else {
    // somehow this function was called with the rack in the target pos
    // Serial.println("Something went wrong");
    rack->setBrake(400);
  }
}

void reset() {
  motorPinSetup();
  sensorPinSetup();
  // kill all motors
  for (auto &motor : drive_motors) {
    motor->setBrake(400);
  }
  conveyor.setBrake(400);
  rack->setBrake(400);
  // TODO reset encoders

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

  discardServo.write((int)t);

  // check for software stop
  // waitingForData checks for data using a different routine
  if (state != waitingForData && xbee.available() > 0) {
    if (xbee.read() == SOFTWARE_STOP) {
      reset();
    }
  }

  switch (state) {
  case driving:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state driving");
#endif
    if (t >= timerTarget) {
      // finished driving, for now go back to waiting for instructions
      for (auto &motor : drive_motors) {
        motor->setBrake(400);
        x_dot = 0;
        y_dot = 0;
        theta_dot = 0;
        nextState = waitingForData;
      }
      // TODO, refactor to not be open loop control
      controlMotors(x_dot, y_dot, theta_dot);
    }
    break;
  case storing:
    break;
  case dispensing:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state dispensing");
#endif
    if (t >= timerTarget) {
      conveyor.setBrake(400);
      stored[0] = stored[1];
      stored[1] = stored[2];
      stored[2] = none;
      // TODO, determine if robot should dispense again
      if (false) {
        startConveyorService(true);
        nextState = dispensing;
      } else {
        // TODO, determine needed x_dot, y_dot, theta_dot
        // nextState = driving;

        // For PM6, every actuation should go back to waitingForData
        nextState = waitingForData;
      }
    }
    break;
  case pressButton:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state pressButton");
#endif
    if (t >= timerTarget) {
      if (servoTarget) {
// servo finished extending, bring it back
#ifdef DEBUG
        Serial.println("Extended");
#endif
        buttonServo.write(PRESS_STORE_ANGLE);
        timerTarget = t + 1;
        servoTarget = false;
        numPressed++;
        nextState = pressButton;
      } else {
// servo finished retracting, extend again
#ifdef DEBUG
        Serial.println("Retracted");
#endif
        if (numPressed == targetPress) {
          // button has been pressed enough times
          numPressed = 0;
          targetPress = 0;
          nextState = waitingForData;
        } else {
          // button needs to be pressed again
          buttonServo.write(PRESS_ANGLE);
          servoTarget = true;
          timerTarget = t + 1;
          nextState = pressButton;
        }
      }
    }
    break;
  case waitingForBlock:
    // TODO
    // NOT NEEDED FOR PM6
    break;
  case movingRack:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state movingRack");
#endif
    if (digitalRead(limitPins[targetRack]) == LOW) {
      // rack is at target
      rack->setBrake(400);
      if (targetRack == 0) {
        // at front of robot, determine next state
        // For PM6, every actuation should go back to waitingForData
        nextState = waitingForData;
      } else if (targetRack == 1) {
        nextState = waitingForData;
        /*
        // at back of robot for discarding or dispensing, determine next state
        if (needsDiscard) {
          discardServo.write(DISCARD_ANGLE);
          // todo, figure out how long servo takes to swing
          servoTarget = true;
          nextState = discarding;
        } else {
          startConveyorService(true);
          nextState = dispensing;
        }
        */
      }
    }
  case waitingForData:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state waitingForData");
#endif
    // should be able to:
    // - drive any direction
    // turn left/right
    // press button X times
    // drive rack to X pos
    // discard

    // wireless strategy:
    // send a 3 byte packet
    // START state parameter
    if (xbee.available() >= 3) {
      if (xbee.read() == START_MESSAGE) {
#ifdef DEBUG
        Serial.println("XBEE triggered");
#endif
        nextState = numToState(xbee.read());
        char param = xbee.read();

        for (int i = 1; i > 10; i++) {
          xbee.read(); // flush xbee entirely
        }
        if (nextState == driving) {
          timerTarget = t + 1;
          switch (param) {
          case 'f':
            x_dot = 1;
            y_dot = 0;
            theta_dot = 0;
            break;
          case 'b':
            x_dot = -1;
            y_dot = 0;
            theta_dot = 0;
            break;
          case 'l':
            x_dot = 0;
            y_dot = -1;
            theta_dot = 0;
            break;
          case 'r':
            x_dot = 0;
            y_dot = 1;
            theta_dot = 0;
          case 'L':
            x_dot = 0;
            y_dot = 0;
            theta_dot = 1;
            break;
          case 'R':
            x_dot = 0;
            y_dot = 0;
            theta_dot = -1;
            break;
          default:
            x_dot = 0;
            y_dot = 0;
            theta_dot = 0;
          }
        } else if (nextState == pressButton) {
          targetPress = param - 48; // convert from ascii to dec
          timerTarget = t + 1;
          buttonServo.write(PRESS_ANGLE);
          servoTarget = true;
        } else if (nextState == discarding) {
          discardServo.write(DISCARD_ANGLE);
          timerTarget = t + 1;
        } else if (nextState == movingRack) {
          moveRackService(param - 48);
        } else if (nextState == dispensing) {
          startConveyorService(true);
          timerTarget = t + param - 48;
        }
      }
    }
    break;
  case discarding:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state discarding");
#endif
    if (t >= timerTarget) {
      if (servoTarget) {
        // servo finished extending, bring it back
        discardServo.write(DISCARD_STORE_ANGLE);
        timerTarget = t + 1;
        servoTarget = false;
        nextState = discarding;
      } else {
        // servo finished retracting, done discarding
        nextState = waitingForData;
      }
    }
    break;
  }
#ifdef DEBUG
  if (state != nextState) {
    shouldPrint = true;
  } else {
    shouldPrint = false;
  }
#endif
  state = nextState;
}