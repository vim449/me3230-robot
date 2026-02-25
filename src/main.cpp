// #define DEBUG

#include "BasicLinearAlgebra.h"
#include "Encoder.h"
#include "HardwareSerial.h"
#include "L298N.h"
#include "PWMServo.h"
#include "QTRSensors.h"
#include "followLine.h"
#include "globals.h"
#include "parseData.h"
#include "sending.h"
#include "services.h"
#include <Arduino.h>

// motors
bool motors_exist = false; // to ensure motor objects are only created once
L298N *drive_motors[3];
L298N *conveyor;
L298N *rack;
PWMServo buttonServo, discardServo;
// maps from x' y' theta' space to motor space, can be found from getJacobian.m
// TODO, this shit fucked
const BLA::Matrix<3, 3, double> motorJacobian = {0,
                                                 1000.0 / 48.0,
                                                 158.688 / 18.0,
                                                 sqrt(3) / 0.096,
                                                 -500.0 / 48.0,
                                                 158.688 / 18.0,
                                                 sqrt(3) / 0.096,
                                                 500.0 / 48.0,
                                                 -158.688 / 18.0};

// sensors
QTRSensors lineQtr;
QTRSensors shovelQtr;
QTRSensors conveyorQtr;
uint16_t lineValues[LINE_COUNT];
bool limitStates[2] = {
    true, true}; // switches are high by default and low when triggered
Encoder *encoders[3];
double hallEffect = 0.0;

// time variables
double t, t0, print_time;

// control variables
State state, nextState;
double x_dot, y_dot, theta_dot;
uint8_t targetRack = 0;
uint8_t currentRack = 0; // index of limit switch the rack is resting at
double timerTarget = 0;
bool servoTarget = false; // true if the servo should be extended
uint8_t targetPress = 0;
uint8_t numPressed = 0; // number of times button pressed
int16_t encoderCounts[3] = {0, 0, 0};
double range = 0;
bool rangeSet = false;
double rangeAlpha = 0.05;

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

void motorPinSetup() {
  if (!motors_exist) {
    Serial.println("Created Motors!");
    drive_motors[0] = new L298N(M1_PWM, M1_C, M1_D);
    encoders[0] = new Encoder(encoderPins[0], encoderPins[1]);
    drive_motors[1] = new L298N(M2_PWM, M2_C, M2_D);
    encoders[1] = new Encoder(encoderPins[2], encoderPins[3]);
    drive_motors[2] = new L298N(M3_PWM, M3_C, M3_D);
    encoders[2] = new Encoder(encoderPins[4], encoderPins[5]);
    conveyor = new L298N(conveyor_PWM, conveyor_C, conveyor_D);
    rack = new L298N(rack_PWM, rack_C, rack_D);
    motors_exist = true;
  }

  bool flipTable[3] = {false, false, true};
  for (int i = 0; i < 3; i++) {
    drive_motors[i]->init();
    drive_motors[i]->flip(flipTable[i]);
  }
  conveyor->init();
  rack->init();
  buttonServo.attach(buttonServo_PWM);
  buttonServo.write(PRESS_STORE_ANGLE);
  discardServo.attach(discardServo_PWM);
  discardServo.write(0);
}

void sensorPinSetup() {
  // set up for line following sensor
  lineQtr.setTypeRC();
  shovelQtr.setTypeRC();
  conveyorQtr.setTypeRC();
  lineQtr.setSensorPins(lineQtrPins, LINE_COUNT);
  shovelQtr.setSensorPins(shovelQtrPins, 1);
  conveyorQtr.setSensorPins(conveyorQtrPins, 1);

  // set up for limit switch
  for (auto &pin : limitPins) {
    pinMode(pin, INPUT_PULLUP);
  }
}

BLA::Matrix<3, 3> mapCenterOfRotation(float x, float y) {
  // TODO, this is not correct
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
  Serial.println("Reset triggered");
  motorPinSetup();
  sensorPinSetup();
  // kill all motors
  for (auto &motor : drive_motors) {
    motor->setBrake(400);
  }
  conveyor->setBrake(400);
  rack->setBrake(400);
  memset(&encoderCounts, 0, sizeof(encoderCounts));

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

  // line following dev
  //  PRINT STATEMENTS
  //  non-blocking way to delay printing
  lineQtr.read(lineValues);
  if (!rangeSet) {
    range = analogRead(A0);
  } else {
    range = rangeAlpha * analogRead(A0) + (1 - rangeAlpha) * range;
  }
  if ((t - print_time) > 0.25) {

    // This for loop is used to print out variables that are arrays
    // for (uint8_t i = 0; i < LINE_COUNT; i++) {
    //   Serial.print(lineValues[i]);
    //   Serial.print('\t');
    // }

    // Print any non-array variables here
    // Serial.println();
    print_time = t;
  }

  // check for software stop
  // waitingForData checks for data using a different routine
  bool shouldStop = false;
  if (state != waitingForData && xbee.available() > 0) {
    if (xbee.read() == SOFTWARE_STOP) {
      shouldStop = true;
    }
  }
  switch (state) {
  case driving:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state driving");
#endif
    if (getRangeDistance(range) < 15) {
      shouldStop = true;
    }
    if (shouldStop) {
      // finished driving, for now go back to waiting for instructions
      stopDriveMotors();
      nextState = waitingForData;
    } else {
      controlMotors();
    }
    break;
  case storing:
    break;
  case dispensing:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state dispensing");
#endif
    if (xbee.available() > 0) {
      if (xbee.read() == SOFTWARE_STOP) {
        conveyor->setBrake(400);
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
        buttonServo.write(30);
        timerTarget = t + PRESS_TIME;
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
          buttonServo.write(60);
          servoTarget = true;
          timerTarget = t + PRESS_TIME;
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
    parseData();
    break;
  case discarding:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state discarding");
#endif
    if (t >= timerTarget) {
      if (servoTarget) {
        // servo finished extending, bring it back
        Serial.println("bringing servo back");
        discardServo.write(30);
        timerTarget = t + 1;
        servoTarget = false;
        nextState = discarding;
      } else {
        // servo finished retracting, done discarding
        nextState = waitingForData;
        Serial.println("left discard");
      }
    }
    break;
  case lineFollowing:
    if (shouldStop) {
      stopDriveMotors();
      nextState = waitingForData;
    } else {
      followLine(LINE_FEED, LINE_KP);
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