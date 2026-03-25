// extern dependencies
#include <Arduino.h>
#include <Encoder.h>
#include <HardwareSerial.h>
#include <L298N.h>
#include <PWMServo.h>
#include <QTRSensors.h>

// internal files
#include "drive.h"
#include "followLine.h"
#include "globals.h"
#include "parseData.h"
#include "sending.h"
#include "services.h"

#include "init.h"

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

  bool flipTable[NUM_MOTORS] = {false, false, true};
  for (int i = 0; i < NUM_MOTORS; i++) {
    drive_motors[i]->init();
    drive_motors[i]->flip(flipTable[i]);
  }
  conveyor->init();
  rack->init();
  conveyor->flip(true);

  buttonServo.attach(buttonServo_PWM);
  discardServo.attach(discardServo_PWM);
  gateServo.attach(gateServo_PWM);
  discardServo.write(DISCARD_STORE_ANGLE);
  delay(20);
  buttonServo.write(PRESS_STORE_ANGLE);
  delay(20);
  gateServo.write(0);
  delay(20);
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

  // set up for color sensor
  pinMode(colorS0, OUTPUT);
  pinMode(colorS1, OUTPUT);
  pinMode(colorS2, OUTPUT);
  pinMode(colorS3, OUTPUT);
  pinMode(colorLED, OUTPUT);
  pinMode(colorReadPin, INPUT);
  // frequency scaling
  digitalWrite(colorS0, HIGH);
  digitalWrite(colorS1, LOW);
  // turn on color sensor LED
  digitalWrite(colorLED, LOW);
}

void homeRack() {
  Serial.println("Homing Rack");
  limitStates[1] = digitalRead(limitPins[1]);
  rack->setSpeed(20);
  delay(10);
  rack->setSpeed(0);
  while (limitStates[1] == 1) {
    limitStates[0] = digitalRead(limitPins[0]);
    limitStates[1] = digitalRead(limitPins[1]);
    rack->setSpeed(-250);
  }
  rack->setSpeed(0);
  Serial.println("Done Homing Rack");
  currentRack = 1;
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
  resetEncoders();
  instantiateTargets();

  t0 = micros() / 1000000.;
  state = waitingForData;
  nextState = state;

  rangeBack = analogRead(rangeBackPin);
  rangeFront = analogRead(rangeFrontPin);
  instantiateTargets();
}

void setup() {
  // put your setup code here, to run once:
  serialSetup();
  pinMode(LED_BUILTIN, OUTPUT);
  reset();
  homeRack();
}

float print_time = 0;
void loop() {
  t_old = t;
  t = micros() / 1000000.0 - t0;
  dt = t - t_old;

  //  PRINT STATEMENTS
  //  non-blocking way to delay printing
  rangeBack =
      rangeAlpha * analogRead(rangeBackPin) + (1 - rangeAlpha) * rangeBack;
  rangeFront =
      rangeAlpha * analogRead(rangeFrontPin) + (1 - rangeAlpha) * rangeFront;
  limitStates[0] = digitalRead(limitPins[0]);
  limitStates[1] = digitalRead(limitPins[1]);
  if ((t - print_time) > 3) {

    // This for loop is used to print out variables that are arrays
    // for (uint8_t i = 0; i < LINE_COUNT; i++) {
    //   Serial.print(lineValues[i]);
    //   Serial.print('\t');
    // }

    shovelQtr.read(shovelQtrValues);
    conveyorQtr.read(conveyorQtrValues);
    // Print any non-array variables here
    // Serial.print("Back sensor: ");
    // Serial.print(rangeBack);
    // Serial.print("\t Front sensor: ");
    // Serial.println(rangeFront);
    // Serial.println("Line Break Sensors");
    // Serial.print("Shovel: ");
    // Serial.println(shovelQtrValues[0]);
    // Serial.print("Conveyor: ");
    // Serial.println(conveyorQtrValues[0]);

    // Serial.println("Limit Switch Values");
    // Serial.print("Front");
    // Serial.println(limitStates[0]);
    // Serial.print("Back");
    // Serial.println(limitStates[1]);
    print_time = t;
  }

  // check for software stop
  // waitingForData checks for data using a different routine
  bool shouldStop = false;
  if (state != waitingForData && xbee.available() > 0) {
    if (xbee.read() == SOFTWARE_STOP) {
      shouldStop = true;
      reset();
    }
  }
  switch (state) {
  case driving:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state driving");
#endif
    if (getRangeDistance(rangeBack, BACK) < 15) {
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
    if (t >= timerTarget) {
      shouldStop = true;
    }
    if (shouldStop) {
      nextState = waitingForData;
      conveyor->setSpeed(0);
      stored[2] = stored[1];
      stored[1] = stored[0];
      stored[0] = inShovel;
    }
    break;
  case dispensing:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state dispensing");
#endif
    if (conveyorQtrValues[0] <= 0) {
      shouldStop = true;
    }
    if (t >= timerTarget) {
      shouldStop = true;
    }

    if (shouldStop) {
      conveyor->setBrake(400);
      stored[0] = stored[1];
      stored[1] = stored[2];
      stored[2] = none;
      // TODO, determine if robot should dispense again
      // if (stored[0] != none) {
      //   startConveyorService(true);
      //   nextState = dispensing;
      // } else {
      //   moveRackService(0);
      // }

      nextState = waitingForData;
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

          // nextState = waitingForBlock;
          // number of seconds for block detection timeout
          timerTarget = t + 20;
        } else {
          // button needs to be pressed again
          buttonServo.write(PRESS_ANGLE);
          servoTarget = true;
          timerTarget = t + PRESS_TIME;
          nextState = pressButton;
        }
      }
    }
    break;
  case waitingForBlock:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state waitngForBlock");
#endif
    // TODO, implement break sensor threshold
    shovelQtr.read(shovelQtrValues);
    if (t >= timerTarget || shovelQtrValues[0] <= 0) {
      shouldStop = true;
    }
    if (shouldStop) {
      if (analogRead(hallEffectPin) < 200) {
        targetPress = getSilverfishHits();
        // press button again
        buttonServo.write(PRESS_ANGLE);
        servoTarget = true;
        timerTarget = t + PRESS_TIME;
        nextState = pressButton;
      } else {
        // not a silver fish, good to go back
        // x_dot = -2.0;
        // y_dot = 0;
        // theta_dot = 0;
        // nextState = driving;
        timerTarget = t + STORE_TIME;
        startConveyorService(true);
        nextState = storing;
      }
    }

    break;
  case movingRack:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state movingRack");
#endif
    if (limitStates[targetRack] == 0) {
      // rack is at target
      Serial.println("Rack Stopped");
      rack->setSpeed(0);
      conveyor->setSpeed(0);
      currentRack = targetRack;
      nextState = waitingForData;
      // if (currentRack == 0) {
      //   // at front of robot, determine next state
      //   nextState = waitingForData;
      // } else if (targetRack == 1) {
      //   needsDiscard = false;
      //   if (needsDiscard) {
      //     discardServo.write(DISCARD_ANGLE);
      //     servoTarget = true;
      //     timerTarget = t + DISCARD_TIME;
      //     nextState = discarding;
      //   } else {
      //     // startConveyorService(true);
      //     // nextState = dispensing;
      //     nextState = waitingForData;
      //   }
      // }
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
        discardServo.write(DISCARD_STORE_ANGLE);
        timerTarget = t + DISCARD_TIME;
        servoTarget = false;
        nextState = discarding;
      } else {
        // servo finished retracting, done discarding
        // moveRackService(0);
        nextState = waitingForData;
      }
    }
    break;
  case lineFollowing:
#ifdef DEBUG
    if (shouldPrint)
      Serial.println("Entered state line following");
#endif
    distToWall = min(getRangeDistance(rangeBack, BACK),
                     getRangeDistance(rangeFront, FRONT));
    if (distToWall <= minDist) {
      shouldStop = true;
    }
    if (shouldStop) {
      stopDriveMotors();
      if (getRangeDistance(rangeFront, FRONT) <= minDist + stopDist) {
        senseColorService();
      } else {
        nextState = waitingForData;
      }
      fullJacobian = motorJacobian; // reset center of rotation
    } else {
      x_dot = map((float)min(distToWall, minDist + stopDist), minDist,
                  minDist + stopDist, 0.2, 2.1);
      followLine(x_dot);
    }
    break;
  case coasting:
    if (shouldStop) {
      stopDriveMotors();
      nextState = waitingForData;
    }
    coastMotors();
    break;
  case trajectory:
    if (shouldStop) {
      stopDriveMotors();
      nextState = waitingForData;
      break;
    }
    readEncoders();
    controlMotorsPathed();
    break;
  case moveGate:
    if (timerTarget >= t) {
      nextState = waitingForData;
    }
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
