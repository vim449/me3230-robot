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
#include "transfer_function.h"

#include "init.h"

void serialSetup() {
    Serial.begin(USB_BAUD);
    // Serial.println("Initialized XBEE");
    xbee.begin(XBEE_BAUD);
}

void motorPinSetup() {
    if (!motors_exist) {
        // Serial.println("Created Motors!");
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
    rack->flip(true);
    conveyor->setSpeed(0);
    rack->setSpeed(0);

    buttonServo.attach(buttonServo_PWM);
    discardServo.attach(discardServo_PWM);
    gateServo.attach(gateServo_PWM);
    // These servos need to be zero'd
    discardServo.write(DISCARD_STORE_ANGLE);
    delay(20);
    buttonServo.write(PRESS_STORE_ANGLE);
    delay(20);
    gateServo.write(GATE_START_ANGLE);
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
    limitStates[0] = digitalRead(limitPins[0]);
    rack->setSpeed(-20);
    delay(10);
    rack->setSpeed(0);
    while (limitStates[0] == 1) {
        limitStates[0] = digitalRead(limitPins[0]);
        limitStates[1] = digitalRead(limitPins[1]);
        limitStates[2] = digitalRead(limitPins[2]);
        rack->setSpeed(350);
    }
    rack->setSpeed(0);
    Serial.println("Done Homing Rack");
    currentRack = 0;
}

void homeShovel() {
    limitStates[2] = digitalRead(limitPins[2]);
    while (limitStates[2] == 1) {
        limitStates[2] = digitalRead(limitPins[2]);
        conveyor->setSpeed(350);
    }
    conveyor->setSpeed(0);
}

void reset() {
    shouldReset = false;
    // Serial.println("Reset triggered");
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
    compMode = false;

    rangeBack = analogRead(rangeBackPin);
    rangeFront = analogRead(rangeFrontPin);
}

void setup() {
    // put your setup code here, to run once:
    serialSetup();
    pinMode(LED_BUILTIN, OUTPUT);
    reset();
    // set to false to do matlab connected transfer function derivation
    if (true) {
        gateServo.write(GATE_START_ANGLE);
        homeRack();
        homeShovel();
    } else {
        loop_cltf(0, 200, 200);
    }
    t0 = micros() / 1000000.;

    // startGame();
}

float print_time = 0;
void loop() {
    t_old = t;
    t = micros() / 1000000.0 - t0;
    dt = t - t_old;

    // get critical sensors
    rangeBack =
        rangeAlpha * analogRead(rangeBackPin) + (1 - rangeAlpha) * rangeBack;
    rangeFront =
        rangeAlpha * analogRead(rangeFrontPin) + (1 - rangeAlpha) * rangeFront;
    limitStates[0] = digitalRead(limitPins[0]);
    limitStates[1] = digitalRead(limitPins[1]);
    limitStates[2] = digitalRead(limitPins[2]);

    // check for software stop
    // waitingForData checks for data using a different routine
    bool shouldStop = false;
    if (state != waitingForData && xbee.available() > 0) {
        if (xbee.peek() == SOFTWARE_END) {
            shouldStop = true;
            nextState = waitingForData;
        } else if (xbee.peek() == SOFTWARE_STOP) {
            shouldReset = true;
        }
        xbee.read(); // need to control char from buffer
    }

    if (shouldReset) {
        reset();
    }

#ifdef DEBUG
    if ((t - print_time) > 0.5) {
        // This for loop is used to print out variables that are arrays
        // for (uint8_t i = 0; i < LINE_COUNT; i++) {
        //   Serial.print(lineValues[i]);
        //   Serial.print('\t');
        // }
        // shovelQtr.read(shovelQtrValues);
        // conveyorQtr.read(conveyorQtrValues);
        // Serial.print(shovelQtrValues[0]);
        // Serial.print('\t');
        // Serial.println(conveyorQtrValues[0]);

        // Serial.print(limitStates[0]);
        // Serial.print('\t');
        // Serial.print(limitStates[1]);
        // Serial.print('\t');
        // Serial.println(limitStates[2]);

        print_time = t;
    }
#endif

    switch (state) {
    case drivingTimeBased:
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
    case storingBlock:
        // timer target just serves as a lock out window so that the switch
        // being pressed initially doesn't stop things
        if (t >= timerTarget && limitStates[2] == false) {
            shouldStop = true;
        }
        if (shouldStop) {
            inShovel = wood;
            conveyor->setSpeed(0);
            stored[2] = stored[1];
            stored[1] = stored[0];
            stored[0] = inShovel;
            // check if comp code
            if (compMode) {
                strat->nextGoalCallback();
            } else {
                nextState = waitingForData;
            }
        }
        break;
    case dispensing:
        conveyorQtr.read(conveyorQtrValues);
        if (conveyorQtrValues[0] < 2200) {
            shouldStop = true;
        }
        if (shouldStop) {
            conveyor->setBrake(400);
            stored[0] = stored[1];
            stored[1] = stored[2];
            stored[2] = none;
            if (compMode) {
                strat->nextGoalCallback();
            } else {
                nextState = waitingForData;
            }
        }
        break;
    case pressButton:
        if (t >= timerTarget) {
            if (servoTarget) {
// servo finished extending, bring it back
#ifdef DEBUG
                // Serial.println("Extended");
#endif
                buttonServo.write(PRESS_STORE_ANGLE);
                timerTarget = t + PRESS_TIME;
                servoTarget = false;
                numPressed++;
                nextState = pressButton;
            } else {
// servo finished retracting, extend again
#ifdef DEBUG
                // Serial.println("Retracted");
#endif
                if (numPressed == targetPress) {
                    // button has been pressed enough times
                    numPressed = 0;
                    targetPress = 0;
                    if (compMode) {
                        strat->nextGoalCallback();
                    } else {
                        nextState = waitingForData;
                    }
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
        shovelQtr.read(shovelQtrValues);
        // TODO, implement break sensor threshold
        if (t >= timerTarget || shovelQtrValues[0] <= 500) {
            shouldStop = true;
        }
        if (shouldStop) {
            if (analogRead(hallEffectPin) < 200) {
                // block is a silverfish
                requireAttacking = true;
                // short circuiting strategy because this is always required
                pressButtonService(getSilverfishHits());
            } else {
                // block is not a silverfish
                requireAttacking = false;
                strat->nextGoalCallback();
            }
        }

        break;
    case movingRack:
        if (limitStates[targetRack] == 0) {
// rack is at target
#ifdef DEBUG
            Serial.println("Rack Stopped");
#endif
            rack->setSpeed(0);
            conveyor->setSpeed(0);
            currentRack = targetRack;
            if (compMode) {
                strat->nextGoalCallback();
            } else {
                nextState = waitingForData;
            }
        }
    case waitingForData:
        parseData();
        break;
    case discarding:
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
                if (compMode) {
                    strat->nextGoalCallback();
                } else {
                    nextState = waitingForData;
                }
            }
        }
        break;
    case lineFollowing:
        distToWall = getPrimaryRangeDist();
        if (distToWall <= minDist) {
            shouldStop = true;
        }
        if (shouldStop) {
            if (compMode) {
                stopDriveMotors();
                fullJacobian = motorJacobian;
                strat->nextGoalCallback();
            } else {
                stopDriveMotors();
                fullJacobian = motorJacobian;
                nextState = waitingForData;
            }
        } else {
            followLine(2.4);
        }
        // if (shouldStop) {
        //     stopDriveMotors();
        //     if (getRangeDistance(rangeFront, FRONT) <= minDist + stopDist) {
        //         senseColorService();
        //     } else {
        //         nextState = waitingForData;
        //     }
        //     fullJacobian = motorJacobian; // reset center of rotation
        // } else {
        //     // x_dot = map((float)min(distToWall, minDist + stopDist),
        //     minDist,
        //     // minDist + stopDist, 0.2, 2.1);
        //     followLine(2.4);
        // }
        break;
    case coasting:
        if (shouldStop) {
            stopDriveMotors();
            nextState = waitingForData;
        }
        coastMotors();
        break;
    case trajectory:
        if (getPrimaryRangeDist() < stopDist) {
            // shouldStop = true;
        }
        if (shouldStop) {
            // externalStop();
            // break;
        }
        readEncoders();
        controlMotorsPathed();
        break;
    case moveGate:
        if (timerTarget >= t) {
            if (compMode) {
                strat->nextGoalCallback();
            } else {
                nextState = waitingForData;
            }
        }
    }

#ifdef DEBUG
    if (state != nextState) {
        printState();
    }
#endif
    state = nextState;
}
