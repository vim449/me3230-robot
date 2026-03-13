// this file uses 'extern' to give all other files access
// to the global variables defined in main.cpp
#pragma once

#include "Encoder.h"
#include "L298N.h"
#include "PWMServo.h"
#include "QTRSensors.h"
#include "globals.h"
#include "sending.h"

// turn off when not connected to pc for performance
// #define DEBUG
#ifdef DEBUG
extern bool shouldPrint;
#endif

// PINS, not sure this is needed tbh

// motors
extern L298N *drive_motors[3];
extern L298N *conveyor;
extern L298N *rack;
extern PWMServo buttonServo, discardServo;
extern BLA::Matrix<3, 3> fullJacobian;

// sensors
extern QTRSensors lineQtr;
#define LINE_COUNT 8
extern uint16_t lineValues[LINE_COUNT];
extern uint16_t lineValuesFiltered[LINE_COUNT];
extern QTRSensors shovelQtr;
extern QTRSensors conveyorQtr;

extern Encoder *encoders[NUM_MOTORS];
extern int16_t encoderCounts[NUM_MOTORS];
extern float theta[NUM_MOTORS];
extern float thetaOld[NUM_MOTORS];

extern double rangeFront, rangeBack;

// control targets
extern double t, t0, timerTarget, dt;
extern State state, nextState;
extern float x_dot, y_dot, theta_dot;
extern uint8_t targetRack, currentRack;
extern uint8_t targetPress;
extern bool servoTarget;
extern double line_err, total_line_err;

// game variables
extern BlockType stored[3];
extern BlockType pick;
extern BlockType sword;
extern bool shield;
extern bool needsDiscard;