// this file uses 'extern' to give all other files access
// to the global variables defined in main.cpp
#pragma once

#include "Encoder.h"
#include "L298N.h"
#include "PWMServo.h"
#include "QTRSensors.h"
#include "globals.h"
#include "sending.h"
#include "strategy.h"

extern bool compMode;
// turn off when not connected to pc for performance
#ifdef DEBUG
extern bool shouldPrint;
#endif

// motors
extern L298N *drive_motors[NUM_MOTORS];
extern L298N *conveyor;
extern L298N *rack;
extern PWMServo buttonServo, discardServo, gateServo;
extern BLA::Matrix<NUM_MOTORS, NUM_MOTORS> fullJacobian;

// sensors
extern QTRSensors lineQtr;
#define LINE_COUNT 8
extern uint16_t lineValues[LINE_COUNT];
extern uint16_t lineValuesFiltered[LINE_COUNT];
extern QTRSensors shovelQtr;
extern QTRSensors conveyorQtr;

extern Encoder *encoders[NUM_MOTORS];
extern int32_t encoderCounts[NUM_MOTORS];
extern float theta[NUM_MOTORS];
extern float thetaOld[NUM_MOTORS];

extern double rangeFront, rangeBack;

// control targets
extern double t, t0, t_old, timerTarget, dt;
extern State state, nextState;
extern float x_dot, y_dot, theta_dot;
extern uint8_t targetRack, currentRack;
extern uint8_t targetPress, numPressed;
extern bool servoTarget;
extern double line_err, total_line_err;
extern bool heading; // which direction the robot is driving towards
extern bool gatePos;
extern double targetDist;

// game variables
extern BlockType stored[3];
extern BlockType inShovel;
extern int numStored;
extern BlockType pick;
extern BlockType sword;
extern PosType headingLocation;
extern PosType currentLocation;
extern bool shield;
extern bool needsDiscard;
extern Strategy *strat;
extern bool shouldReset;
extern bool requireAttacking;
