#pragma once
// initialize all variables, having it just sitting in main was ugly as hell

#include <Arduino.h>
#include <Encoder.h>
#include <HardwareSerial.h>
#include <L298N.h>
#include <PWMServo.h>
#include <QTRSensors.h>

#include "globals.h"
#include "sending.h"
#include "strategy.h"

bool compMode = false;

// TODO move these out of being externs
double line_err = 0;
double total_line_err = 0;

// motors
bool motors_exist = false; // to ensure motor objects are only created once
L298N *drive_motors[NUM_MOTORS];
L298N *conveyor;
L298N *rack;
PWMServo buttonServo, discardServo, gateServo;
// maps from x' y' theta' space to motor space, can be found from getJacobian.m
BLA::Matrix<NUM_MOTORS, NUM_MOTORS> fullJacobian = motorJacobian;

// sensors
QTRSensors lineQtr;
QTRSensors shovelQtr;
QTRSensors conveyorQtr;
uint16_t lineValues[LINE_COUNT];
uint16_t lineValuesFiltered[LINE_COUNT];
uint16_t shovelQtrValues[1];
uint16_t conveyorQtrValues[1];
bool limitStates[3] = {
    true, true}; // switches are high by default and low when triggered
Encoder *encoders[NUM_MOTORS];
double hallEffect = 0.0;

// time variables
double t, t_old, t0, dt;

// control variables
State state, nextState;
float x_dot, y_dot, theta_dot;
uint8_t targetRack = 0;
uint8_t currentRack = 0; // index of limit switch the rack is resting at
double timerTarget = 0;

bool servoTarget = false; // true if the servo should be extended
bool gatePos = false;
uint8_t targetPress = 0;
uint8_t numPressed = 0; // number of times button pressed

int32_t encoderCounts[NUM_MOTORS] = {0};
float theta[NUM_MOTORS] = {0};
float thetaOld[NUM_MOTORS] = {0};
float omega[NUM_MOTORS] = {0};

double rangeBack = 0;
double rangeFront = 0;
const double rangeAlpha = 0.025;
double distToWall = 0;

// game variables
BlockType stored[3] = {none};
BlockType inShovel = none;
BlockType pick = wood;
BlockType sword = wood;
PosType headingLocation = start;
PosType currentLocation = start;
bool shield = false;
bool needsDiscard = false;
Strategy *strat = new defaultStrategy();
bool shouldReset = false;
bool requireAttacking = false;
