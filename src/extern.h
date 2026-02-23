// this file uses 'extern' to give all other files access
// to the global variables defined in main.cpp

#pragma once
#include "BasicLinearAlgebra.h"
#include "Encoder.h"
#include "L298N.h"
#include "PWMServo.h"
#include "QTRSensors.h"
#include "sending.h"

// DEFINITIONS
#define DISCARD_ANGLE 180
#define DISCARD_STORE_ANGLE 0
#define PRESS_ANGLE 180
#define PRESS_STORE_ANGLE 0
#define xbee Serial3

// turn off when not connected to pc for performance
// #define DEBUG
#ifdef DEBUG
bool shouldPrint = true;
#endif

// PINS, not sure this is needed tbh

// motors
extern L298N *drive_motors[3];
extern L298N *conveyor;
extern L298N *rack;
extern PWMServo buttonServo, discardServo;
extern const BLA::Matrix<3, 3, double> motorJacobian;

// sensors
extern QTRSensors lineQtr;
#define LINE_COUNT 8
extern uint16_t lineValues[LINE_COUNT];
extern QTRSensors shovelQtr;
extern QTRSensors conveyorQtr;
extern Encoder *encoders[3];

// control targets
extern double t, t0, timerTarget;
extern State state, nextState;
extern double x_dot, y_dot, theta_dot;
extern uint8_t targetRack, currentRack;
extern uint8_t targetPress;
extern bool servoTarget;
extern double lineD;
