#pragma once
#include <Arduino.h>
#include <BasicLinearAlgebra.h>

#define DISCARD_ANGLE 180
#define DISCARD_STORE_ANGLE 45
#define DISCARD_TIME 2
#define PRESS_ANGLE 60
#define PRESS_STORE_ANGLE 0
#define PRESS_TIME 0.20
#define GATE_CLOSE_ANGLE 0
#define GATE_OPEN_ANGLE 90
#define GATE_TIME 1.1
#define xbee Serial3
#define BACK false
#define FRONT true
#define STORE_TIME 2.1
#define DISPENSE_TIME 12
#define OPEN true
#define CLOSE false

#define minDist 4.0
#define stopDist 3.0

#define DEBUG

// ALL PINS
// motor pins
#define NUM_MOTORS 3
// const uint8_t encoderPins[6] = {20, 21, 19, 18, 3, 2};
const uint8_t encoderPins[6] = {20, 21, 3, 2, 19, 18};
const uint8_t motorCurrentPins[NUM_MOTORS] = {A5, A4, A3};
const uint8_t M1_PWM = 5, M1_C = 42, M1_D = 43;
const uint8_t M2_PWM = 7, M2_C = 40, M2_D = 41;
const uint8_t M3_PWM = 6, M3_C = 38, M3_D = 39;
// const uint8_t rack_PWM = 45, rack_C = 47, rack_D = 48;
// const uint8_t conveyor_PWM = 44, conveyor_C = 49, conveyor_D = 50;
const uint8_t buttonServo_PWM = 11, discardServo_PWM = 12, gateServo_PWM = 13;

const uint8_t rack_PWM = 45, rack_C = 49, rack_D = 50;
const uint8_t conveyor_PWM = 44, conveyor_C = 47, conveyor_D = 48;

// misc sensor pins
const uint8_t hallEffectPin = A6;
const uint8_t rangeBackPin = A0;
const uint8_t rangeFrontPin = A1;
const uint8_t limitPins[2] = {51, 52};

// reflectance sensors
#define LINE_COUNT 8
const uint8_t lineQtrPins[LINE_COUNT] = {23, 25, 27, 29, 31, 33, 35, 37};
const uint8_t shovelQtrPins[1] = {34};
const uint8_t conveyorQtrPins[1] = {36};

// Color Sensor
const uint8_t colorLED = 22, colorS0 = 26, colorS1 = 28, colorS2 = 30,
              colorS3 = 32, colorReadPin = 24;

const long XBEE_BAUD = 115200;
const long USB_BAUD = 57600;

const double LINE_KP = 0.375;
const double LINE_FEED = 0.9;

const double rw = 47.4 / 1000.0 / 2.0;
const BLA::Matrix<NUM_MOTORS, NUM_MOTORS, float> motorJacobian = {
    0,           1 / rw,      0.1417 / rw,        sqrt(3) / (2 * rw),
    -0.5 / rw,   0.1565 / rw, sqrt(3) / (2 * rw), 0.5 / rw,
    -0.1565 / rw};

struct ColorSensing {
  float Red;
  float Green;
  float Blue;
  float Clear;
};

enum BlockType { none, wood, stone, iron, diamond };
