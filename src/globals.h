#pragma once
#include <Arduino.h>

#define DISCARD_ANGLE 90
#define DISCARD_STORE_ANGLE 30
#define PRESS_ANGLE 60
#define PRESS_STORE_ANGLE 30
#define PRESS_TIME 0.25
#define xbee Serial3

// ALL PINS
const uint8_t M1_PWM = 5, M1_C = 42, M1_D = 43;
const uint8_t M2_PWM = 7, M2_C = 40, M2_D = 41;
const uint8_t M3_PWM = 6, M3_C = 38, M3_D = 39;
const uint8_t rack_PWM = 46, rack_C = 47, rack_D = 48;
const uint8_t conveyor_PWM = 45, conveyor_C = 49, conveyor_D = 50;
const uint8_t buttonServo_PWM = 11, discardServo_PWM = 12;
const uint8_t lineQtrPins[8] = {23, 25, 27, 29, 31, 33, 35, 37};
const uint8_t shovelQtrPins[1] = {34};
const uint8_t conveyorQtrPins[1] = {36};
const uint8_t limitPins[2] = {51, 52};
const uint8_t encoderPins[6] = {20, 21, 18, 19, 2, 3};
const uint8_t motorCurrentPins[3] = {A5, A4, A3};
const uint8_t hallEffectPin = A6;
const uint8_t rangeFinderPin = A0;

const long XBEE_BAUD = 115200;
const long USB_BAUD = 57600;

const double LINE_KP = 0.375;
const double LINE_FEED = 0.9;
