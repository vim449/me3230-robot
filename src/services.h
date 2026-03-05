#pragma once
#include "globals.h"

void startConveyorService(bool forward);
void moveRackService(uint8_t target);
BLA::Matrix<3, 3, float> mapCenterOfRotation(float x, float y);
void mapCenterOfRotation(float x, float y, bool overload);
inline double toSeconds(double microseconds) { return microseconds / 1000000.; }
// void controlMotors(double x_dot, double y_dot, double theta_dot);
void controlMotors();
void controlMotors(float x, float y);
void controlMotorsClamped();
void controlMotorsClamped(float x, float y);
double getRangeDistance(double range, bool side);
void stopDriveMotors();
void coastMotors();
ColorSensing getColorData();
float rampSpeed(float x);
float map(float x, float in_min, float in_max, float out_min, float out_max);
void senseColorService();

uint8_t getBlockHits(BlockType block);
uint8_t getSilverfishHits();