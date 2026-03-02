#pragma once
#include "extern.h"

void startConveyorService(bool forward);
void moveRackService(uint8_t target);
void startLineFollowing();
inline double toSeconds(double microseconds) { return microseconds / 1000000.; }
// void controlMotors(double x_dot, double y_dot, double theta_dot);
void controlMotors();
void controlMotors(float x, float y);
void controlMotorsClamped(float x, float y);
double getRangeDistance(double range, bool side);
void stopDriveMotors();
void coastMotors();
ColorSensing getColorData();