#pragma once

#include "extern.h"
void startConveyorService(bool forward);
void moveRackService(uint8_t target);
inline double toSeconds(double microseconds) { return microseconds / 1000000.; }
// void controlMotors(double x_dot, double y_dot, double theta_dot);
void controlMotors();
double getRangeDistance(double range);
void stopDriveMotors();