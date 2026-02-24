#pragma once

#include "extern.h"
void startConveyorService(bool forward);
void moveRackService(uint8_t target);
inline double toSeconds(double microseconds) { return microseconds / 100000.; }
void controlMotors();
double getRangeDistance();
void stopDriveMotors();