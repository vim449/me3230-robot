#pragma once
#include "extern.h"

// this file contains all services small enough to not warrant their own file

void startConveyorService(bool forwards) {
  conveyor->setSpeed(forwards ? 400 : -400);
}

void moveRackService(uint8_t target) {
  targetRack = target;
  if (targetRack > currentRack) {
    // rack needs to drive forwards
    // Serial.println("Rack driving forwards");
    rack->setSpeed(400); // TODO, determine if this is a reasonable speed
  } else if (targetRack < currentRack) {
    // rack needs to drive backwards
    // Serial.println("Rack driving backwards");
    rack->setSpeed(-400);
  } else {
    // somehow this function was called with the rack in the target pos
    // Serial.println("Something went wrong");
    rack->setBrake(400);
  }
}

inline double toSeconds(double microseconds) { return microseconds / 1000000.; }

void controlMotors() {
  BLA::Matrix<3, 1, double> in = {x_dot, y_dot, theta_dot};
  BLA::Matrix<3, 1, double> motorSpeeds = motorJacobian * in;

  for (int i = 0; i < 3; i++) {
    drive_motors[i]->setSpeed(motorSpeeds(i));
  }
}