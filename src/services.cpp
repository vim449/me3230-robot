#include "services.h"
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

void controlMotors() {
  BLA::Matrix<3, 1, double> in = {x_dot, y_dot, theta_dot};
  BLA::Matrix<3, 1, double> motorSpeeds = motorJacobian * in;

  for (int i = 0; i < 3; i++) {
    drive_motors[i]->setSpeed(motorSpeeds(i));
  }
}

double getRangeDistance() {
  int rangeReading = analogRead(A0);
  // TODO, recallibrate
  float distance = exp(-0.8585 * log10(rangeReading) + 1.424);
  return distance;
}

void stopDriveMotors() {
  for (auto &motor : drive_motors) {
    motor->setBrake(400);
  }
  x_dot = 0;
  y_dot = 0;
  theta_dot = 0;
}