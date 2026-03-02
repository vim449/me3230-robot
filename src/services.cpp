#include "services.h"
#include "extern.h"
// this file contains all services small enough to not warrant their own file

void startConveyorService(bool forwards) {
  conveyor->setSpeed(forwards ? 400 : -400);
}

void startLineFollowing() {
  total_line_err = 0; // reset PID integral windup
  lineFilterInitialized =
      false; // line values will jump discontinuously so reset filter
  mapCenterOfRotation(-1.586, 0, true);
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

BLA::Matrix<3, 3, float> mapCenterOfRotation(float x, float y) {
  // TODO, find a way to cache total transmission jacobian
  BLA::Matrix<3, 3> J;
  J(0, 0) = 1;
  J(0, 1) = 0;
  J(0, 2) = -y;
  J(1, 0) = 0;
  J(1, 1) = 1;
  J(1, 2) = x;
  J(2, 0) = 0;
  J(2, 1) = 0;
  J(2, 2) = 1;
  return J;
}

void mapCenterOfRotation(float x, float y, bool overload) {
  fullJacobian = motorJacobian * mapCenterOfRotation(x, y);
}

void controlMotors() {
  BLA::Matrix<3, 1, float> in = {x_dot, y_dot, theta_dot};
  BLA::Matrix<3, 1, float> motorSpeeds = motorJacobian * in;

  for (int i = 0; i < 3; i++) {
    drive_motors[i]->setSpeed(motorSpeeds(i));
  }
}

void controlMotors(float x, float y) {
  BLA::Matrix<3, 1, float> in = {x_dot, y_dot, theta_dot};
  BLA::Matrix<3, 1, float> motorSpeeds =
      motorJacobian * mapCenterOfRotation(x, y) * in;

  for (int i = 0; i < 3; i++) {
    drive_motors[i]->setSpeed(motorSpeeds(i));
  }
}

void controlMotorsClamped(float x, float y) {
  // Sets a center of rotation at x,y from primary coords, then scales all
  // coordinates down by the amount that the fastest motor is saturated
  BLA::Matrix<3, 1, float> in = {x_dot, y_dot, theta_dot};
  BLA::Matrix<3, 1, float> motorSpeeds =
      motorJacobian * mapCenterOfRotation(x, y) * in;
  double max = 400.0;
  for (int i = 0; i < 3; i++) {
    max = abs(motorSpeeds(i)) > max ? motorSpeeds(i) : max;
  }
  for (int i = 0; i < 3; i++) {
    drive_motors[i]->setSpeed(motorSpeeds(i) / max * 400.0);
  }
}

void controlMotorsClamped() {
  // Assumes that full rotation jacobian has been pre-cached and just scaless
  // motor coordinates down by the fastest motor's saturation
  BLA::Matrix<3, 1> in = {x_dot, y_dot, theta_dot};
  BLA::Matrix<3, 1> motorSpeeds = fullJacobian * in;
  double max = 400.0;
  for (int i = 0; i < 3; i++) {
    max = abs(motorSpeeds(i)) > max ? motorSpeeds(i) : max;
  }
  for (int i = 0; i < 3; i++) {
    drive_motors[i]->setSpeed(motorSpeeds(i) / max * 400.0);
  }
}

double getRangeDistance(double range, bool side) {
  if (side == FRONT) {
    // Front range finder
    return exp(-0.9912 * log(range) + 7.813);
  } else {
    // Back Range finder
    return exp(-0.9912 * log(range) + 7.813);
  }
}

void stopDriveMotors() {
  for (auto &motor : drive_motors) {
    motor->setBrake(400);
  }
  x_dot = 0;
  y_dot = 0;
  theta_dot = 0;
}

void coastMotors() {
  for (auto &motor : drive_motors) {
    motor->setBrake(0);
  }
  x_dot = 0;
  y_dot = 0;
  theta_dot = 0;
}

inline float readPulse() {
  return pulseIn(colorReadPin, LOW) + pulseIn(colorReadPin, HIGH);
}

inline float movingAverage(float arr[], uint8_t arr_size) {
  float sum = 0;
  for (int i = 0; i < arr_size; i++) {
    sum += arr[i] / arr_size;
  }
  return sum;
}

ColorSensing getColorData() {
  // returns a ColorSensing struct containing the pulseTime measurements of the
  // color sensor

  // TODO, talk to dr mascaro about delay() usage, see if switching to a timer
  // check is needed
  const uint8_t numSamples = 8;
  float R[numSamples], G[numSamples], B[numSamples], C[numSamples];
  // get Red data
  digitalWrite(colorS2, LOW);
  digitalWrite(colorS3, LOW);
  delay(10);
  for (int i = 0; i < numSamples; i++) {
    R[i] = readPulse();
  }

  // get Green data
  digitalWrite(colorS2, LOW);
  digitalWrite(colorS3, HIGH);
  delay(10);
  for (int i = 0; i < numSamples; i++) {
    G[i] = readPulse();
  }

  // get Blue data
  digitalWrite(colorS2, HIGH);
  digitalWrite(colorS3, HIGH);
  delay(10);
  for (int i = 0; i < numSamples; i++) {
    B[i] = readPulse();
  }

  // get Clear data
  digitalWrite(colorS2, HIGH);
  digitalWrite(colorS3, LOW);
  delay(10);
  for (int i = 0; i < numSamples; i++) {
    C[i] = readPulse();
  }

  // TODO, convert to frequency
  ColorSensing data = {
      movingAverage(R, numSamples), movingAverage(G, numSamples),
      movingAverage(B, numSamples), movingAverage(C, numSamples)};

  return data;
};