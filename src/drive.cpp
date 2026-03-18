#include "drive.h"
#include "extern.h"
#include "sending.h"
#include "services.h"

const int MAX_SEGMENTS = 8;
int segment = 0;
int totalSegments = 0;
const float encoderAlpha = 0.05;
const float gearRatio = 50.0;
const float countsPerRev = 64.0;
extern float omega[NUM_MOTORS];
float theta_des[NUM_MOTORS] = {0};
float theta_targets[MAX_SEGMENTS][NUM_MOTORS];
float omega_targets[MAX_SEGMENTS][NUM_MOTORS];
void (*callback[MAX_SEGMENTS])();

void instantiateTargets() {
  for (int i = 0; i < MAX_SEGMENTS; i++) {
    for (int j = 0; j < NUM_MOTORS; j++) {
      theta_targets[i][j] = 0;
      omega_targets[i][j] = 0;
    }
  }
  segment = 0;
}

void readEncoders() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    thetaOld[i] = theta[i];
    encoderCounts[i] = encoders[i]->read();
    theta[i] = TWO_PI * encoderCounts[i] / gearRatio / countsPerRev;
    omega[i] = encoderAlpha * omega[i] +
               (1 - encoderAlpha) * (theta[i] - thetaOld[i]) / dt;
  }
}

void resetEncoders() {
  memset(&encoderCounts, 0, sizeof(encoderCounts));
  for (auto enc : encoders) {
    enc->write(0);
  }
  memset(&theta_des, 0, sizeof(theta_des));
  t_old = t;
}

float trajectoryKp = 1.50;
void controlMotorsPathed() {
  float V[NUM_MOTORS];
  int withinTarget = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    // ramp target
    if (abs(theta_des[i] - theta_targets[segment][i]) > 0.05) {
      theta_des[i] += omega_targets[segment][i] * dt;
    }
    if (abs(theta[i] - theta_targets[segment][i]) < 0.50) {
      withinTarget++;
      drive_motors[i]->setSpeed(0);
    } else {
      // control law
      V[i] = trajectoryKp * (theta_des[i] - theta[i]);
      // drive motor
      V[i] = constrain(V[i], -10, 10);
      drive_motors[i]->setSpeed(400.0 * V[i] / 10.0);
    }
  }
  if (withinTarget >= NUM_MOTORS) {
    Serial.println("Hit Target");
    // callback[segment](); // run the callback for finishing segment
    segment++;
    if (segment == totalSegments) {
      stopDriveMotors();
      nextState = waitingForData;
    }
  }
}

void addToTargets(BLA::Matrix<NUM_MOTORS, 1, float> w, double time, int idx) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    omega_targets[idx][i] = w(i);
    // theta_targets[idx][i] =
    //     i > 0 ? w(i) * time + theta_targets[idx - 1][i] : w(i) * time;
    theta_targets[idx][i] = w(i) * time;
  }
  callback[idx] = *resetEncoders;
  Serial.println(idx);
  if ((idx + 1) >= totalSegments) {
    totalSegments = idx + 1;
  }
}

void generateStraightPath(double angle, double distance, double time, int idx) {
  // angle is CCW from forwards
  float x_speed = cos(angle) * distance / time;
  Serial.print("X speed: ");
  Serial.println(x_speed);
  float y_speed = -sin(angle) * distance / time;
  BLA::Matrix<NUM_MOTORS, 1, float> globalVel = {x_speed, y_speed, 0};
  auto w = motorJacobian * globalVel;
  addToTargets(w, time, idx);
}

void generatePointTurn(double angle, double time, int idx) {
  // angle is CCW from forwards
  BLA::Matrix<NUM_MOTORS, 1, float> globalVel = {0, 0, (float)(angle / time)};
  auto w = motorJacobian * globalVel;
  addToTargets(w, time, idx);
}

void generateSwingTurn(double x_center, double y_center, double angle,
                       double time, int idx) {
  // angle is CCW from forwards
  BLA::Matrix<NUM_MOTORS, 1, float> globalVel = {0, 0, (float)(angle / time)};
  auto w = motorJacobian * mapCenterOfRotation(x_center, y_center) * globalVel;
  addToTargets(w, time, idx);
}

void startStraightService(double angle, double dist, double time) {
  segment = 0;
  totalSegments = 1;
  resetEncoders();
  generateStraightPath(angle, dist, time, 0);
  t_old = t;
  memset(&theta_des, 0, sizeof(theta_des));
  nextState = trajectory;
  Serial.print("Target 1: ");
  Serial.print(theta_targets[0][0]);
  Serial.print("\tTarget 2: ");
  Serial.print(theta_targets[0][1]);
  Serial.print("\tTarget 3: ");
  Serial.println(theta_targets[0][2]);
}

void startTurnService(double angle, double time) {
  segment = 0;
  totalSegments = 1;
  resetEncoders();
  generatePointTurn(angle, time, 0);
  t_old = t;
  memset(&theta_des, 0, sizeof(theta_des));
  nextState = trajectory;
  Serial.print("Target 1: ");
  Serial.print(theta_targets[0][0]);
  Serial.print("\tTarget 2: ");
  Serial.print(theta_targets[0][1]);
  Serial.print("\tTarget 3: ");
  Serial.println(theta_targets[0][2]);
}

void startHookService(double angle, double time, double radius) {
  segment = 0;
  totalSegments = 1;
  resetEncoders();
  radius = radius > 0 ? radius + 0.1289 : radius - 0.1289;
  generateSwingTurn(0, radius, angle, time, 0);
  t_old = t;
  memset(&theta_des, 0, sizeof(theta_des));
  nextState = trajectory;
}

void beginPathing() {
  segment = 0;
  totalSegments = 0;
  resetEncoders();
}
