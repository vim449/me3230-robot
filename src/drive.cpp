#include "drive.h"
#include "globals.h"
#include "services.h"

const int MAX_SEGMENTS = 4;
int segment = 0;
int totalSegments = MAX_SEGMENTS;
const float encoderAlpha = 0.25;
const float gearRatio = 50.0;
const float countsPerRev = 64.0;
float omega[NUM_MOTORS] = {0};
float theta_des[NUM_MOTORS] = {0};
float theta_targets[MAX_SEGMENTS][NUM_MOTORS];
float omega_targets[MAX_SEGMENTS][NUM_MOTORS];
void (*callback[MAX_SEGMENTS])();

void readEncoders() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    thetaOld[i] = theta[i];
    encoderCounts[i] = encoders[i]->read();
    // TODO confirm this maps to theta correctly
    theta[i] = TWO_PI * encoderCounts[i] / gearRatio / countsPerRev;
    // IIR filtering
    // TODO tune alpha
    omega[i] = encoderAlpha * omega[i] +
               (1 - encoderAlpha) * (theta[i] - thetaOld[i]) / dt;
  }
}

float Kp = 0.5;
void controlMotorsPathed() {
  float V[NUM_MOTORS];
  int withinTarget = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    // ramp target
    theta_des[i] += omega_targets[segment][i] * dt;
    if (abs(theta_des[i] - theta_targets[segment][i] < 0.01)) {
      withinTarget++;
    }
    // control law
    V[i] = Kp * (theta_des[i] - theta[i]);

    // drive motor
    V[i] = constrain(V[i], -10, 10);
    drive_motors[i]->setSpeed(40.0 * V[i]);
  }
  if (withinTarget == NUM_MOTORS) {
    segment++;
    if (segment == totalSegments) {
      stopDriveMotors();
      nextState = waitingForData;
    }
    callback[segment - 1](); // run the callback for finishing that segment
  }
}

void addToTargets(BLA::Matrix<NUM_MOTORS, 1, float> w, double time, int idx) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    omega_targets[idx][i] = w(i);
    theta_targets[idx][i] = w(i) * time;
    callback[idx] = *NOP;
    // by default, assign a no-op callback that can be overwritten later
  }
}

void generateStraightPath(double angle, double distance, double time, int idx) {
  // angle is CCW from forwards
  float x_speed = cos(angle) * distance / time;
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
