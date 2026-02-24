#include "extern.h"
#include "services.cpp"

// distance in milimeters
const double lineArrayDist[LINE_COUNT] = {0, 8, 16, 24, 32, 40, 48, 56};
const uint16_t LINE_BIAS[LINE_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0};
const double TARGET_LINE = 28.0; // TODO, calibrate this

void followLine(double feed_rate, double Kp) {
  double totalVal = 0;
  lineQtr.read(lineValues);
  for (int i = 0; i < LINE_COUNT; i++) {
    lineValues[i] -= LINE_BIAS[i];
    lineD += lineValues[i] * lineArrayDist[i];
    totalVal += lineValues[i];
  }
  lineD /= totalVal;
  double err = lineD - TARGET_LINE;

  y_dot = 0;
  x_dot = feed_rate;
  theta_dot = err * Kp;
  controlMotors();
}