#include "followLine.h"
#include "extern.h"
#include "services.h"

// distance in milimeters
const double lineArrayDist[LINE_COUNT] = {0, 0.8, 1.6, 2.4, 3.2, 4.0, 4.8, 5.6};
const uint16_t LINE_BIAS[LINE_COUNT] = {146, 96, 96, 96, 96, 96, 96, 122};
const double TARGET_LINE = 2.8; // TODO, calibrate this

void followLine(double feed_rate, double Kp) {
  double lineD = 0;
  double totalVal = 0;
  lineQtr.read(lineValues);
  for (int i = 0; i < LINE_COUNT; i++) {
    lineValues[i] -= LINE_BIAS[i];
    lineD += lineValues[i] * lineArrayDist[i];
    totalVal += lineValues[i];
  }
  lineD /= totalVal;
  // lineD = lineD / totalVal;
  double err = lineD - TARGET_LINE;

  y_dot = 0;
  x_dot = constrain(feed_rate - 0.2 * pow(abs(err), 2), 0, 999);
  theta_dot = -(err * Kp);
  controlMotors();
}