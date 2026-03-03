#include "followLine.h"
#include "extern.h"
#include "globals.h"
#include "services.h"

// distance in milimeters
const double lineArrayDist[LINE_COUNT] = {0, 0.8, 1.6, 2.4, 3.2, 4.0, 4.8, 5.6};
const uint16_t LINE_BIAS[LINE_COUNT] = {146, 96, 96, 96, 96, 96, 96, 122};
const double TARGET_LINE = 2.8;
double last_time = 0;

void startLineFollowing() {
  total_line_err = 0; // reset PID integral windup
  // reset line filtering
  for (int i = 0; i < LINE_COUNT; i++) {
    lineValuesFiltered[i] = lineValues[i] - LINE_BIAS[i];
  }
  // pre-cache line following rotation jacobian
  // TODO, find jacobian for turning in front of sensor and add a check here
  mapCenterOfRotation(-1.586, 0, true);
  x_dot = 0;
  delay(10);
}

double lineLocalize() {
  float alpha = 0.5;
  double lineD = 0;
  double totalVal = 0;
  lineQtr.read(lineValues);
  // line values will always be filtered by this point
  for (int i = 0; i < LINE_COUNT; i++) {
    lineValues[i] -= LINE_BIAS[i];
    lineValuesFiltered[i] =
        alpha * lineValues[i] + (1 - alpha) * lineValuesFiltered[i];
    lineD += lineValuesFiltered[i] * lineArrayDist[i];
    totalVal += lineValuesFiltered[i];
  }
  return lineD /= totalVal;
}

void followLine(double feed_rate) {
  feed_rate = 2.1; // normally 2.1, map down if less
  float Kp = 1.3;
  float Ki = 0.002;
  float Kd = 0.025;

  float linePos = lineLocalize();
  // lineD = lineD / totalVal;
  float last_err = line_err;
  line_err = linePos - TARGET_LINE;
  // want to sample total err and derivative error in millis, not seconds
  float d_err = (line_err - last_err) / (t * 1000. - last_time * 1000.);
  total_line_err += line_err / (t * 1000. - last_time * 1000.);

  y_dot = 0;
  // x_dot = constrain(feed_rate - 0.015 * pow(abs(line_err), 1.2),
  // feed_rate / 1.25, feed_rate);
  x_dot = feed_rate;
  theta_dot = -(line_err * Kp + d_err * Kd + total_line_err * Ki);
  controlMotorsClamped();
  last_time = t;
}