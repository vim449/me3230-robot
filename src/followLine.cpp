#include "followLine.h"
#include "extern.h"
#include "services.h"

// distance in milimeters
const double lineArrayDist[LINE_COUNT] = {0, 0.8, 1.6, 2.4, 3.2, 4.0, 4.8, 5.6};
const uint16_t LINE_BIAS[LINE_COUNT] = {146, 96, 96, 96, 96, 96, 96, 122};
const double TARGET_LINE = 2.8;
double last_time = 0;

double lineLocalize() {
  float alpha = 0.5;
  double lineD = 0;
  double totalVal = 0;
  lineQtr.read(lineValues);
  if (!lineFilterInitialized) {
    lineFilterInitialized = true;
    for (int i = 0; i < LINE_COUNT; i++) {
      lineValuesFiltered[i] = lineValues[i];
    }
  }
  for (int i = 0; i < LINE_COUNT; i++) {
    lineValues[i] -= LINE_BIAS[i];
    lineValuesFiltered[i] =
        alpha * lineValues[i] + (1 - alpha) * lineValuesFiltered[i];
    lineD += lineValuesFiltered[i] * lineArrayDist[i];
    totalVal += lineValuesFiltered[i];
  }
  return lineD /= totalVal;
}

void followLine(double feed_rate, double Kp, double Kd, double Ki) {
  feed_rate = 2.1;
  // TODO, map feed_rate based on wall distance
  Kp = 1.6;
  Ki = 0.010;
  Kd = 0.030;

  double linePos = lineLocalize();
  // lineD = lineD / totalVal;
  double last_err = line_err;
  line_err = linePos - TARGET_LINE;
  // want to sample total err and derivative error in millis, not seconds
  double d_err = (line_err - last_err) / (t * 1000. - last_time * 1000.);
  total_line_err += line_err / (t * 1000. - last_time * 1000.);

  y_dot = 0;
  // x_dot = constrain(feed_rate - 0.015 * pow(abs(line_err), 1.2),
  // feed_rate / 1.25, feed_rate);
  x_dot = feed_rate;
  theta_dot = -(line_err * Kp + d_err * Kd + total_line_err * Ki);
  controlMotorsClamped();
  last_time = t;
}