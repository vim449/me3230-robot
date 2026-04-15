#include "followLine.h"
#include "extern.h"
#include "services.h"

// distance in milimeters
const double lineArrayDist[LINE_COUNT] = {0, 0.8, 1.6, 2.4, 3.2, 4.0, 4.8, 5.6};
const uint16_t LINE_BIAS[LINE_COUNT] = {146, 96, 96, 96, 96, 96, 96, 122};
// const double TARGET_LINE = 2.7;
bool followForwards = true;
const double TARGET_LINE = 2.8;

void startLineFollowing() {
    total_line_err = 0; // reset PID integral windup
    // reset line filtering
    for (int i = 0; i < LINE_COUNT; i++) {
        lineValuesFiltered[i] = lineValues[i] - LINE_BIAS[i];
    }
    // pre-cache line following rotation jacobian
    // TODO, find jacobian for turning in front of sensor and add a check here
    mapCenterOfRotation(-0.01586, 0, true);
    x_dot = 2.6;
    delay(10);
    nextState = lineFollowing;
    followForwards = true;
}

void startLineFollowingBack() {
    total_line_err = 0; // reset PID integral windup
    // reset line filtering
    for (int i = 0; i < LINE_COUNT; i++) {
        lineValuesFiltered[i] = lineValues[i] - LINE_BIAS[i];
    }
    x_dot = 0.0;
    mapCenterOfRotation(0.08, 0, true);
    delay(10);
    nextState = lineFollowing;
    followForwards = false;
}

double lineLocalize() {
    float alpha = 0.5;
    double lineD = 0.0;
    double totalVal = 0.0;
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

float line_print_time = 0;
void followLine(double feed_rate) {
  float Kp, Ki, Kd;
    if (followForwards) {
      Kp = 16.0;
      Ki = 0.02;
      Kd = 0.5;
    } else {
      Kp = 16.0;
      Ki = 0.02;
      Kd = 0.5;
    }

    float linePos = lineLocalize();
    // lineD = lineD / totalVal;
    float last_err = line_err;
    line_err = linePos - TARGET_LINE;
    // want to sample total err and derivative error in millis, not seconds
    float d_err = (line_err - last_err) / dt;
    total_line_err += line_err * dt;
    total_line_err = constrain(total_line_err, -1.0 / abs(Ki), 1.0 / abs(Ki));

    y_dot = 0.0;
    // x_dot = constrain(feed_rate - 0.015 * pow(abs(line_err), 1.2),
    // feed_rate / 1.25, feed_rate);
    x_dot = feed_rate;
    theta_dot = -(line_err * Kp + d_err * Kd + total_line_err * Ki);

    if (t - line_print_time > 0.5) {
        // Serial.print("theta dot: ");
        // Serial.println(theta_dot, 3);
        line_print_time = t;
    }
    controlMotorsClamped();
}
