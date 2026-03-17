#include "services.h"
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

BLA::Matrix<NUM_MOTORS, NUM_MOTORS, float> mapCenterOfRotation(float x,
                                                               float y) {
  BLA::Matrix<NUM_MOTORS, NUM_MOTORS> J;
  J(0, 0) = 1;
  J(0, 1) = 0;
  J(0, 2) = -y;
  J(1, 0) = 0;
  J(1, 1) = 1;
  J(1, 2) = -x;
  J(2, 0) = 0;
  J(2, 1) = 0;
  J(2, 2) = 1;
  return J;
}

void mapCenterOfRotation(float x, float y, bool overload) {
  // stores total mapped jacobian to global variable
  // yes, i know the overload boolean is a hack, shut up
  fullJacobian = motorJacobian * mapCenterOfRotation(x, y);
}

void controlMotors() {
  // note, this does not use the mapped center of rotation, maybe i should, idk
  BLA::Matrix<NUM_MOTORS, 1, float> in = {x_dot, y_dot, theta_dot};
  BLA::Matrix<NUM_MOTORS, 1, float> motorSpeeds = motorJacobian * in;

  for (int i = 0; i < NUM_MOTORS; i++) {
    drive_motors[i]->setSpeed(motorSpeeds(i));
  }
}

void controlMotors(float x, float y) {
  BLA::Matrix<NUM_MOTORS, 1, float> in = {x_dot, y_dot, theta_dot};
  BLA::Matrix<NUM_MOTORS, 1, float> motorSpeeds =
      motorJacobian * mapCenterOfRotation(x, y) * in;

  for (int i = 0; i < NUM_MOTORS; i++) {
    drive_motors[i]->setSpeed(motorSpeeds(i));
  }
}

void controlMotorsClamped(float x, float y) {
  // Sets a center of rotation at x,y from primary coords, then scales all
  // coordinates down by the amount that the fastest motor is saturated
  BLA::Matrix<NUM_MOTORS, 1, float> in = {x_dot, y_dot, theta_dot};
  BLA::Matrix<NUM_MOTORS, 1, float> motorSpeeds =
      motorJacobian * mapCenterOfRotation(x, y) * in;
  double max = 400.0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    max = abs(motorSpeeds(i)) > max ? motorSpeeds(i) : max;
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
    drive_motors[i]->setSpeed(motorSpeeds(i) / max * 400.0);
  }
}

void controlMotorsClamped() {
  // Assumes that full rotation jacobian has been pre-cached and just scaless
  // motor coordinates down by the fastest motor's saturation
  BLA::Matrix<NUM_MOTORS, 1> in = {x_dot, y_dot, theta_dot};
  BLA::Matrix<NUM_MOTORS, 1> motorSpeeds = fullJacobian * in;
  double max = 400.0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    max = abs(motorSpeeds(i)) > max ? motorSpeeds(i) : max;
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
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
  Serial.println("Color Called");
  digitalWrite(colorLED, HIGH); // turn on the LED temporarily

  const uint8_t numSamples = 8;
  float R[numSamples], G[numSamples], B[numSamples], C[numSamples];
  // get Red data
  digitalWrite(colorS2, LOW);
  digitalWrite(colorS3, LOW);
  delay(30);
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

  ColorSensing data = {
      movingAverage(R, numSamples), movingAverage(G, numSamples),
      movingAverage(B, numSamples), movingAverage(C, numSamples)};

  digitalWrite(colorLED, LOW);
  return data;
};

uint8_t getBlockHits(BlockType block) {
  if (block == wood) {
    return pick == wood ? 5 : pick == stone ? 4 : pick == iron ? 2 : 1;
  } else if (block == stone) {
    return pick == wood ? 10 : pick == stone ? 5 : pick == iron ? 3 : 2;
  } else if (block == iron) {
    return pick == wood ? -10 : pick == stone ? 10 : pick == iron ? 5 : 3;
  } else {
    return pick == wood ? -10 : pick == stone ? -10 : pick == iron ? 10 : 5;
  }
}

uint8_t getSilverfishHits() {
  return sword == wood ? 10 : sword == stone ? 7 : sword == iron ? 4 : 1;
}

void senseColorService() {
  // Color sensor only used on middle mines, which give stone iron diamond only

  ColorSensing data = getColorData(); // NOTE, will cause a 30ms delay
  float red = 100 * data.Clear / data.Red;
  float blue = 100 * data.Clear / data.Blue;
  float green = 100 * data.Clear / data.Green;

  BlockType block = stone;
  if (red >= 50 && blue <= 30 && green <= 30) {
    block = iron;
  } else if (blue >= 50 && red <= 30 && green <= 30) {
    block = diamond;
  }
  targetPress = getBlockHits(block);

  // TODO add condition to mine unminable block anyways
  if (false) {
    targetPress *= -1;
  }
  if (targetPress > 0) {
    timerTarget = t + PRESS_TIME;
    buttonServo.write(PRESS_ANGLE);
    servoTarget = true;
    nextState = pressButton;
    xbee.write((byte)block);
#ifdef DEBUG
    switch (block) {
    case wood:
      Serial.println("Wood");
      break;
    case stone:
      Serial.println("Stone");
      break;
    case iron:
      Serial.println("Iron");
      break;
    case diamond:
      Serial.println("Diamond");
      break;
    }
#endif
  } else {
    Serial.println("Can't mine block with current pickaxe");
    nextState = waitingForData;
    targetPress = 0;
  }
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void NOP() {};
