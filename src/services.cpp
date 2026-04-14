#include "services.h"
#include "extern.h"
#include "globals.h"
#include "sending.h"
// this file contains all services small enough to not warrant their own file

void startConveyorService(bool forwards) {
    timerTarget = t + 1.0;
    conveyor->setSpeed(forwards ? 300 : -300);
    nextState = dispensing;
}

void moveRackService(uint8_t target) {
    targetRack = target;
    if (targetRack > currentRack) {
        Serial.println("Driving Rack Backwards");
        rack->setSpeed(-200);
    } else if (targetRack < currentRack) {
        Serial.println("Driving Rack Forwards");
        rack->setSpeed(200);
    } else {
        // somehow this function was called with the rack in the target pos
        // Serial.println("Something went wrong");
        Serial.println("Null state triggered");
        rack->setSpeed(0);
    }
    nextState = movingRack;
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
    // note, this does not use the mapped center of rotation, maybe i should,
    // idk
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
    // Assumes that full rotation jacobian has been pre-cached and just scales
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
        return -0.0000008529 * pow(range, 3) + 0.0008084 * pow(range, 2) +
               -0.2788 * range + 36.91;
    } else {
        // Back Range finder
        return pow(range / 1105.44, -1.343);
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
    // returns a ColorSensing struct containing the pulseTime measurements of
    // the color sensor
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
    digitalWrite(colorS2, HIGH);
    digitalWrite(colorS3, HIGH);
    delay(10);
    for (int i = 0; i < numSamples; i++) {
        G[i] = readPulse();
    }

    // get Blue data
    digitalWrite(colorS2, LOW);
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

int getBlockHits(BlockType block) {
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

void printBlock(BlockType block) {
    switch (block) {
    case wood:
        Serial.print("Wood");
        break;
    case stone:
        Serial.print("Stone");
        break;
    case iron:
        Serial.print("Iron");
        break;
    case diamond:
        Serial.print("Diamond");
        break;
    case none:
        break;
    }
}

double getPrimaryRangeDist() {
    switch (headingLocation) {
    case start:
    case leftTree:
    case leftMine:
    case rightMine:
    case rightTree:
        return getRangeDistance(rangeFront, FRONT);
    case chest:
    case craft:
        return getRangeDistance(rangeBack, BACK);
        break;
    }
}

void pressButtonService(int goalPress) {
    // sets all variables needed to begin pressing the mine button goalpress
    // times
    numPressed = 0;
    targetPress = goalPress;
    timerTarget = t + PRESS_TIME;
    buttonServo.write(PRESS_ANGLE);
    servoTarget = true;
    nextState = pressButton;
}

void senseColorService() {
    // Color sensor only used on middle mines, which give stone iron diamond
    // only
    ColorSensing data = getColorData(); // NOTE, will cause a 30ms delay
    float red = 100 * data.Clear / data.Red;
    float blue = 100 * data.Clear / data.Blue;
    float green = 100 * data.Clear / data.Green;
    // Serial.println(red);
    // Serial.println(green);
    // Serial.println(blue);

    BlockType block = stone;
    if (red >= 45 && blue <= 30 && green <= 30) {
        block = iron;
    } else if (blue >= 30 && red <= 40 && green <= 35) {
        block = diamond;
    }
    int hits = getBlockHits(block);
    inShovel = hits > 0 ? block : none;
    hits = abs(hits);

    pressButtonService(hits);
    xbee.write((byte)block);
#ifdef DEBUG
    Serial.print("Sensed block ");
    printBlock(block);
    Serial.println();
#endif
}

float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void NOP() {}

void printState() {
    Serial.print("Entered State ");
    switch (nextState) {
    case drivingTimeBased:
        Serial.print("Driving");
        break;
    case storingBlock:
        Serial.print("Storing");
        break;
    case dispensing:
        Serial.print("Dispensing");
        break;
    case discarding:
        Serial.print("Discarding");
        break;
    case pressButton:
        Serial.print("Pressing Button");
        break;
    case waitingForBlock:
        Serial.print("Waiting For Block");
        break;
    case movingRack:
        Serial.print("Moving Rack");
        break;
    case waitingForData:
        Serial.print("Waiting For Data");
        break;
    case lineFollowing:
        Serial.print("Line Following");
        break;
    case coasting:
        Serial.print("Coasting Motors");
        break;
    case moveGate:
        Serial.print("Moving Gate");
        break;
    case trajectory:
        Serial.print("Trajectory");
        break;
    }
    Serial.println();
}

void storeBlockService() {
    timerTarget = t + STORE_TIME;
    conveyor->setSpeed(200);
    nextState = storingBlock;
    numStored = numStored + 1;
}

void discardBlockService() {
    discardServo.write(DISCARD_ANGLE);
    timerTarget = t + DISCARD_TIME;
    servoTarget = true;
    nextState = discarding;
}

void gateService(bool shouldOpen) {
    gateServo.write(shouldOpen ? GATE_OPEN_ANGLE : GATE_CLOSE_ANGLE);
    timerTarget = t + GATE_TIME;
    nextState = moveGate;
}

void startGame() {
    currentLocation = start;
    compMode = true;
    stored[0] = none;
    stored[1] = none;
    stored[2] = none;
    state = waitingForData;
    nextState = waitingForData;
    strat->nextGoalCallback();

    gateServo.write(GATE_CLOSE_ANGLE);
    t0 = micros() / 1000000.;
    t = micros() / 1000000. - t0;
}
