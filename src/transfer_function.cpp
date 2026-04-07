#include "transfer_function.h"
#include "BasicLinearAlgebra.h"
#include "extern.h"
#include <Arduino.h>

double mCommand[NUM_MOTORS] = {0, 0, 0};

double GearRatio = 50.0;
int countsPerRev = 64; // encoder counts per Rev

double alpha = 0.05; // digital filter
BLA::Matrix<NUM_MOTORS, 1, float> omegaTF = {0, 0, 0};
BLA::Matrix<3, 1, float> speed_out = {0, 0, 0};
BLA::Matrix<NUM_MOTORS, 3> invJac = BLA::Inverse(motorJacobian);
float stepTime = 3.0;

double thetaDes[NUM_MOTORS] = {0, 0, 0};
double thetaDesUnfiltered[NUM_MOTORS] = {0, 0, 0};
double thetaFinal[NUM_MOTORS] = {0, 0, 0};
double omegaDes[NUM_MOTORS] = {0, 0, 0};
double Kp, Kd, Ki;

double error_tf[NUM_MOTORS] = {0, 0, 0};
double dError_tf[NUM_MOTORS] = {0, 0, 0};
double intError_tf[NUM_MOTORS] = {0, 0, 0};

double stepSize = 0.0;
double rampSpeed = 0.0;

void loop_cltf(double step1, double step2, double step3) {
    // set up
    Serial.begin(230400); // need to use high baud rate when printing during
                          // feedback control
    Serial.println("Testing Testing");
    String inString = "";
    while (Serial.available() == 0)
        ;
    inString = Serial.readStringUntil(' ');
    Kp = inString.toFloat();
    inString = Serial.readStringUntil(' ');
    Kd = inString.toFloat();
    inString = Serial.readStringUntil(' ');
    Ki = inString.toFloat();
    inString = Serial.readStringUntil(' ');
    stepSize = inString.toFloat();
    inString = Serial.readStringUntil(' ');
    alpha = inString.toFloat();
    inString = Serial.readStringUntil(' ');
    stepTime = inString.toFloat(); // your desired velocity for 4.5.5 is
                                   // StepSize/StepDuration

    t0 = micros() / 1000000.; // do this once before starting feedback control
    t_old = 0;                // do this once before starting feedback control

    thetaFinal[0] = 0.0;
    thetaFinal[1] = stepSize;
    thetaFinal[2] = stepSize;
    rampSpeed = stepSize / (stepTime - 1.0);
    bool shouldStep = false;

    while (true) {
        t = micros() / 1000000. - t0;
        dt = t - t_old; // sample time
        for (int i = 0; i < NUM_MOTORS; i++) {
            thetaOld[i] = theta[i];
            encoderCounts[i] = encoders[i]->read();
            // calculate your position and velocity here
            theta[i] = 2 * PI * encoderCounts[i] / (countsPerRev * GearRatio);
            omegaTF(i) = ((theta[i] - thetaOld[i]) / dt) * alpha +
                         omegaTF(i) * (1 - alpha);
            if (shouldStep) {
                thetaDes[i] = thetaFinal[i];
                omegaDes[i] = 0.0;
            } else {
                thetaDesUnfiltered[i] += rampSpeed * dt;
                thetaDesUnfiltered[i] =
                    constrain(thetaDesUnfiltered[i], -abs(thetaFinal[i]),
                              abs(thetaFinal[i]));
                thetaDes[i] =
                    thetaDes[i] * 0.95 + thetaDesUnfiltered[i] * (1 - 0.95);
                omegaDes[i] = stepSize / stepTime;
            }

            // error
            error_tf[i] = thetaDes[i] - theta[i];
            dError_tf[i] = omegaDes[i] - omegaTF(i);
            intError_tf[i] += error_tf[i] * dt;
            intError_tf[i] = constrain(intError_tf[i], -10.0 / Ki, 10.0 / Ki);

            // mCommand in volts
            mCommand[i] =
                Kp * error_tf[i] + Ki * intError_tf[i] + Kd * dError_tf[i];
            drive_motors[i]->setSpeed(400 * mCommand[i] / 10.0);
            // drive_motors[i]->setSpeed(400);
        }

        t_old = t;
        if (t < stepTime) {
            Serial.print(t, 5);
            Serial.print('\t');
            Serial.print(theta[1], 5);
            Serial.print('\t');
            Serial.print(dError_tf[1], 5);
            Serial.print('\t');
            Serial.print(intError_tf[1], 5);
            Serial.print('\t');
            Serial.print(mCommand[1], 5);
            Serial.print('\t');
            Serial.print(thetaDes[1], 5);
            Serial.println();
        }
    }
}

void loop_oltf(int step1, int step2, int step3) {
    while (true) {
        t = micros() / 1000000. - t0;
        dt = t - t_old; // sample time

        for (int i = 0; i < NUM_MOTORS; i++) {
            thetaOld[i] = theta[i];
            encoderCounts[i] = encoders[i]->read();
            // calculate your position and velocity here
            theta[i] = 2 * PI * encoderCounts[i] / (countsPerRev * GearRatio);
            omegaTF(i) = ((theta[i] - thetaOld[i]) / dt) * alpha +
                         omegaTF(i) * (1 - alpha);
        }

        // Put step command here
        if (t >= 0.0 && t <= stepTime) {
            mCommand[0] = step1;
            mCommand[1] = step2;
            mCommand[2] = step3;
        } else {
            mCommand[0] = 0;
            mCommand[1] = 0;
            mCommand[2] = 0;
        }

        for (int i = 0; i < NUM_MOTORS; i++) {
            drive_motors[i]->setSpeed(mCommand[i]);
        }
        // Put print commands here, make sure to use enough decimal places for
        // your time and velocity
        if (t < 2 * stepTime) {
            Serial.print(t, 4);
            Serial.print('\t');
            Serial.print(omegaTF(1), 4);
            Serial.print('\t');
            Serial.print(omegaTF(2), 4);
            Serial.print('\t');
            Serial.print(omegaTF(3), 4);
            Serial.print('\t');
            Serial.print(mCommand[0]);
            Serial.print('\t');
            Serial.print(mCommand[1]);
            Serial.print('\t');
            Serial.print(mCommand[2]);
            Serial.print('\n');
        }

        t_old = t;
    }
}
