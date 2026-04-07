#pragma once

void instantiateTargets();
void readEncoders();
void resetEncoders();
void controlMotorsPathed();
void generateStraightPath(double angle, double distance, double time, int idx);
void generatePointTurn(double angle, double time, int idx);
void generateSwingTurn(double x_center, double y_center, double angle,
                       double time, int idx);
void externalStop();
void startStraightService(double angle, double dist, double time);
void startTurnService(double angle, double time);
void startHookService(double angle, double time, double radius);
void beginPathing();
void setCallback(int idx, void (*ptr)());
void setCallback(void (*ptr)());
