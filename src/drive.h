#pragma once

void readEncoders();
void controlMotorsPathed();
void generateStraightPath(double angle, double distance, double time, int idx);
void generatePointTurn(double angle, double time, int idx);
void generateSwingTurn(double x_center, double y_center, double angle,
                       double time, int idx);
