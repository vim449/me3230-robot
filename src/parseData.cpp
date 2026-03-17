#include "parseData.h"
#include "drive.h"
#include "extern.h"
#include "followLine.h"
#include "services.h"

void parseData(void) {
    if (xbee.available() >= 3) {
        if (xbee.read() == START_MESSAGE) {
#ifdef DEBUG
            Serial.println("XBEE triggered");
#endif
            nextState = (State)xbee.read();
            char param = xbee.read();

            if (nextState == driving) {
                switch (param) {
                case 'f':
                    startStraightService(0, 0.5, 2.5);
                    break;
                case 'b':
                    x_dot = -1;
                    y_dot = 0;
                    theta_dot = 0;
                    break;
                case 'l':
                    startHookService(2 * PI, 5, -0.10);
                    break;
                case 'r':
                    startHookService(-2 * PI, 5, 0.10);
                    break;
                case 'L':
                    startTurnService(PI, 2.5);
                    break;
                case 'R':
                    startTurnService(-PI, 2.5);
                    break;
                default:
                    x_dot = 0;
                    y_dot = 0;
                    theta_dot = 0;
                }
            } else if (nextState == pressButton) {
                targetPress = param - 48; // convert from ascii to dec
                timerTarget = t + PRESS_TIME;
                buttonServo.write(PRESS_ANGLE);
                servoTarget = true;
            } else if (nextState == discarding) {
                discardServo.write(DISCARD_ANGLE);
                // buttonServo.write(buttonServo.read());
                timerTarget = t + 2;
                servoTarget = true;
            } else if (nextState == movingRack) {
                moveRackService(param - 48);
            } else if (nextState == dispensing) {
                startConveyorService(true);
            } else if (nextState == lineFollowing) {
                startLineFollowing();
            } else if (nextState == coasting) {
                x_dot = 0;
                y_dot = 0;
                theta_dot = 0;
            } else if (nextState == waitingForBlock) {
                senseColorService();
            } else if (nextState == trajectory) {
                beginPathing();
                for (int i = 0; i < param; i++) {
                    char pathType = xbee.read();
                    float time = xbee.parseFloat();
                    float angle = xbee.parseFloat();
                    float param = xbee.parseFloat();
                    // swing turns have 3 params
                    // straight lines have 3 params
                    // point turns have 2 params, 3rd param will just be dropped
                    switch (pathType) {
                    case 's':
                        generateStraightPath(angle, param, time, i);
                        break;
                    case 'p':
                        generatePointTurn(angle, time, i);
                        break;
                    case 'h':
                        generateSwingTurn(param, 0, angle, time, i);
                        break;
                    }
                }
            }
        }
    }
}
