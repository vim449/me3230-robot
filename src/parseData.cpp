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

            if (nextState == drivingTimeBased) {
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
                pressButtonService(param -
                                   48); // -48 to convert from ascii to dec
            } else if (nextState == discarding) {
                discardBlockService();
            } else if (nextState == movingRack) {
                Serial.print("Moving rack to: ");
                Serial.println(param - 48);
                moveRackService(param - 48);
            } else if (nextState == dispensing) {
                if ((param - 48) % 2 == 0) { // dispense block
                    startConveyorService(true);
                } else { // store block
                    storeBlockService();
                }
            } else if (nextState == lineFollowing) {
                startLineFollowing();
            } else if (nextState == coasting) {
                startGame();
                return;
            } else if (nextState == waitingForBlock) {
                senseColorService();
            } else if (nextState == trajectory) {
                beginPathing();
                for (int i = 0; i < (param - 48); i++) {
                    char pathType = xbee.read();
                    float time = xbee.parseFloat(SKIP_WHITESPACE);
                    float angle = xbee.parseFloat(SKIP_WHITESPACE);
                    float pathParam = xbee.parseFloat(SKIP_WHITESPACE);
                    // Serial.print(pathType);
                    // Serial.print('\t');
                    // Serial.println(i);
                    // swing turns have 3 params
                    // straight lines have 3 params
                    // point turns have 2 params, 3rd param will just be dropped
                    switch (pathType) {
                    case 's':
                        generateStraightPath(angle, pathParam, time, i);
                        break;
                    case 'p':
                        generatePointTurn(angle, time, i);
                        break;
                    case 'h':
                        pathParam = pathParam > 0 ? pathParam + 0.1289
                                                  : pathParam - 0.1289;
                        generateSwingTurn(0, pathParam, angle, time, i);
                        break;
                    default:
                        Serial.println("Invalid path");
                        break;
                    }
                }
                t_old = t;
            } else if (nextState == moveGate) {
                if (param == '0') {
                    servoTarget = CLOSE;
                    gatePos = false;
                    gateServo.write(GATE_CLOSE_ANGLE);
                } else {
                    servoTarget = OPEN;
                    gatePos = true;
                    gateServo.write(GATE_OPEN_ANGLE);
                }
                timerTarget = t + GATE_TIME;
            } else if (nextState == waitingForData) {
                // I'm hijacking this for starting the competition
                compMode = true;
                gateService(true);
                currentLocation = start;
                strat->nextGoalCallback();
            }
        }
    }
}
