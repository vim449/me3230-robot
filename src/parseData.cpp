#include "parseData.h"
#include "extern.h"
#include "globals.h"
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
          x_dot = 1;
          y_dot = 0;
          theta_dot = 0;
          break;
        case 'b':
          x_dot = -1;
          y_dot = 0;
          theta_dot = 0;
          break;
        case 'l':
          x_dot = 0;
          y_dot = -1;
          theta_dot = 0;
          break;
        case 'r':
          x_dot = 0;
          y_dot = 1;
          theta_dot = 0;
        case 'L':
          x_dot = 0;
          y_dot = 0;
          theta_dot = 1;
          break;
        case 'R':
          x_dot = 0;
          y_dot = 0;
          theta_dot = -1;
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
        Serial.println("triggered discard");
        discardServo.write(90);
        // buttonServo.write(buttonServo.read());
        timerTarget = t + 2;
        servoTarget = true;
      } else if (nextState == movingRack) {
        moveRackService(param - 48);
      } else if (nextState == dispensing) {
        startConveyorService(true);
      }
    }
  }
}