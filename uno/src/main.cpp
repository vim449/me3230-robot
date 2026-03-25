#include "SoftwareSerial.h"
#include "sending.h"
#include <Arduino.h>

SoftwareSerial xbee(2, 3);
bool needParam = false;
bool readyToSend = false;
bool inState = false;
State state;
uint8_t param = 0;

enum BlockType { none, wood, stone, iron, diamond };

void menu() {
  // menu printer

  // valid options are: Drive, Press Button, discard, dispense
  Serial.println("Please select a command to send to the mega");
  Serial.println("Options are Drive: 'a', Press Button 'b', discard 'c', "
                 "dispense 'd', move rack 'e', line follow 'l', coast 'C', "
                 "sense color 's', path follow 'p'");
}

void menu(State submenu) {
  // submenu printer
  switch (submenu) {
  case driving:
    Serial.println("Please enter one of the following");
    Serial.println("(f)orwards, (b)ackwards, (l)eft, (r)ight, turn (L)eft, "
                   "turn (R)ight");
    break;
  case pressButton:
    Serial.println("Please enter how many times the button should be pressed");
    break;
  case dispensing:
    Serial.println(
        "Please enter a number of seconds to drive the conveyor for");
    break;
  case movingRack:
    Serial.println("Please enter the index of the limit switch to go to");
    break;
  case lineFollowing:
    Serial.println("Now line following");
    break;
  case coasting:
    Serial.println("Now Coasting");
    break;
  case waitingForBlock:
    Serial.println("Sensing block color");
    break;
  case moveGate:
    Serial.println("Press anything but 0 to open the gate, 0 to close");
    break;
  }
}

void setup() {
  Serial.begin(9600);
  xbee.begin(115200);
  menu();
}

char getType() {
  Serial.println("What type of path would you like?");
  Serial.println("'s' for straight, 'p' for point turn, 'h' for swing turn");
  char path = '\n';
  while (path == '\n') {
    // determines how many total segments need parameters
    if (Serial.available()) {
      path = Serial.read();
      if (path == 's' || path == 'p' || path == 'h') {
        String substr = path == 's'   ? "Straight"
                        : path == 'p' ? "point turn"
                                      : "swing turn";
        Serial.print("\nSelected path type ");
        Serial.println(substr);
        return path;
      } else {
        path = '\n';
      }
    }
  }
  return 's';
}

float getTime() {
  Serial.println("Please enter how many seconds this path should take, press "
                 "enter to finish");
  float time = -1.0;
  while (time == -1.0) {
    if (Serial.available()) {
      String parseString = Serial.readStringUntil('\n');
      if (parseString != NULL) {
        time = parseString.toFloat();
        if (time < 0 || time > 10) {
          Serial.println("invalid time entered");
          time = -1.0;
        }
      }
    }
  }
  return time;
}
#define NULL_ANGLE 1080.0
#define MAX_SEGMENTS 8
void pathFunction() {
  Serial.setTimeout(50000);
  inState = false;
  Serial.println("How many segments would you like to define?");
  int segments = -1;
  while (segments == -1) {
    // determines how many total segments need parameters
    if (Serial.available()) {
      segments = Serial.read();
      segments -= 48;
      if (segments > MAX_SEGMENTS || segments < 1) {
        Serial.println("Invalid segment count, please enter again");
        segments = -1;
      }
    }
  }
  Serial.print("\nNeed parameters for ");
  Serial.print(segments);
  Serial.println(" segments");

  char pathList[MAX_SEGMENTS];
  float timeList[MAX_SEGMENTS];
  float angleList[MAX_SEGMENTS];
  float paramList[MAX_SEGMENTS];
  for (int i = 0; i < segments; i++) {
    Serial.print("Defining arguments for path segment ");
    Serial.println(i + 1);
    char path = getType();
    float time = getTime();

    float angle = NULL_ANGLE;
    Serial.println("Please enter the angle (in degrees) this path should "
                   "be along, press "
                   "enter to finish");
    while (angle == NULL_ANGLE) {
      if (Serial.available()) {
        String parseString = Serial.readStringUntil('\n');
        if (parseString != NULL) {
          angle = parseString.toFloat();
          angle = angle * PI / 180; // convert from deg to rad
        }
      }
    }

    float param = -1;
    switch (path) {
    case 's':
      Serial.print("Please enter the distance (m) the robot should drive");
      break;
    case 'p':
      param = 0.0; // point turns don't need an extra parameter, exit
                   // immediately
      break;
    case 'h':
      Serial.print("Please enter the radius (m) of swing turn the robot should "
                   "make (negative for left turns)");
      break;
    }
    Serial.println(" press enter to finish");
    while (param == -1) {
      if (Serial.available()) {
        String parseString = Serial.readStringUntil('\n');
        if (parseString != NULL) {
          param = parseString.toFloat();
        }
      }
    }
    pathList[i] = path;
    timeList[i] = time;
    angleList[i] = angle;
    paramList[i] = param;
  }

  // at this point, all parameters are finally ready, send to mega
  xbee.write(START_MESSAGE);
  xbee.write((byte)trajectory);
  xbee.print(segments);
  Serial.print(segments);
  Serial.print('\t');
  for (int i = 0; i < segments; i++) {
    xbee.print(pathList[i]);
    xbee.print(timeList[i]);
    xbee.print('\t'); // separator for floats
    xbee.print(angleList[i]);
    xbee.print('\t'); // separator for floats
    xbee.print(paramList[i]);
    // xbee.print('\t'); // separator for floats

    Serial.print(pathList[i]);
    Serial.print(' ');
    Serial.print(timeList[i]);
    Serial.print('\t'); // separator for floats
    Serial.print(angleList[i]);
    Serial.print('\t'); // separator for floats
    Serial.print(paramList[i]);
    Serial.print('\t'); // separator for floats
  }
  xbee.println();
  delay(20);
}

void loop() {
  if (inState) {
    if (xbee.available() > 0) {
      switch ((BlockType)xbee.read()) {
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
      case none:
        Serial.println("No Block");
        break;
      default:
        Serial.println("Silverfish");
        break;
      }
      // Serial.println(xbee.read());
    }
    if (Serial.available() > 0) {
      int arg = Serial.read();
      if (arg == '1') {
        inState = false;
        xbee.write(SOFTWARE_STOP);
        Serial.println("Sent software stop");
        menu();
      } else {
        Serial.println("Ignored");
      }
    }
  } else {
    if (Serial.available() > 0) {
      int arg = Serial.read();
      if (arg != '\n') {
        if (!needParam) {
          // reading a state
          switch (arg) {
          case 'a':
            menu(driving);
            state = driving;
            needParam = true;
            break;
          case 'b':
            menu(pressButton);
            state = pressButton;
            needParam = true;
            break;
          case 'c':
            state = discarding;
            param = 0;
            needParam = false;
            readyToSend = true;
            break;
          case 'd':
            menu(dispensing);
            state = dispensing;
            needParam = true;
            break;
          case 'e':
            menu(movingRack);
            state = movingRack;
            needParam = true;
            break;
          case 'l':
            menu(lineFollowing);
            state = lineFollowing;
            needParam = false;
            readyToSend = true;
            break;
          case 'C':
            menu(coasting);
            state = coasting;
            needParam = false;
            readyToSend = true;
            break;
          case 's':
            menu(waitingForBlock);
            state = waitingForBlock;
            needParam = false;
            readyToSend = true;
            break;
          case 'g':
            menu(moveGate);
            state = moveGate;
            needParam = true;
            readyToSend = false;
            break;
          case 'p':
            pathFunction(); // this function handles all of the
                            // pathing input,
                            // including writing to the mega
            inState = true;
            break;
          default:
            break;
          }
        } else {
          // reading a param for a state
          param = arg;
          readyToSend = true;
        }
      }
    }

    if (readyToSend) {
      Serial.print("Wrote to Mega ");
      Serial.print(START_MESSAGE);
      Serial.print(" ");
      Serial.print((byte)state);
      Serial.print(" ");
      Serial.println(param);

      xbee.write(START_MESSAGE);
      xbee.write((byte)state);
      xbee.write(param);
      delay(20);
      readyToSend = false;
      needParam = false;
      inState = true;

      Serial.println("Enter 1 to exit current state");
    }
  }
}
