#include "SoftwareSerial.h"
#include "sending.h"
#include <Arduino.h>

SoftwareSerial xbee(2, 3);
bool needParam = false;
bool readyToSend = false;
State state;
uint8_t param = 0;

void menu() {
  // menu printer

  // valid options are: Drive, Press Button, discard, dispense
  Serial.println("Please select a command to send to the mega");
  Serial.println("Options are Drive: 'a', Press Button 'b', discard 'c', "
                 "dispense 'd', move rack 'e'");
}

void menu(State submenu) {
  // submenu printer
  switch (submenu) {
  case driving:
    Serial.println("Please enter one of the following");
    Serial.println(
        "(f)orwards, (b)ackwards, (l)eft, (r)ight, turn (L)eft, turn (R)ight");
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
  }
}

void setup() {
  Serial.begin(9600);
  xbee.begin(115200);
  menu();
}

void loop() {
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
    Serial.print(stateToNum(state));
    Serial.print(" ");
    Serial.println(param);

    xbee.write(START_MESSAGE);
    xbee.write(stateToNum(state));
    xbee.write(param);
    delay(20);
    readyToSend = false;
    needParam = false;
    menu();
  }
}