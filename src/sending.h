#include <Arduino.h>
#define START_MESSAGE 255
#define SOFTWARE_STOP 200
#define SOFTWARE_START 100

enum State {
  driving,
  storing,
  dispensing,
  discarding,
  pressButton,
  waitingForBlock,
  movingRack,
  waitingForData,
};

State numToState(uint8_t num) {
  switch (num) {
  case 0:
    return driving;
  case 1:
    return storing;
  case 2:
    return dispensing;
  case 3:
    return discarding;
  case 4:
    return pressButton;
  case 5:
    return waitingForBlock;
  case 6:
    return movingRack;
  case 7:
    return waitingForData;
  default:
    return waitingForData;
  }
}

uint8_t stateToNum(State state) {
  switch (state) {
  case driving:
    return 0;
  case storing:
    return 1;
  case dispensing:
    return 2;
  case discarding:
    return 3;
  case pressButton:
    return 4;
  case waitingForBlock:
    return 5;
  case movingRack:
    return 6;
  case waitingForData:
    return 7;
  default:
    return 7;
  }
}