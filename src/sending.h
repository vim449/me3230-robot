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
