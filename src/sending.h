// all code that needs to exist on both the uno and mega for wireless
// transmission state handling is stored here
#pragma once

#define START_MESSAGE 255
#define SOFTWARE_STOP 200
#define SOFTWARE_END 150
#define SOFTWARE_START 100

enum State {
    drivingTimeBased,
    storingBlock,
    dispensing,
    discarding,
    pressButton,
    waitingForBlock,
    movingRack,
    waitingForData,
    lineFollowing,
    coasting,
    moveGate,
    trajectory,
};
