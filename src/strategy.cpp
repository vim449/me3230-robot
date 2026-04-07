#include "strategy.h"
#include "sending.h"
#include <drive.h>
#include <extern.h>
#include <services.h>

// A strategy needs to define:
// what to do with a block type
// when to stop mining
// where to go once done mining
// desired crafts
// which mine and tree to use
//

void endOfPathCallback() {
    resetEncoders();
    currentLocation = headingLocation;
    strat->nextGoalCallback();
}

// generates a trajectory to drive to the left mine from start
void driveToLeftMine() {
    if (currentLocation == start) {
        const float lineDist = 0.5;
        const float turnRadius = 0.20;
        beginPathing();
        generateStraightPath(0.0, lineDist, 3, 0);
        generateSwingTurn(0.0, turnRadius, PI / 2.0, 3, 1);
        generateStraightPath(0.0, lineDist, 3, 2);
    }
    headingLocation = leftMine;
    setCallback(&endOfPathCallback);
}

// generates a trajectory to drive to the right mine from start
void driveToRightMine() {
    if (currentLocation == start) {
        const float lineDist = 0.5;
        const float turnRadius = 0.20;
        beginPathing();
        generateStraightPath(0.0, lineDist, 3, 0);
        generateSwingTurn(0.0, turnRadius, PI / 2.0, 3, 1);
        if (true) {
            generateStraightPath(0.0, lineDist, 3, 2);
            // strafe to mine
            generateStraightPath(-PI / 2.0, 24.0 * 0.0254, 3, 3);
        } else {
            // drives directly from turn to mine
            const float dist =
                sqrt(lineDist * lineDist +
                     (24.0 * 0.0254) *
                         (24.0 * 0.0254)); // straight line distance from
                                           // end of turn to the mine
            const float ang = -atan2(24.0 * 0.0254, lineDist);
            generateStraightPath(ang, dist, 3, 1);
        }
    }
    headingLocation = rightMine;
    setCallback(&endOfPathCallback);
}

void driveToChest() {
    beginPathing();
    if (currentLocation == leftMine) {
    } else if (currentLocation == rightMine) {
    } else if (currentLocation == leftTree) {
    } else if (currentLocation == rightTree) {
    } else if (currentLocation == start) {
    } else if (currentLocation == craft) {
    }
    headingLocation = chest;
    setCallback(&endOfPathCallback);
}
void driveToCraftingTable() {
    if (currentLocation == leftMine) {
    } else if (currentLocation == rightMine) {
    } else if (currentLocation == leftTree) {
    } else if (currentLocation == rightTree) {
    } else if (currentLocation == start) {
    } else if (currentLocation == chest) {
    }
    headingLocation = craft;
    setCallback(&endOfPathCallback);
}

Strategy::Strategy() {}
void defaultStrategy::nextGoalCallback() {
    int craftBlocks = 0;
    switch (state) {
    case drivingTimeBased:
        break;
    case storingBlock:
        if (stored[2] != none) {
            // conveyor full
            for (BlockType b : stored) {
                craftBlocks += neededMaterials[(int)b];
            }
            if (craftBlocks > 0) {
                // contains blocks that can be crafted
                driveToCraftingTable();
            } else {
                // current blocks are not useful for current craft
                driveToChest();
            }
        } else {
            // conveyor not full, mine again
            if (currentLocation == leftMine || currentLocation == rightMine) {
                senseColorService();
            } else {
                pressButtonService(getBlockHits(wood));
            }
        }
        break;
    case dispensing:
        if (currentLocation == chest) {
            // if conveyor isn't empty, dispense again
            if (stored[0] != none) {
                startConveyorService(true);
            } else {
                // done depositing, move back to wherever
                // TODO, logic to determine where to drive to
                driveToLeftMine();
            }
        } else {
            // at crafting table, need to move before placing next block
        }
        break;
    case discarding:
        // after discarding a block, need to move rack and pinion forwards, then
        // mine again
        moveRackService(0);
        break;
    case pressButton:
        if (requireAttacking) {
            // just killed a silverfish, store the block
            storeBlockService();
        } else {
            // block will drop, wait for block
            // lockout time
            timerTarget = t + 20;
            nextState = waitingForBlock;
        }

        break;
    case waitingForBlock:
        strat->handleBlock(inShovel);
        break;
    case movingRack:
        if (currentLocation == chest || currentLocation == craft) {
            if (currentRack == 1) {
                // rack finished extending, need to handle dispensing
                if (currentLocation == chest) {
                    gateService(true); // open gate
                } else if (currentLocation == craft) {
                }
            } else {
                // rack finished retracting, needs to drive somewhere
            }
        } else {
            // rack was moved at one of the mining locations, must have been for
            // discarding
            if (currentRack == 1) {
                // rack finished extending, begin discard
                discardBlockService();
            } else {
                // rack retracted in, mine again
                if (currentLocation == rightMine ||
                    currentLocation == leftMine) {
                    senseColorService();
                } else {
                    pressButtonService(getBlockHits(wood));
                }
            }
        }
        break;
    case waitingForData:
        // at start of comp, need to go from start to left mine
        driveToLeftMine();
        break;
    case lineFollowing:
        break;
    case coasting:
        break;
    case moveGate:
        if (gatePos) {
            // gate is open, can begin dispensing
            if (currentLocation == craft) {
                startConveyorService(true);
            } else {
                // TODO, crafting table adjusments
            }
        } else {
            // gate is closed, do other shit
        }
        break;
    case trajectory:
        if (currentLocation == rightMine || currentLocation == leftMine) {
            senseColorService();
        } else if (currentLocation == rightTree ||
                   currentLocation == leftTree) {

        } else if (currentLocation == chest || currentLocation == craft) {
            // extend rack to begin crafting
            moveRackService(1);
        }
        break;
    }
}

void defaultStrategy::getCurrentCraftTarget() {}
void defaultStrategy::handleBlock(BlockType block) {
    // TODO, write logic to determine if block should be stored or discarded
    // currently just discarding if it's an unminable block
    if (inShovel == none) {
        // discardBlockService();
        storeBlockService();
    } else {
        storeBlockService();
    }
}
