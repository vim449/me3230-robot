#include "strategy.h"
#include "followLine.h"
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
bool needsFullRow(CraftType item) {
    switch (item) {
    case StoneSword:
    case IronSword:
    case DiamondSword:
        return false;
    case StonePickaxe:
    case IronPickaxe:
    case DiamondPickaxe:
        return true;
    case Shield:
        return true;
    }
}

BlockType requiredOre(CraftType item) {
    switch (item) {
    case StonePickaxe:
    case StoneSword:
        return stone;
    case IronPickaxe:
    case IronSword:
    case Shield:
        return iron;
    case DiamondPickaxe:
    case DiamondSword:
        return diamond;
    }
}

void endOfPathCallback() {
    resetEncoders();
    currentLocation = headingLocation;
    strat->nextGoalCallback();
}

void slowApproach() {
    resetEncoders();
    currentLocation = headingLocation;
    stopDriveMotors();
    delay(100);
    startLineFollowing();
}

// generates a trajectory to drive to the left mine from start
void driveToLeftMine() {
    if (currentLocation == start) {
        // tested working
        beginPathing();
        // generateStraightPath(0.0, lineDist, 3, 0);
        // generateSwingTurn(0.0, turnRadius, PI / 2.0, 3, 1);
        // generateStraightPath(0.0, lineDist, 3, 2);
        generateStraightPath(-PI / 2.0, 0.81, 3.0, 0);
        generateStraightPath(0, 0.40, 2.0, 1);
    } else if (currentLocation == chest) {
        double x_dist = 0.45;
        double y_dist = 0.33;
        double total_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
        double ang = atan2(-y_dist, x_dist);
        generateStraightPath(ang, total_dist, 5.0, 0);
    } else if (currentLocation == craft) {
    }
    // TODO, chest and crafting table to mine
    headingLocation = leftMine;
    setCallback(&slowApproach);
    nextState = trajectory;
}

void driveToRightMine() {
    beginPathing();
    if (currentLocation == start) {
        if (false) {
            // right forward right
            // tested working
            generateStraightPath(-PI / 2.0, 0.77, 3.0, 0);
            generateStraightPath(0, 0.30, 1.5, 1);
            generateStraightPath(-PI / 2.0, 0.68, 2.5, 2);
        } else {

            // right diag right
            // tested working, more consistent
            generateStraightPath(-PI / 2.0, 0.5, 2.3, 0);
            generateStraightPath(-PI / 4.0, 0.65, 3.0, 1);
            generateStraightPath(-PI / 2.0, 0.48, 2.0, 2);
        }
    }
    headingLocation = rightMine;
    setCallback(&slowApproach);
    nextState = trajectory;
}

// generates a trajectory to drive to the right mine from start
void driveToRightTree() {
    beginPathing();
    if (currentLocation == start) {
        if (false) {
            // right forward right
            // tested working
            generateStraightPath(-PI / 2.0, 0.77, 3.0, 0);
            generateStraightPath(0, 0.30, 2.0, 1);
            generateStraightPath(-PI / 2.0, 1.30, 6.0, 2);
        } else {

            // right diag right
            // tested working, much more consistent of the two
            generateStraightPath(-PI / 2.0, 0.5, 2.3, 0);
            generateStraightPath(-PI / 4.0, 0.65, 3.0, 1);
            generateStraightPath(-PI / 2.0, 1.12, 3.8, 2);
        }
    } else if (currentLocation == craft) {
        // tested working
        double x_dist = 0.56;
        double y_dist = 0.83;
        double total_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
        double ang = atan2(-y_dist, x_dist);
        generateStraightPath(ang, total_dist, 5.0, 0);
    } else if (currentLocation == chest) {
        generateStraightPath(-PI + 0.30, 0.6, 3.0, 0);
    }
    headingLocation = rightMine;
    setCallback(&slowApproach);
    nextState = trajectory;
}

void driveToChest() {
    // TODO
    beginPathing();
    generateStraightPath(-PI + 0.30, 0.6, 3.0, 0);
    // if (currentLocation == leftMine) {
    // } else if (currentLocation == rightMine) {
    // } else if (currentLocation == leftTree) {
    // } else if (currentLocation == rightTree) {
    //     generateStraightPath(-0.46, 0.6, 3.0, 0);
    // } else if (currentLocation == start) {
    // } else if (currentLocation == craft) {
    // }
    headingLocation = chest;
    setCallback(&endOfPathCallback);
    nextState = trajectory;
}
void driveToCraftingTable() {
    beginPathing();
    double x_dist = -0.65;
    double y_dist;
    if (currentLocation == leftMine) {
        y_dist = 0.33;
    } else if (currentLocation == rightMine) {
        // TODO
        y_dist = -0.33;
    } else if (currentLocation == leftTree) {
        // TODO
        y_dist = 0.92;
    } else if (currentLocation == rightTree) {
        // TODO
        y_dist = -0.92;
    } else {
        // must be chest
        // TODO
        x_dist = 0.1;
        y_dist = -1.10;
    }
    double total_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
    double ang = atan2(-y_dist, x_dist);
    generateStraightPath(ang, total_dist, 5.0, 0);
    headingLocation = craft;
    setCallback(&endOfPathCallback);
    nextState = trajectory;
}

void setCraftOutput(CraftType item) {
    switch (item) {
    case StonePickaxe:
        pick = stone;
        break;
    case IronPickaxe:
        pick = iron;
        break;
    case DiamondPickaxe:
        pick = diamond;
        break;
    case StoneSword:
        sword = stone;
        break;
    case IronSword:
        sword = iron;
        break;
    case DiamondSword:
        sword = diamond;
        break;
    case Shield:
        shield = true;
        break;
    }
}

Strategy::Strategy() {}
void defaultStrategy::nextGoalCallback() {
    int fillCount = 0;
    switch (state) {
    case drivingTimeBased:
        break;
    case storingBlock:
        conveyor->setSpeed(0);
        if (currentLocation == leftMine || currentLocation == rightMine) {
            fillCount = 3;
        } else {
            // if crafting is needed, only grab two wood, otherwise 3
            fillCount = 3;
            // currentCraft < sizeof(craftList) / sizeof(craftList[0]) ? 2 : 3;
        }

        if (true) {
            // conveyor full
            // if (currentCraft < sizeof(craftList) / sizeof(craftList[0])) {
            //     // crafting needs to be done, discarding guarantees conveyor
            //     is
            //     // full with blocks useful for crafting
            //     driveToCraftingTable();
            // } else {
            //     // crafting doesn't need to be done, go deposit
            //     driveToChest();
            // }
            driveToChest();
        } else {
            // conveyor not full, need to approach wall again
            this->storeShift = false;
            slowApproach();
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
            conveyor->setSpeed(0);
            switch (this->craftPos) {
            case 1:
            case 2:
                // craft finished
                this->performingCraft = false;
                setCraftOutput(craftList[currentCraft]);
                currentCraft++;
                if (currentCraft < sizeof(craftList) / sizeof(craftList[0])) {
                    // not done crafting, go back to mines
                    driveToLeftMine();
                } else {
                    // done craftign, wood maxx
                    driveToRightTree();
                }
            case 3:
            case 4:
            case 5:
                // need to move to 2 to put in final block
                beginPathing();
                generateStraightPath(0, 0.085, 0.3, 0);
                headingLocation = craft;
                setCallback(&endOfPathCallback);
                this->craftPos = 2;
                nextState = trajectory;
                break;
            case 6:
            case 7:
                // need to move to 8 to put in next block
                beginPathing();
                generateStraightPath(-PI / 2.0, 0.085, 0.3, 0);
                headingLocation = craft;
                setCallback(&endOfPathCallback);
                this->craftPos = 8;
                nextState = trajectory;
                break;
            case 8:
                // need to move to 9 to put in next block
                if (needsFullRow(craftList[currentCraft])) {
                    // if a full row is needed, need to move to position 9
                    beginPathing();
                    generateStraightPath(-PI / 2.0, 0.085, 0.3, 0);
                    headingLocation = craft;
                    setCallback(&endOfPathCallback);
                    this->craftPos = 9;
                    nextState = trajectory;
                } else {
                    // if a full row isn't needed, move to position 5
                    this->backRowDone = true;
                    this->performingCraft = false;
                    gateServo.write(GATE_CLOSE_ANGLE);
                    // driveToRightTree();
                    // nextState = trajectory;
                    moveRackService(0);
                }
                break;
            case 9:
                // full belt of stone dispensed now need to grab wood.
                this->backRowDone = true;
                this->performingCraft = false;
                gateServo.write(GATE_CLOSE_ANGLE);
                // driveToRightTree();
                // nextState = trajectory;
                moveRackService(0);
                break;
            }
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
        this->handleBlock(inShovel);
        break;
    case movingRack:
        if (currentLocation == chest || currentLocation == craft) {
            if (currentRack == 1) {
                // rack finished extending, need to handle dispensing
                if (currentLocation == chest) {
                    // gateService(true); // open gate
                    gateServo.write(GATE_OPEN_ANGLE);
                    startConveyorService(true);
                } else {
                    this->performingCraft = true;
                    if (!this->backRowDone) {
                        // begin micro adjustments
                        if (needsFullRow(craftList[currentCraft])) {
                            beginPathing();
                            generateStraightPath(PI / 2.0, 0.09, 0.5, 0);
                            headingLocation = craft;
                            setCallback(&endOfPathCallback);
                            this->craftPos = 9;
                            nextState = trajectory;
                        } else {
                            this->craftPos = 8;
                            startConveyorService(true);
                        }
                    } else {
                        // need to place 2 more blocks, start by driving
                        // forwards
                        beginPathing();
                        generateStraightPath(0, 0.09, 0.5, 0);
                        headingLocation = craft;
                        setCallback(&endOfPathCallback);
                        this->craftPos = 5;
                        nextState = trajectory;
                    }
                }
            } else {
                // rack finished retracting, needs to drive somewhere
                if (this->performingCraft) {
                    driveToRightTree();
                } else {
                    // can really go wherever, going to defaul to left mine
                    driveToLeftMine();
                }
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
                    inShovel = wood;
                }
            }
        }
        break;
    case lineFollowing:
        if (currentLocation == leftMine || currentLocation == rightMine) {
            senseColorService();
        } else if (currentLocation == leftTree ||
                   currentLocation == rightTree) {
            pressButtonService(getBlockHits(wood));
            inShovel = wood;
        }
        break;
    case waitingForData:
    case coasting:
        // driveToLeftMine();
        driveToRightTree();
        Serial.print("Currently want block ");
        printBlock(requiredOre(craftList[0]));
        Serial.println();
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
            pressButtonService(getBlockHits(wood));
            inShovel = wood;
        } else if (currentLocation == chest || currentLocation == craft) {
            if (!this->performingCraft) {
                // extend rack to begin crafting
                moveRackService(1);
            } else {
                gateServo.write(GATE_OPEN_ANGLE);
                startConveyorService(true);
            }
        }
        break;
    }
}

void storeBlockServiceShifted() {
    beginPathing();
    generateStraightPath(0, -0.05, 0.33, 0);
    nextState = trajectory;
    setCallback(&storeBlockService);
}

void defaultStrategy::getCurrentCraftTarget() {}
void defaultStrategy::handleBlock(BlockType block) {
    // TODO, write logic to determine if block should be stored or discarded
    // currently just discarding if it's an unminable block
    if (currentLocation == leftMine || currentLocation == rightMine) {
        if (inShovel == requiredOre(craftList[this->currentCraft])) {
            // cant actually store directly, need to shift backwards and then
            // store block
            this->storeShift = true;
            storeBlockServiceShifted();
        } else {
            moveRackService(1);
        }
    } else {
        // mined at a tree
        storeBlockServiceShifted();
    }
}
