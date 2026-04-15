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

void defaultStrategy::storeBlockServiceShifted() {
    beginPathing();
    this->storeShift = true;
    generateStraightPath(0, -0.05, 0.33, 0);
    nextState = trajectory;
    setCallback(&storeBlockService);
}

void printCraft(CraftType item) {
    switch (item) {
    case StoneSword:
      Serial.print("Stone Sword");
      break;
    case IronSword:
      Serial.print("Iron Sword");
      break;
    case DiamondSword:
      Serial.print("Diamond Sword");
      break;
    case StonePickaxe:
      Serial.print("Stone Pickaxe");
      break;
    case IronPickaxe:
      Serial.print("Iron Pickaxe");
      break;
    case DiamondPickaxe:
      Serial.print("Diamond Pickaxe");
      break;
    case Shield:
      Serial.print("Shield");
      break;
    case NONE:
      Serial.print("nothing");
      break;
    }
}

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
    case NONE:
        // this should never be called
        shouldReset = true;
        return false;
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
    case NONE:
        // this should never be called
        shouldReset = true;
        return none;
    }
}

int numRequiredOre(CraftType item) {
  switch (item) {
    case StonePickaxe:
    case IronPickaxe:
    case DiamondPickaxe:
      return 3;
    case StoneSword:
    case IronSword:
    case DiamondSword:
      return 2;
    case Shield:
      return 1;
    case NONE:
      return 3;
  }
}

int numRequiredWood(CraftType item) {
  switch (item) {
    case StonePickaxe:
    case IronPickaxe:
    case DiamondPickaxe:
      return 2;
    case StoneSword:
    case IronSword:
    case DiamondSword:
      return 1;
    case Shield:
      return 3;
    case NONE:
      return 3;
  }
}

int numRequiredOre() {
  return numRequiredOre(strat->currentItem);
}
int numRequiredWood() {
  return numRequiredWood(strat->currentItem);
}

void endOfPathCallback() {
    resetEncoders();
    currentLocation = headingLocation;
    strat->nextGoalCallback();
}

void slowApproach() {
    resetEncoders();
    buttonServo.write(PRESS_RETRACT_ANGLE);
    currentLocation = headingLocation;
    stopDriveMotors();
    delay(20); //micro delay to make sure line following has no inertia
    startLineFollowing();
    targetDist = longDist;
}

void slowApproachBack() {
    resetEncoders();
    currentLocation = headingLocation;
    stopDriveMotors();
    delay(20); //micro delay to make sure line following has no inertia
    mapCenterOfRotation(0.08, 0, true);
    startLineFollowingBack();
    targetDist = 5.0;
}

// generates a trajectory to drive to the left mine from start
void driveToLeftMine() {
    beginPathing();
    if (currentLocation == start) {
        // tested working
        // generateStraightPath(0.0, lineDist, 3, 0);
        // generateSwingTurn(0.0, turnRadius, PI / 2.0, 3, 1);
        // generateStraightPath(0.0, lineDist, 3, 2);
        generateStraightPath(-PI / 2.0, 0.81, 2.5, 0);
        generateStraightPath(0, 0.40, 1.25, 1);
    } else if (currentLocation == chest) {
      // needs testing
        double x_dist = 0.45;
        double y_dist = 0.33;
        double total_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
        double ang = atan2(-y_dist, x_dist);
        generateStraightPath(ang, total_dist, 5.0, 0);
    } else if (currentLocation == craft) {
      // TODO, needs calibrating
        // double x_dist = 0.45;
        // double y_dist = -0.33;
        // double total_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
        // double ang = atan2(-y_dist, x_dist);
        // generateStraightPath(ang, total_dist, 5.0, 0);
        generateStraightPath(PI/2.0, 0.32, 1.5, 0);
        generateStraightPath(0, 0.3, 1.5, 1);
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
        // double x_dist = 0.46;
        // double y_dist = 2;
        // double total_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
        // double ang = atan2(-y_dist, x_dist);
        // generateStraightPath(ang, total_dist, 6.0, 0);
      generateStraightPath(0, 0.2, 1.5, 0);
      generateStraightPath(-PI/2.0, 0.96 + (strat->backRowDone * 0.09), 4.0, 1);
    } else if (currentLocation == chest) {
        generateStraightPath(0.30, 0.6, 3.0, 0);
    }
    headingLocation = rightTree;
    setCallback(&slowApproach);
    nextState = trajectory;
}

void driveToChest() {
    // TODO
    beginPathing();
    if (currentLocation == rightTree) {
    generateStraightPath(-PI + 0.30, 0.6, 3.0, 0);
    } else if (currentLocation == leftMine) {
    generateStraightPath(0.0, 0.10, 0.35, 0);
    generateStraightPath(0.0, -0.10, 0.35, 1);
    generateStraightPath(-PI/2.0, 1.2, 4.8*1.2, 2);
    generateStraightPath(0.0, -0.585, 3.5, 3);
    }
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
    double x_dist = -0.60;
    double y_dist;
    if (currentLocation == leftMine) {
        y_dist = 0.315;
    } else if (currentLocation == rightMine) {
        // TODO
        y_dist = -0.35;
    } else if (currentLocation == leftTree) {
        // TODO
        y_dist = 0.95;
    } else if (currentLocation == rightTree) {
        // TODO
        y_dist = -0.93;
    } else {
        // must be chest
        // TODO
        x_dist = 0.1;
        y_dist = -1.10;
    }
    // double total_dist = sqrt(x_dist * x_dist + y_dist * y_dist);
    // double ang = atan2(-y_dist, x_dist);
    // generateStraightPath(ang, total_dist, 5.0, 0);
    fullJacobian = motorJacobian;
    buttonServo.write(PRESS_STORE_ANGLE);
    generateStraightPath(0.0, 0.10, 0.35, 0);
    generateStraightPath(0.0, -0.10, 0.35, 1);
    generateStraightPath(-PI/2.0, y_dist, 4.8*abs(y_dist), 2);
    generateStraightPath(0.0, -0.61, 3.5, 3);
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

void defaultStrategy::finishCraft() {
    this->performingCraft = false;
    this->backRowDone = false;
    setCraftOutput(craftList[currentCraft]);
    currentCraft++;
    if ( currentCraft >= (sizeof(craftList) / sizeof(craftList[0])) ) {
        // all crafts are finished
        currentItem = NONE;
    } else {
        this->currentItem = craftList[currentCraft];
    }
}

Strategy::Strategy() {}
void defaultStrategy::nextGoalCallback() {
    int fillCount;
    switch (state) {
    case drivingTimeBased:
        break;
    case storingBlock:
        conveyor->setSpeed(0);
        if (currentCraft != NONE) {
            if (currentLocation == leftMine || currentLocation == rightMine) {
                fillCount = numRequiredOre();
            } else {
                fillCount = numRequiredWood();
            }
        } else {
            fillCount = 3;
        }
#ifdef DEBUG
        Serial.print("Attempting to get: ");
        Serial.print(fillCount);
        Serial.println(" blocks");
        Serial.print("Have ");
        Serial.print(numStored);
        Serial.println(" blocks stored");
#endif

        if (numStored >= fillCount) {
            //conveyor full
            if (currentItem != NONE) {
                // crafting needs to be done, discarding guarantees conveyor is
                // full with blocks useful for crafting
                driveToCraftingTable();
            } else {
                // crafting doesn't need to be done, go deposit
                driveToChest();
            }
        } else {
            // conveyor not full, need to approach wall again
            this->storeShift = false;
            slowApproach();
            targetDist = shortDist; // robot is driving slow, get closer to the wall
        }
        break;
    case dispensing:
        numStored--;
        if (currentLocation == chest) {
            // if conveyor isn't empty, dispense again
            if (numStored > 0) {
                startConveyorService(true);
            } else {
                // done depositing, move back to wherever
                // TODO, logic to determine where to drive to
                moveRackService(0);
            }
        } else {
            // at crafting table, need to move before placing next block
            conveyor->setSpeed(0);
            switch (this->craftPos) {
            case 1:
              // this position should never be hit
              break;
            case 2:
                // craft finished
                finishCraft();
                // close rack, then handle moving elsewhere
                moveRackService(0);
                break;
            case 3:
                break;
            case 4:
                break;
            case 5:
                // need to move to 2 to put in final block
                delay(200);
                beginPathing();
                generateStraightPath(0, 0.10, 0.8, 0);
                headingLocation = craft;
                setCallback(&endOfPathCallback);
                this->craftPos = 2;
                nextState = trajectory;
                break;
            case 6:
            case 7:
                // need to move to 8 to put in next block
                delay(200);
                beginPathing();
                generateStraightPath(-PI / 2.0, -0.08, 0.8, 0);
                headingLocation = craft;
                setCallback(&endOfPathCallback);
                this->craftPos = 8;
                nextState = trajectory;
                break;
            case 8:
                // need to move to 9 to put in next block
                // if(true) {
                if (needsFullRow(currentItem)) {
                    // if a full row is needed, need to move to position 9
                    delay(200);
                    beginPathing();
                    generateStraightPath(-PI / 2.0, -0.08, 0.8, 0);
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
                currentLocation = craft;
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
            // just killed the silverfish, handle block
            this->handleBlock(inShovel);
        } else {
            // block will drop, wait for block
            // lockout time
            timerTarget = t + 1.2;
            nextState = waitingForBlock;
        }

        break;
    case waitingForBlock:
        this->handleBlock(inShovel);
        break;
    case movingRack:
#ifdef DEBUG
        Serial.print("Attempting to craft: ");
        printCraft(currentItem);
        Serial.println();

        Serial.print("Back row state: ");
        Serial.println(backRowDone ? "done" : "not done");

        Serial.print("Currently at: ");
        printLocation();
        Serial.println();
#endif
        if (currentLocation == chest || currentLocation == craft) {
            Serial.println("Crafting table check triggered");
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
                        if (needsFullRow(currentItem)) {
                            beginPathing();
                            delay(200);
                            generateStraightPath(PI / 2.0, -0.08, 0.8, 0);
                            headingLocation = craft;
                            setCallback(&endOfPathCallback);
                            this->craftPos = 7;
                            nextState = trajectory;
                        } else {
                            this->craftPos = 8;
                            startConveyorService(true);
                        }
                    } else {
                        // need to place 2 more blocks, start by driving
                        // forwards
                        beginPathing();
                        delay(200);
                        generateStraightPath(0, 0.11, 0.8, 0);
                        headingLocation = craft;
                        setCallback(&endOfPathCallback);
                        this->craftPos = 5;
                        nextState = trajectory;
                    }
                }
            } else {
                limitStates[2] = digitalRead(limitPins[2]);
                while (limitStates[2] == 1) {
                    limitStates[2] = digitalRead(limitPins[2]);
                    conveyor->setSpeed(350);
                }
                conveyor->setSpeed(0);
                gateServo.write(GATE_CLOSE_ANGLE);
                // driveToRightTree();
                //rack finished retracting, needs to drive somewhere
                if (this->performingCraft == false && this->backRowDone == true) {
                  driveToRightTree();
                } else if (this->performingCraft) {
                    driveToRightTree();
                } else {
                  // determine where to go to start new craft
                  if (currentItem != NONE) {
                      // not done crafting, go back to mines
                      driveToLeftMine();
                  } else {
                      // done craftign, wood maxx
                    if (pick == iron || pick == diamond) {
                      // can mine every block, go for more score
                      driveToLeftMine();
                    } else {
                      driveToRightTree();
                    }
                  }
                }
            }
        } else {
          Serial.println("Not at crafting table");
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
        } else {
            if (!this->performingCraft) {
                // extend rack to begin crafting
                moveRackService(1);
            } else {
                gateServo.write(GATE_OPEN_ANGLE);
                startConveyorService(true);
            }
        }
        break;
    case waitingForData:
    case coasting:
        driveToLeftMine();
        // driveToRightTree();
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
          // in theory this should never trigger as I always line follow to the wall
          pressButtonService(12);
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


void defaultStrategy::getCurrentCraftTarget() {}
void defaultStrategy::handleBlock(BlockType block) {
    // TODO, write logic to determine if block should be stored or discarded
    // currently just discarding if it's an unminable block
    //currentLocation = leftMine;
    if (currentLocation == leftMine || currentLocation == rightMine) {
        if (inShovel == requiredOre(craftList[this->currentCraft])) {
            // cant actually store directly, need to shift backwards and then
            // store block
            this->storeBlockServiceShifted();
        } else {
            moveRackService(1);
        }
    } else {
        // mined at a tree
        this->storeBlockServiceShifted();
    }
}
