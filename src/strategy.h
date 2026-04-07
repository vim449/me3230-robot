#pragma once
#include "globals.h"

void driveToLeftMine();

class Strategy {
  public:
    Strategy();
    virtual void nextGoalCallback() = 0;
    virtual void handleBlock(BlockType) = 0;
    virtual void getCurrentCraftTarget() = 0;

  private:
    // TODO, determine needed member variables

    // boolean for each blocktype if it's needed
    // none, wood, stone, iron, diamond
    bool neededMaterials[5] = {false, false, false, false, false};
};

class defaultStrategy : public Strategy {
  public:
    void nextGoalCallback();
    void handleBlock(BlockType);
    void getCurrentCraftTarget();

  private:
    bool neededMaterials[5] = {false, false, false, false, false};
};
