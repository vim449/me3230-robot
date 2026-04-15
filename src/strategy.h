#pragma once
#include "globals.h"

void driveToLeftMine();
void driveToRightMine();
void driveToCraftingTable();

enum CraftType {
    StonePickaxe,
    IronPickaxe,
    DiamondPickaxe,
    StoneSword,
    IronSword,
    DiamondSword,
    Shield,
    NONE
};

bool needFullRow(CraftType item);
BlockType requiredOre(CraftType item);

class Strategy {
  public:
    Strategy();
    virtual void nextGoalCallback() = 0;
    virtual void handleBlock(BlockType) = 0;
    virtual void getCurrentCraftTarget() = 0;
    CraftType craftList[3] = {StonePickaxe, IronPickaxe, DiamondPickaxe};
    CraftType currentItem = craftList[0]; // just a helper variable to shorten other code
    bool backRowDone = false;

  private:
};

class defaultStrategy : public Strategy {
  public:
    void nextGoalCallback();
    void handleBlock(BlockType);
    void getCurrentCraftTarget();
    //CraftType craftList[3] = {StonePickaxe, IronPickaxe, DiamondPickaxe};
    CraftType craftList[2] = {StonePickaxe, IronPickaxe};
    CraftType currentItem = craftList[0]; // just a helper variable to shorten other code

  private:
    void finishCraft();
    void storeBlockServiceShifted();

    bool storeShift = false;
    bool performingCraft = false;
    int craftPos = 8; // top to bottom left to right, 1->9

    int currentCraft = 0;
};
