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
    Shield
};

bool needFullRow(CraftType item);
BlockType requiredOre(CraftType item);

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
    bool storeShift = false;
    bool performingCraft = false;
    bool backRowDone = true;
    int craftPos = 8; // top to bottom left to right, 1->9
    bool neededMaterials[5] = {false, false, false, false, false};
    // CraftType craftList[3] = {StonePickaxe, IronPickaxe, DiamondPickaxe};
    CraftType craftList[0] = {};
    int currentCraft = 0;
};
