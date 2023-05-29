#pragma once
#include "FSMState.h"
#include <bits/stdc++.h>
#include <iostream>
#include "controller.h"
#include <queue>

using namespace std;

class FSMState;

class FSM
{
public:
    FSM();
    inline FSMState *getCurrentState() const { return currentState; }
    void toggle();
    void setState(FSMState &newState);

    bool placeDown();
    bool pickUp();
    bool moveTo(coordinates pos, rotMatrix rot);
    void moveGripperTo(int diameter);
    bool isPositionQueueEmpty();
    void addPosition(coordinates pos, rotMatrix rot);
    pair<coordinates, rotMatrix> getNextPosition();

    bool isGripping = false;
    bool isError = false;
    bool isDone = false;

    int counter = 0;

private:
    queue<pair<coordinates, rotMatrix>> *positions;
    Controller *controller;
    FSMState *currentState;
};