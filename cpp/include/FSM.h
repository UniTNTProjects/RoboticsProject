#pragma once
#include "FSMState.h"
#include <bits/stdc++.h>
#include <iostream>
#include "controller.h"

using namespace std;

class FSMState;

class FSM
{
public:
    FSM();
    inline FSMState *getCurrentState() const { return currentState; }
    void toggle();
    void setState(FSMState &newState);
    int currentPosIndex = -1;
    vector<tuple<double, double, double>> positions;
    Controller controller;

private:
    FSMState *currentState;
};