#pragma once
#include "FSMState.h"

class FSMState;

class FSM
{
public:
    FSM();
    // Same as before
    inline FSMState *getCurrentState() const { return currentState; }
    // In here, we'll delegate the state transition to the currentState
    void toggle();
    // This will get called by the current state
    void setState(FSMState &newState);

private:
    // LightState here is now a class, not the enum that we saw earlier
    FSMState *currentState;
};