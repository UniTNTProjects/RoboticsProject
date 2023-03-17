#pragma once
#include "FSM.h"

// Forward declaration to resolve circular dependency/include
class FSM;

class FSMState
{
public:
    virtual void enter(FSM *fsm) = 0;
    virtual void toggle(FSM *fsm) = 0;
    virtual void exit(FSM *fsm) = 0;
    virtual ~FSMState() {}
};