#pragma once
#include "FSMState.h"
#include "FSM.h"

class Wait : public FSMState
{
public:
    void enter(FSM *fsm) {}
    void toggle(FSM *fsm);
    void exit(FSM *fsm) {}
    static FSMState &getInstance();

private:
    Wait() {}
    Wait(const Wait &other);
    Wait &operator=(const Wait &other);
};
