#include "ConcreteFSMState.h"

void Wait::toggle(FSM *fsm)
{
    fsm->setState(Wait::getInstance());
}

void Wait::enter(FSM *fsm)
{
    // Do something
}

void Wait::exit(FSM *fsm)
{
    // Do something
}

FSMState &Wait::getInstance()
{
    static Wait instance;
    return instance;
}