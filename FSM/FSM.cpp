#include "FSM.h"
#include "ConcreteFSMState.h"

FSM::FSM()
{
    currentState = &Wait::getInstance();
}

void FSM::setState(FSMState &newState)
{

    currentState->exit(this);

    currentState = &newState;

    currentState->enter(this);
}

void FSM::toggle()
{

    currentState->toggle(this);
}