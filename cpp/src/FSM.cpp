#include "FSM.h"
#include "ConcreteFSMState.h"

FSM::FSM()
{
    currentState = &Wait::getInstance();
    controller = Controller(1000.);
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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "controller_ur5");

    FSM fsm;
    fsm = FSM();
    while (true)
    {
        fsm.toggle();
    }

    return 0;
}