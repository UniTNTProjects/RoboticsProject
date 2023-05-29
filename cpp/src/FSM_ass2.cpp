#include "ConcreteFSMState.h"

using namespace std;

void Wait::toggle(FSM *fsm)
{

    if (fsm->counter++ < 1)
    {
        fsm->setState(Move::getInstance());
    }
    else
    {
        fsm->isDone = true;
    }
}

void Wait::enter(FSM *fsm)
{
    // Do something
    cout << "Entered Wait State" << endl;
}

void Wait::exit(FSM *fsm)
{
    // Do something
    coordinates a, b;
    a << 0.2, -0.2, 0.6;
    b << 0.3, -0.3, 0.6;

    rotMatrix rot;
    rot << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;

    fsm->addPosition(a, rot);
    fsm->addPosition(b, rot);

    cout << "Exited Wait State" << endl
         << endl;
}

FSMState &Wait::getInstance()
{
    static Wait instance;
    return instance;
}

void Move::toggle(FSM *fsm)
{

    if (fsm->isError)
    {
        fsm->setState(Wait::getInstance());
    }
    else
    {
        if (fsm->isGripping)
        {
            fsm->setState(PlaceDown::getInstance());
        }
        else
        {
            fsm->setState(PickUp::getInstance());
        }
    }
}

void Move::enter(FSM *fsm)
{
    cout << "Entered Move State" << endl;

    // Do something
    pair<coordinates, rotMatrix> nextPos = fsm->getNextPosition();
    if (!(fsm->moveTo(nextPos.first, nextPos.second)))
    {
        fsm->isError = true;
        cout << "Error moving to position: " << nextPos.first << endl;
    }

    return;
}

void Move::exit(FSM *fsm)
{
    cout << "Exited Move State" << endl
         << endl;
    // Do something
    return;
}

FSMState &Move::getInstance()
{
    static Move instance;
    return instance;
}

void PickUp::toggle(FSM *fsm)
{
    fsm->setState(Move::getInstance());
}

void PickUp::enter(FSM *fsm)
{
    cout << "Entered PickUp State" << endl;
    fsm->pickUp();
}

void PickUp::exit(FSM *fsm)
{
    // Do something
    cout << "Exited PickUp State" << endl
         << endl;
    return;
}

FSMState &PickUp::getInstance()
{
    static PickUp instance;
    return instance;
}

void PlaceDown::toggle(FSM *fsm)
{
    // exit already for assignment 1?
    fsm->setState(Wait::getInstance());
}

void PlaceDown::enter(FSM *fsm)
{
    cout << "Entered PlaceDown State" << endl;

    fsm->placeDown();
}

void PlaceDown::exit(FSM *fsm)
{
    // Do something

    cout << "Exited PlaceDown State" << endl
         << endl;
    return;
}

FSMState &PlaceDown::getInstance()
{
    static PlaceDown instance;
    return instance;
}
