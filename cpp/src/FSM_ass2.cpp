#include "ConcreteFSMState.h"

using namespace std;

void Wait::toggle(FSM *fsm)
{

    if (fsm->positions.size() > 10)
    {
        return;
    }
    fsm->setState(Move::getInstance());
}

void Wait::enter(FSM *fsm)
{
    // Do something
    cout << "Entered Wait State" << endl;
}

void Wait::exit(FSM *fsm)
{
    // Do something
    fsm->positions.push_back(make_tuple((double)fsm->currentPosIndex / 10, 0.0, 0.0));
    fsm->positions.push_back(make_tuple((double)fsm->currentPosIndex / 5, 0.0, 0.0));
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
    cout << "currentPosIndex: " << fsm->currentPosIndex << endl;

    if (fsm->currentPosIndex % 2 == 0)
    {
        // move to next position
        fsm->currentPosIndex++;

        cout << "Moving to position: " << get<0>(fsm->positions[fsm->currentPosIndex]) << ", " << get<1>(fsm->positions[fsm->currentPosIndex]) << ", " << get<2>(fsm->positions[fsm->currentPosIndex]) << endl;

        // if we have picked up an object already enter place down state
        fsm->setState(PlaceDown::getInstance());
    }
    else
    {
        // move to next position
        fsm->currentPosIndex++;

        cout << "Moving to position: " << get<0>(fsm->positions[fsm->currentPosIndex]) << ", " << get<1>(fsm->positions[fsm->currentPosIndex]) << ", " << get<2>(fsm->positions[fsm->currentPosIndex]) << endl;

        // if we dont have picked up an object already enter pick up state
        fsm->setState(PickUp::getInstance());
    }
}

void Move::enter(FSM *fsm)
{
    cout << "Entered Move State" << endl;
    // Do something
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

    fsm->controller.move_gripper_to(10);
    cout << "Picked up object at position: " << get<0>(fsm->positions[fsm->currentPosIndex]) << ", " << get<1>(fsm->positions[fsm->currentPosIndex]) << ", " << get<2>(fsm->positions[fsm->currentPosIndex]) << endl;
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

    // Do something
    fsm->controller.move_gripper_to(60);

    cout << "Placed down object at position: " << get<0>(fsm->positions[fsm->currentPosIndex]) << ", " << get<1>(fsm->positions[fsm->currentPosIndex]) << ", " << get<2>(fsm->positions[fsm->currentPosIndex]) << endl;
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
