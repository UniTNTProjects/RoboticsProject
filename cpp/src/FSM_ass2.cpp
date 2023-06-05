#include "ConcreteFSMState.h"

using namespace std;

void Wait::toggle(FSM *fsm)
{

    if (fsm->counter++ < 4)
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
    cout << "\n/////////////////////////\nEntered Wait State\n/////////////////////////\n"
         << endl;
}

void Wait::exit(FSM *fsm)
{
    if (fsm->counter == 1)
    {
        rotMatrix rot;
        rot << -1, 0, 0,
            0, -1, 0,
            0, 0, 1;

        // Do something
        coordinates cord0, cord1, cord2, cord3, cord4, cord5, cord6, cord7;
        cord0 << 0.7, 0.55, 0.87;
        cord1 << 0.3, 0.5, 0.87;
        cord2 << 0.6, 0.2, 0.87;
        cord3 << 0.4, 0.25, 0.87;
        cord4 << 0.85, 0.6, 0.87;
        cord5 << 0.85, 0.4, 0.87;
        cord6 << 0.85, 0.2, 0.87;
        cord7 << 0.85, 0.7, 0.87;

        fsm->addPosition(fsm->translateBlockCordToRobotCord(cord0), rot);
        fsm->addPosition(fsm->translateBlockCordToRobotCord(cord4), rot);

        fsm->addPosition(fsm->translateBlockCordToRobotCord(cord1), rot);
        fsm->addPosition(fsm->translateBlockCordToRobotCord(cord5), rot);

        fsm->addPosition(fsm->translateBlockCordToRobotCord(cord2), rot);
        fsm->addPosition(fsm->translateBlockCordToRobotCord(cord6), rot);

        fsm->addPosition(fsm->translateBlockCordToRobotCord(cord3), rot);
        fsm->addPosition(fsm->translateBlockCordToRobotCord(cord7), rot);
    }
    /*
    coordinates a,b;
    a << 0.2, -0.2, 0.6;
    b << 0.3, -0.3, 0.6;


    fsm->addPosition(a, rot);
    fsm->addPosition(b, rot);
    */

    cout << "\n/////////////////////////\nExited Wait State\n/////////////////////////\n"
         << endl
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
    cout << "\n/////////////////////////\nEntered Move State\n/////////////////////////\n"
         << endl;

    // Do something
    pair<coordinates, rotMatrix> nextPos = fsm->getNextPosition();
    if (!(fsm->moveTo(nextPos.first, nextPos.second, false)))
    {
        coordinates newIntermidiatePos = fsm->getCurrentPosition();
        double new_z = newIntermidiatePos(2) - 0.25;
        if (new_z < 0.45)
        {
            new_z = 0.45;
        }
        newIntermidiatePos(2) = new_z;
        if (fsm->moveTo(newIntermidiatePos, nextPos.second, true))
        {
            coordinates secondIntermidiatePos = nextPos.first;
            secondIntermidiatePos(2) = new_z;
            if (fsm->moveTo(secondIntermidiatePos, nextPos.second, false))
            {
                if (!(fsm->moveTo(nextPos.first, nextPos.second, false)))
                {
                    fsm->isError = true;
                    cout << "\n!!!!!!!!\nError moving to position: " << nextPos.first << "\n!!!!!!!!!\n"
                         << endl;
                    return;
                }
            }
            else
            {
                fsm->isError = true;
                cout << "\n!!!!!!!!\nError moving to position: " << nextPos.first << "\n!!!!!!!!!\n"
                     << endl;
            }
        }
        else
        {
            fsm->isError = true;
            cout << "\n!!!!!!!!\nError moving to position: " << nextPos.first << "\n!!!!!!!!!\n"
                 << endl;
        }
    }

    return;
}

void Move::exit(FSM *fsm)
{
    cout << "\n/////////////////////////\nExited Move State\n/////////////////////////\n"
         << endl
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
    cout << "\n/////////////////////////\nEntered PickUp State\n/////////////////////////\n"
         << endl;
    if (!fsm->pickUp())
    {
        cout << "\n!!!!!!!!\nError picking up block\n!!!!!!!!!\n"
             << endl;
        fsm->isError = true;
    }
}

void PickUp::exit(FSM *fsm)
{
    // Do something
    cout << "\n/////////////////////////\nExited PickUp State\n/////////////////////////\n"
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
    cout << "\n/////////////////////////\nEntered PlaceDown State\n/////////////////////////\n"
         << endl;

    if (!fsm->placeDown())
    {
        cout << "\n!!!!!!!!\nError placing down block\n!!!!!!!!!\n"
             << endl;
        fsm->isError = true;
    }
}

void PlaceDown::exit(FSM *fsm)
{
    // Do something

    cout << "\n/////////////////////////\nExited PlaceDown State\n/////////////////////////\n"
         << endl;
    return;
}

FSMState &PlaceDown::getInstance()
{
    static PlaceDown instance;
    return instance;
}
