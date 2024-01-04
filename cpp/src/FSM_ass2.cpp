#include "ConcreteFSMState.h"

using namespace std;

void Wait::toggle(FSM *fsm)
{
    if (fsm->isPositionQueueEmpty())
    {
        fsm->isDone = true;
    }
    else
    {
        fsm->setState(Move::getInstance());
    }
}

void Wait::enter(FSM *fsm)
{
    // Do something
    cout << "\n/////////////////////////\nEntered Wait State\n/////////////////////////\n"
         << endl;
    fsm->setPermission(true);
    fsm->get_ins.call(fsm->srv_points);
    coordinates blockCord, silCord;
    while (fsm->srv_points.response.point.size() == 0)
    {
        fsm->get_ins.call(fsm->srv_points);
    }
    blockCord << fsm->srv_points.response.point[0].x, fsm->srv_points.response.point[0].y, fsm->srv_points.response.point[0].z;
    silCord << fsm->srv_points.response.point[1].x, fsm->srv_points.response.point[1].y, fsm->srv_points.response.point[1].z;
    cout << "Block coordinates: " << blockCord << endl;
    rotMatrix rot;

    fsm->setPermission(false);
    switch (fsm->srv_points.response.angle)
    {
    case 0:
        rot << -1, 0, 0,
            0, -1, 0,
            0, 0, 1;
        break;
    case 45:
        rot << -0.707, -0.707, 0,
            0.707, -0.707, 0,
            0, 0, 1;
        break;
    case 90:
        rot << 0, -1, 0,
            1, 0, 0,
            0, 0, 1;
        break;
    }

    cout << "Block coordinates: " << blockCord << endl;
    cout << "Sil coordinates: " << silCord << endl;
    cout << "Rot:" << rot << endl;

    if (blockCord(0) == 0 && blockCord(1) == 0 && blockCord(2) == 0)
    {
        return;
    }

    fsm->addPosition(fsm->translateBlockCordToRobotCord(blockCord), rot);
    fsm->addPosition(fsm->translateBlockCordToRobotCord(silCord), rot);
}

void Wait::exit(FSM *fsm)
{
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

        fsm->isError = true;
        cout << "\n!!!!!!!!\nError moving to position: " << nextPos.first << "\n!!!!!!!!!\n"
             << endl;
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
