#include "ConcreteFSMState.h"

using namespace std;

void Init::toggle(FSM *fsm)
{
    cout << "Init toggle" << endl;
    fsm->setState(Wait::getInstance());
}

void Init::enter(FSM *fsm)
{
    cout << "\n/////////////////////////\nEntered Init State\n/////////////////////////\n"
         << endl;
}

void Init::exit(FSM *fsm)
{

    fsm->setPermission(true);
    cout << "Waiting for block" << endl;
    sleep(5);
    cout << "Hope the block is arrived" << endl;
    fsm->get_ins.call(fsm->srv_points);
    for (int i = 0; i < fsm->srv_points.response.instructions.size(); i++)
    {
        coordinates blockCord, silCord;
        rotMatrix rot;
        blockCord << fsm->srv_points.response.instructions[i].block.x,
            fsm->srv_points.response.instructions[i].block.y,
            fsm->srv_points.response.instructions[i].block.z;
        switch (fsm->srv_points.response.instructions[i].block.angle)
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

        silCord << fsm->srv_points.response.instructions[i].sil.x,
            fsm->srv_points.response.instructions[i].sil.y,
            fsm->srv_points.response.instructions[i].sil.z;
        coordinates blockCordRobot = fsm->translateBlockCordToRobotCord(blockCord);
        coordinates silCordRobot = fsm->translateBlockCordToRobotCord(silCord);
        blockCordRobot(2) = 0.86;
        silCordRobot(2) = 0.86;
        blockCordRobot(2) -= fsm->heightPickAndPlace;
        silCordRobot(2) -= fsm->heightPickAndPlace;
        fsm->addPosition(blockCordRobot, rot);
        fsm->addPosition(silCordRobot, rot);
    }

    fsm->setPermission(false);
    cout << "\n/////////////////////////\nExited Init State\n/////////////////////////\n"
         << endl
         << endl;
}

FSMState &Init::getInstance()
{
    static Init instance;
    return instance;
}

void Wait::toggle(FSM *fsm)
{
    if (fsm->isPositionQueueEmpty())
    {
        cout << "Queue is empty" << endl;
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
    // fix height
    // move up before pick
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
        fsm->isError = false;
        fsm->removePosition();
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
