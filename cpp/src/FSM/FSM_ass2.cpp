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
        // cout << "Queue is empty" << endl;
        // fsm->isDone = true;
        fsm->setState(Search::getInstance());
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
         << endl;
}

FSMState &Search::getInstance()
{
    static Search instance;
    return instance;
}

void Search::toggle(FSM *fsm)
{
    if (fsm->isPositionQueueEmpty())
    {
        fsm->isDone = true;
        fsm->setState(Wait::getInstance());
    }
    else
    {
        fsm->setState(Move::getInstance());
    }
}

void Search::enter(FSM *fsm)
{
    cout << "\n/////////////////////////\nEntered Search State\n/////////////////////////\n"
         << endl;
    // Do something
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

        int angle = fsm->srv_points.response.instructions[i].block.angle;
        double angle_rad = angle * M_PI / 180.0;

        rot << cos(angle_rad), -sin(angle_rad), 0,
            sin(angle_rad), cos(angle_rad), 0,
            0, 0, 1;

        // FIX TO DEFAULT ROT, TEST ONLY PURPOSE
        // rot << -1, 0, 0,
        //     0, -1, 0,
        //     0, 0, 1;

        // switch (fsm->srv_points.response.instructions[i].block.angle)
        // {
        // case 0:
        //     rot << -1, 0, 0,
        //         0, -1, 0,
        //         0, 0, 1;
        //     break;
        // case 45:
        //     rot << -0.707, -0.707, 0,
        //         0.707, -0.707, 0,
        //         0, 0, 1;
        //     break;
        // case 90:
        //     rot << 0, -1, 0,
        //         1, 0, 0,
        //         0, 0, 1;
        //     break;
        // }

        silCord
            << fsm->srv_points.response.instructions[i].sil.x,
            fsm->srv_points.response.instructions[i].sil.y,
            fsm->srv_points.response.instructions[i].sil.z;

        // cout << "Block cord (gazebo_ref): " << blockCord << endl;
        // cout << "Sil cord (gazebo_ref): " << silCord << endl;
        cout << "Block type: " << fsm->srv_points.response.instructions[i].type << endl;
        cout << "Angle: " << angle << endl;

        coordinates blockCordRobot = fsm->translateBlockCordToRobotCord(blockCord);
        coordinates silCordRobot = fsm->translateBlockCordToRobotCord(silCord);

        blockCordRobot(2) = 0.86;
        silCordRobot(2) = 0.86;

        cout << "Added position to queue" << endl;
        cout << "Block cord (robot_ref): " << blockCordRobot << endl;
        cout << "Sil cord (robot_ref): " << silCordRobot << endl;
        cout << "---------------------" << endl;
        fsm->addPosition(blockCordRobot, rot);
        fsm->addPosition(silCordRobot, rot);
    }

    return;
}

void Search::exit(FSM *fsm)
{
    // Do something
    cout << "\n/////////////////////////\nExited Search State\n/////////////////////////\n"
         << endl;
    fsm->setPermission(false);
    return;
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

    vector<pair<coordinates, rotMatrix>> poses_rots;

    coordinates pos = nextPos.first;
    pos(2) -= fsm->heightPickAndPlace;

    if (fsm->isGripping)
    {
        pos(2) -= 0.01;
    }

    poses_rots.push_back(make_pair(pos, nextPos.second));
    poses_rots.push_back(nextPos);

    bool pick_or_place[2] = {false, true};
    bool homing[2] = {false, false};
    bool up_and_move_flag[2] = {false, false};
    bool move_to_near_axis_flag[2] = {false, false};

    if (!(fsm->moveToMultiple(poses_rots, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag)))
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
    fsm->pickUp();
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

    fsm->placeDown();
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
