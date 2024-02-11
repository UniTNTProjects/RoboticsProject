#include "ConcreteFSMState.h"
#include "eigen3/Eigen/Geometry"

using namespace std;
using namespace Eigen;

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
    coordinates defaultCordArray = {0.35, 0.03, 0.7};
    rotMatrix rotDefault;
    rotDefault << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;
    double initial_timer = ros::Time::now().toSec();
    fsm->setPermission(true);
    fsm->moveTo(defaultCordArray, rotDefault, false, false, false, false, false);
    fsm->setPermission(false);

    for (int i = 0; i < fsm->srv_points.response.instructions.size(); i++)
    {
        coordinates blockCord, silCord;
        rotMatrix rot;
        rotMatrix rot_default;
        blockCord << fsm->srv_points.response.instructions[i].block.x,
            fsm->srv_points.response.instructions[i].block.y,
            fsm->srv_points.response.instructions[i].block.z - 0.055;

        int angle = fsm->srv_points.response.instructions[i].block.angle;
        double angle_rad = angle * M_PI / 180.0;

        Vector3d block_angles = {0, 0, angle_rad};
        // Quaternion
        Quaterniond q = AngleAxisd(block_angles(0), Vector3d::UnitX()) *
                        AngleAxisd(block_angles(1), Vector3d::UnitY()) *
                        AngleAxisd(block_angles(2), Vector3d::UnitZ());

        coordinates side_pos;
        side_pos << blockCord(0), blockCord(1), blockCord(2);

        rot << cos(angle_rad), -sin(angle_rad), 0,
            sin(angle_rad), cos(angle_rad), 0,
            0, 0, 1;

        rot_default
            << -1,
            0, 0,
            0, -1, 0,
            0, 0, 1;

        silCord
            << fsm->srv_points.response.instructions[i].sil.x,
            fsm->srv_points.response.instructions[i].sil.y,
            fsm->srv_points.response.instructions[i].sil.z;

        cout << "Block type: " << fsm->srv_points.response.instructions[i].type << endl;
        cout << "Angle: " << angle_rad * 180 / M_PI << endl;

        coordinates blockCordRobot = fsm->translateBlockCordToRobotCord(side_pos);
        coordinates silCordRobot = fsm->translateBlockCordToRobotCord(silCord);

        silCordRobot(2) = 0.84;

        cout << "Added position to queue" << endl;
        cout << "Block cord (robot_ref): " << blockCordRobot << endl;
        cout << "Sil cord (robot_ref): " << silCordRobot << endl;
        cout << "---------------------" << endl;

        fsm->addPosition(blockCordRobot, rot);
        fsm->addPosition(silCordRobot, rot_default);
    }

    fsm->timer_rec_objct = ros::Time::now().toSec() - initial_timer;

    cout << "Time to receive object: " << fsm->timer_rec_objct << endl;
    fsm->start_objct_pick = ros::Time::now().toSec();

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
    if (fsm->init)
    {
        coordinates defaultCordArray = {0.35, 0.03, 0.7};
        rotMatrix rotDefault;
        rotDefault << -1, 0, 0,
            0, -1, 0,
            0, 0, 1;
        fsm->moveTo(defaultCordArray, rotDefault, false, false, false, false, false);
    }
    else
    {
        fsm->init = true;
    }
    // Do something
    fsm->setPermission(true);
    cout << "Waiting for block" << endl;
    sleep(5);
    fsm->get_ins.call(fsm->srv_points);
    for (int i = 0; i < fsm->srv_points.response.instructions.size(); i++)
    {
        coordinates blockCord, silCord;
        rotMatrix rot;
        rotMatrix rot_default;
        blockCord << fsm->srv_points.response.instructions[i].block.x,
            fsm->srv_points.response.instructions[i].block.y,
            fsm->srv_points.response.instructions[i].block.z;

        int angle = fsm->srv_points.response.instructions[i].block.angle;
        double angle_rad = angle * M_PI / 180.0;

        Vector3d block_angles = {0, 0, angle_rad};
        // Quaternion
        Quaterniond q = AngleAxisd(block_angles(0), Vector3d::UnitX()) *
                        AngleAxisd(block_angles(1), Vector3d::UnitY()) *
                        AngleAxisd(block_angles(2), Vector3d::UnitZ());

        coordinates side_pos;
        side_pos << blockCord(0), blockCord(1), blockCord(2);

        rot << cos(angle_rad), -sin(angle_rad), 0,
            sin(angle_rad), cos(angle_rad), 0,
            0, 0, 1;

        // FIX TO DEFAULT ROT, TEST ONLY PURPOSE
        rot_default
            << -1,
            0, 0,
            0, -1, 0,
            0, 0, 1;

        silCord
            << fsm->srv_points.response.instructions[i].sil.x,
            fsm->srv_points.response.instructions[i].sil.y,
            fsm->srv_points.response.instructions[i].sil.z;

        cout << "Block type: " << fsm->srv_points.response.instructions[i].type << endl;
        cout << "Angle: " << angle_rad * 180 / M_PI << endl;

        coordinates blockCordRobot = fsm->translateBlockCordToRobotCord(side_pos);
        coordinates silCordRobot = fsm->translateBlockCordToRobotCord(silCord);

        silCordRobot(2) = 0.84;

        cout << "Added position to queue" << endl;
        cout << "Block cord (robot_ref): " << blockCordRobot << endl;
        cout << "Sil cord (robot_ref): " << silCordRobot << endl;
        cout << "---------------------" << endl;
        fsm->addPosition(blockCordRobot, rot);
        fsm->addPosition(silCordRobot, rot_default);
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
        if (!fsm->isGripping)
        {
            fsm->removePosition();
            fsm->removePosition();
        }
        else
        {
            fsm->placeDown();
        }
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
    bool side_pick_flag[2] = {false, false};

    fsm->isSidePick = false;
    switch (fsm->moveToMultiple(poses_rots, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag, side_pick_flag))
    {
    case 0:
        fsm->isError = true;
        cout << "\n!!!!!!!!\nError moving to position: " << nextPos.first << "\n!!!!!!!!!\n"
             << endl;
        break;
    case 1:
        cout << "\n!!!!!!!!\nMoved to position: " << nextPos.first << "\n!!!!!!!!!\n"
             << endl;
        break;
    case 2:
        cout << "\n!!!!!!!!\nMoved to position with side pick: " << nextPos.first << "\n!!!!!!!!!\n"
             << endl;
        fsm->isSidePick = true;
        break;
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
    fsm->end_objct_pick = ros::Time::now().toSec();
    fsm->objct_pick_time = fsm->end_objct_pick - fsm->start_objct_pick;
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
