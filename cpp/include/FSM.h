#pragma once
#include "FSMState.h"
#include <bits/stdc++.h>
#include <iostream>
#include "controller.h"
#include <queue>
#include <computer_vision/GetPoints.h>
#include <std_msgs/Bool.h>
using namespace std;

class FSMState;

class FSM
{
public:
    FSM();
    inline FSMState *getCurrentState() const { return currentState; }
    void toggle();
    void setState(FSMState &newState);

    void placeDown();
    void pickUp();
    bool moveTo(coordinates pos, rotMatrix rot, bool pick_or_place, bool homing, bool up_and_move_flag, bool move_to_near_axis_flag);
    bool moveToMultiple(vector<pair<coordinates, rotMatrix>> poses_rots, bool *pick_or_place, bool *homing, bool *up_and_move_flag, bool *move_to_near_axis_flag);
    void moveGripperTo(int diameter);
    bool isPositionQueueEmpty();
    void addPosition(coordinates pos, rotMatrix rot);
    void removePosition();
    pair<coordinates, rotMatrix> getNextPosition();
    coordinates translateBlockCordToRobotCord(coordinates blockCord);
    coordinates getCurrentPosition();

    bool isGripping = false;
    bool isError = false;
    bool isDone = false;

    double heightPickAndPlace = 0.14;

    int counter = 0;

    ros::ServiceClient get_ins;
    computer_vision::GetPoints srv_points;
    void setPermission(bool permission);

private:
    queue<pair<coordinates, rotMatrix>> *positions;
    Controller *controller;
    FSMState *currentState;

    int closeGripperDiameter = 28;
    int openGripperDiameter = 70;

    coordinates defaultCord;
    rotMatrix defaultRot;

    ros::NodeHandle node;
};