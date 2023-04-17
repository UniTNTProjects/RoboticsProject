#pragma once
#include "FSMState.h"
#include "FSM.h"
#include <iostream>

class Wait : public FSMState
{
public:
    void enter(FSM *fsm);
    void toggle(FSM *fsm);
    void exit(FSM *fsm);
    static FSMState &getInstance();

private:
    Wait() {}
    Wait(const Wait &other);
    Wait &operator=(const Wait &other);
};

class Move : public FSMState
{
public:
    void enter(FSM *fsm);
    void toggle(FSM *fsm);
    void exit(FSM *fsm);
    static FSMState &getInstance();

private:
    Move() {}
    Move(const Move &other);
    Move &operator=(const Move &other);
};

class PickUp : public FSMState
{
public:
    void enter(FSM *fsm);
    void toggle(FSM *fsm);
    void exit(FSM *fsm);
    static FSMState &getInstance();

private:
    PickUp() {}
    PickUp(const PickUp &other);
    PickUp &operator=(const PickUp &other);
};

class PlaceDown : public FSMState
{
public:
    void enter(FSM *fsm);
    void toggle(FSM *fsm);
    void exit(FSM *fsm);
    static FSMState &getInstance();

private:
    PlaceDown() {}
    PlaceDown(const PlaceDown &other);
    PlaceDown &operator=(const PlaceDown &other);
};
