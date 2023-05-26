#include "FSM.h"
#include "ConcreteFSMState.h"

using namespace std;

FSM::FSM()
{
    currentState = &Wait::getInstance();
    controller = new Controller(1000.);
    positions = new queue<pair<coordinates, rotMatrix>>();
    this->moveGripperTo(60);
    cout << "FSM initialized" << endl;
}

void FSM::setState(FSMState &newState)
{

    currentState->exit(this);

    currentState = &newState;

    currentState->enter(this);
}

pair<coordinates, rotMatrix> FSM::getNextPosition()
{
    pair<coordinates, rotMatrix> nextPos = positions->front();
    positions->pop();

    return nextPos;
}

void FSM::addPosition(coordinates pos, rotMatrix rot)
{
    cout << "Adding position to queue" << endl;

    positions->push(make_pair(pos, rot));
    cout << "Queue done" << endl;
}

bool FSM::isPositionQueueEmpty()
{
    return positions->empty();
}

bool FSM::moveTo(coordinates pos, rotMatrix rot)
{
    return controller->move_to(pos, rot, 20);
}

void FSM::moveGripperTo(int diameter)
{
    controller->move_gripper_to(diameter);
}

bool FSM::pickUp()
{
    pair<coordinates, rotMatrix> currentPos = controller->get_position();
    currentPos.first(2) += 0.1;

    if (moveTo(currentPos.first, currentPos.second))
    {
        moveGripperTo(20);
        isGripping = true;
        currentPos.first(2) -= 0.1;
        if (moveTo(currentPos.first, currentPos.second))
        {
            return true;
        }
    }

    return false;
}

bool FSM::placeDown()
{
    pair<coordinates, rotMatrix> currentPos = controller->get_position();
    currentPos.first(2) += 0.1;

    if (moveTo(currentPos.first, currentPos.second))
    {
        moveGripperTo(60);
        isGripping = false;
        currentPos.first(2) -= 0.1;
        if (moveTo(currentPos.first, currentPos.second))
        {
            return true;
        }
    }

    return false;
}

void FSM::toggle()
{

    currentState->toggle(this);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "controller_ur5");

    FSM fsm;

    while (!fsm.isDone)
    {
        fsm.toggle();
    }

    return 0;
}