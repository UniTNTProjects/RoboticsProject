#include "FSM.h"
#include "ConcreteFSMState.h"

using namespace std;

FSM::FSM()
{
    currentState = &Wait::getInstance();
    controller = new Controller(250., true);
    positions = new queue<pair<coordinates, rotMatrix>>();

    get_ins = node.serviceClient<computer_vision::GetPoints>("computer_vision/Points");

    this->moveGripperTo(openGripperDiameter);
    cout << "FSM initialized" << endl;
}

void FSM::toggle()
{

    currentState->toggle(this);
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

    return nextPos;
}

coordinates FSM::translateBlockCordToRobotCord(coordinates blockCord)
{

    coordinates robotCord, robotReferenceCord;
    robotReferenceCord << 0.5, 0.35, 1.46;
    robotCord << blockCord(0) - robotReferenceCord(0), robotReferenceCord(1) - blockCord(1), robotReferenceCord(2) - blockCord(2);

    return robotCord;
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

bool FSM::moveTo(coordinates pos, rotMatrix rot, bool pick_or_place)
{
    return controller->move_to(pos, rot, this->controller->steps, pick_or_place, false);
}

coordinates FSM::getCurrentPosition()
{
    return controller->get_position().first;
}

void FSM::moveGripperTo(int diameter)
{
    controller->move_gripper_to(diameter);
}

bool FSM::pickUp()
{
    pair<coordinates, rotMatrix> currentPos = getNextPosition();
    positions->pop();
    currentPos.first(2) += heightPickAndPlace;

    if (moveTo(currentPos.first, currentPos.second, true))
    {
        moveGripperTo(closeGripperDiameter);
        isGripping = true;
        currentPos.first(2) -= heightPickAndPlace;
        if (moveTo(currentPos.first, currentPos.second, true))
        {
            return true;
        }
    }

    return false;
}

bool FSM::placeDown()
{
    pair<coordinates, rotMatrix> currentPos = getNextPosition();
    positions->pop();
    currentPos.first(2) += heightPickAndPlace - (0.01);

    if (moveTo(currentPos.first, currentPos.second, true))
    {
        moveGripperTo(openGripperDiameter);
        isGripping = false;
        currentPos.first(2) -= heightPickAndPlace - (0.01);
        if (moveTo(currentPos.first, currentPos.second, true))
        {
            return true;
        }
    }

    return false;
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