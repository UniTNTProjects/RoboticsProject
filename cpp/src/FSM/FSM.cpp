#include "FSM.h"
#include "ConcreteFSMState.h"

using namespace std;

FSM::FSM()
{
    currentState = &Init::getInstance();
    controller = new Controller(250., true);
    positions = new queue<pair<coordinates, rotMatrix>>();

    get_ins = node.serviceClient<computer_vision::GetPoints>("computer_vision/Points");

    this->moveGripperTo(openGripperDiameter);
    this->setPermission(true);
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
    // robotReferenceCord << 0.525, 0.34, 1.5;
    robotReferenceCord << 0.5, 0.35, 1.5;

    robotCord << blockCord(0) - robotReferenceCord(0), robotReferenceCord(1) - blockCord(1), robotReferenceCord(2) - blockCord(2);

    return robotCord;
}

void FSM::addPosition(coordinates pos, rotMatrix rot)
{
    cout << "Adding position to queue" << endl;

    positions->push(make_pair(pos, rot));
    cout << "Queue done" << endl;
}

void FSM::removePosition()
{
    positions->pop();
}

bool FSM::isPositionQueueEmpty()
{
    return positions->empty();
}

bool FSM::moveTo(coordinates pos, rotMatrix rot, bool pick_or_place, bool homing, bool up_and_move_flag, bool move_to_near_axis_flag)
{
    return controller->move_to(pos, rot, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag);
}

bool FSM::moveToMultiple(vector<pair<coordinates, rotMatrix>> poses_rots, bool *pick_or_place, bool *homing, bool *up_and_move_flag, bool *move_to_near_axis_flag)
{
    return controller->move_to_multiple(poses_rots, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag);
}

coordinates FSM::getCurrentPosition()
{
    return controller->get_position().first;
}

void FSM::moveGripperTo(int diameter)
{
    controller->move_gripper_to(diameter);
}

void FSM::setPermission(bool permission_to_send)
{
    std_msgs::Bool permission;
    ros::Rate loop_rate(100);
    permission.data = permission_to_send;
    cout << "Sending permission" << endl;
    controller->permission_pub.publish(permission);
    loop_rate.sleep();
}
bool FSM::pickUp()
{
    pair<coordinates, rotMatrix> currentPos = getNextPosition();
    positions->pop();
    currentPos.first(2) += heightPickAndPlace;

    if (moveTo(currentPos.first, currentPos.second, true, false, false, false))
    {
        moveGripperTo(closeGripperDiameter);
        isGripping = true;
        currentPos.first(2) -= heightPickAndPlace;
        if (moveTo(currentPos.first, currentPos.second, true, false, false, false))
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

    if (moveTo(currentPos.first, currentPos.second, true, false, false, false))
    {
        moveGripperTo(openGripperDiameter);
        isGripping = false;
        currentPos.first(2) -= heightPickAndPlace - (0.01);
        if (moveTo(currentPos.first, currentPos.second, true, false, false, false))
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