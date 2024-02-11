#include "FSM.h"
#include "ConcreteFSMState.h"

using namespace std;

/**
 * @brief Construct a new FSM::FSM object
 * 
 */
FSM::FSM()
{
    currentState = &Init::getInstance();
    controller = new Controller(250., false);
    positions = new queue<pair<coordinates, rotMatrix>>();

    get_ins = node.serviceClient<computer_vision::GetInstructions>("computer_vision/Instructions");

    this->setPermission(true);
    this->moveGripperTo(openGripperDiameter);
    cout << "FSM initialized" << endl;
}

/**
 * @brief 
 * 
 */
void FSM::toggle()
{

    currentState->toggle(this);
}

/**
 * @brief 
 * 
 * @param newState 
 */
void FSM::setState(FSMState &newState)
{

    currentState->exit(this);

    currentState = &newState;

    currentState->enter(this);
}

/**
 * @brief 
 * 
 * @return pair<coordinates, rotMatrix> 
 */
pair<coordinates, rotMatrix> FSM::getNextPosition()
{
    pair<coordinates, rotMatrix> nextPos = positions->front();

    return nextPos;
}

/**
 * @brief 
 * 
 * @param blockCord 
 * @return coordinates 
 */
coordinates FSM::translateBlockCordToRobotCord(coordinates blockCord)
{

    coordinates robotCord, robotReferenceCord;
    // robotReferenceCord << 0.525, 0.34, 1.5;
    robotReferenceCord << 0.5, 0.35, 1.75;

    robotCord << blockCord(0) - robotReferenceCord(0), robotReferenceCord(1) - blockCord(1), robotReferenceCord(2) - blockCord(2);

    return robotCord;
}

/**
 * @brief 
 * 
 * @param pos 
 * @param rot 
 */
void FSM::addPosition(coordinates pos, rotMatrix rot)
{
    cout << "Adding position to queue" << endl;

    positions->push(make_pair(pos, rot));
    cout << "Queue done" << endl;
}

/**
 * @brief 
 * 
 */
void FSM::removePosition()
{
    positions->pop();
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool FSM::isPositionQueueEmpty()
{
    return positions->empty();
}

/**
 * @brief 
 * 
 * @param pos 
 * @param rot 
 * @param pick_or_place 
 * @param homing 
 * @param up_and_move_flag 
 * @param move_to_near_axis_flag 
 * @param side_pick_flag 
 * @return int 
 */
int FSM::moveTo(coordinates pos, rotMatrix rot, bool pick_or_place, bool homing, bool up_and_move_flag, bool move_to_near_axis_flag, bool side_pick_flag)
{
    return controller->move_to(pos, rot, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag, side_pick_flag);
}

/**
 * @brief 
 * 
 * @param poses_rots 
 * @param pick_or_place 
 * @param homing 
 * @param up_and_move_flag 
 * @param move_to_near_axis_flag 
 * @param side_picks_flag 
 * @return int 
 */
int FSM::moveToMultiple(vector<pair<coordinates, rotMatrix>> poses_rots, bool *pick_or_place, bool *homing, bool *up_and_move_flag, bool *move_to_near_axis_flag, bool *side_picks_flag)
{
    return controller->move_to_multiple(poses_rots, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag, side_picks_flag);
}

/**
 * @brief 
 * 
 * @return coordinates 
 */
coordinates FSM::getCurrentPosition()
{
    return controller->get_position().first;
}

/**
 * @brief 
 * 
 * @param diameter 
 */
void FSM::moveGripperTo(int diameter)
{
    controller->move_gripper_to(diameter);
}

/**
 * @brief 
 * 
 * @param permission_to_send 
 */
void FSM::setPermission(bool permission_to_send)
{
    std_msgs::Bool permission;
    ros::Rate loop_rate(100);
    permission.data = permission_to_send;
    cout << "Sending permission" << endl;
    controller->permission_pub.publish(permission);
    loop_rate.sleep();
}

/**
 * @brief 
 * 
 */
void FSM::pickUp()
{
    positions->pop();
    moveGripperTo(closeGripperDiameter);
    isGripping = true;
    controller->setGripping(true, this->isSidePick);
}

/**
 * @brief 
 * 
 */
void FSM::placeDown()
{

    positions->pop();
    moveGripperTo(openGripperDiameter);
    isGripping = false;
    controller->setGripping(false, false);
}

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{

    ros::init(argc, argv, "controller_ur5");

    FSM fsm;

    while (!fsm.isDone)
    {
        fsm.toggle();
    }
    cout << "Done" << endl;
    cout << "Time: " << fsm.objct_pick_time << endl;
    cout << "Recognition time: " << fsm.timer_rec_objct << endl;

    return 0;
}