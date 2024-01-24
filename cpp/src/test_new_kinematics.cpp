#include "controller.h"
#include <computer_vision/GetPoints.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>

using namespace std;

coordinates translateBlockCordToRobotCord(coordinates blockCord);

double sum(vector<double> vec)
{
    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i];
    }
    return sum;
}

int main(int argc, char **argv)
{
    cout << "Init test_ur5" << endl;

    ros::init(argc, argv, "test_ur5");
    Controller controller = Controller(250., true);

    coordinates cord, cord2;
    cord << 0.2, 0.2, 0.5;
    rotMatrix rotDefault;
    rotDefault << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;

    // cord << 0.05, 0.05, 0.;
    // controller.move_to(translateBlockCordToRobotCord(cord), rotDefault, 20, false, false);
    // cord << 0.95, 0.05, 0.;
    // controller.move_to(translateBlockCordToRobotCord(cord), rotDefault, 20, false, false);
    // cord << 0.95, 0.75, 0.;
    // controller.move_to(translateBlockCordToRobotCord(cord), rotDefault, 20, false, false);
    // cord << 0.05, 0.75, 0.;
    // controller.move_to(translateBlockCordToRobotCord(cord), rotDefault, 20, false, false);
    float cords[][2] = {{0.3, 0.20}, {0.1, 0.3}, {0.3, 0.3}, {0.5, 0.5}, {0.2, 0.5}};
    // for each element in cords
    for (int i = 0; i < 5; i++)
    {
        cord << cords[i][0], cords[i][1], 0.;
        cord = translateBlockCordToRobotCord(cord);
        cord2 = cord;
        cord2(2) = 0.85;
        controller.move_to(cord, rotDefault, false, false, false, false);
        controller.move_to(cord2, rotDefault, true, false, false, false);
        controller.move_to(cord, rotDefault, true, false, false, false);
        // cord(2) = 0.50;
        // controller.move_to(cord, rotDefault, 20, true, false);
    }

    for (int i = 0; i < 5; i++)
    {
        cord << cords[i][0], cords[i][1], 0.;
        cord = translateBlockCordToRobotCord(cord);
        cord2 = cord;
        cord2(2) = 0.85;
        vector<pair<coordinates, rotMatrix>> poses_rots;
        poses_rots.push_back(make_pair(cord, rotDefault));
        poses_rots.push_back(make_pair(cord2, rotDefault));
        poses_rots.push_back(make_pair(cord, rotDefault));

        bool pick_or_place[] = {false, true, true};
        bool homing[] = {false, false, false};
        bool up_and_move_flag[] = {false, false, false};
        bool move_to_near_axis_flag[] = {false, false, false};

        controller.move_to_multiple(poses_rots, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag);
    }
}

coordinates translateBlockCordToRobotCord(coordinates blockCord)
{

    coordinates robotCord, robotReferenceCord;
    // robotReferenceCord << 0.525, 0.34, 1.5;
    robotReferenceCord << 0.5, 0.35, 1.5;

    robotCord << blockCord(0) - robotReferenceCord(0), robotReferenceCord(1) - blockCord(1), 0.72;
    cout << "blockCord: " << blockCord.transpose() << endl;
    cout << "robotCord: " << robotCord.transpose() << endl;
    return robotCord;
}
