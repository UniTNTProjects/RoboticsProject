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

    coordinates cord;
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
    float cords[][2] = {{0.1, 0.3}, {0.3, 0.3}, {0.5, 0.5}, {0.3, 0.25}, {0.2, 0.5}};
    // for each element in cords
    for (int i = 0; i < 5; i++)
    {
        cord << cords[i][0], cords[i][1], 0.;
        cord = translateBlockCordToRobotCord(cord);
        controller.move_to(cord, rotDefault, 20, false, false, false);
        cord(2) = 0.615;
        controller.move_to(cord, rotDefault, 20, true, false, false);
        // cord(2) = 0.50;
        // controller.move_to(cord, rotDefault, 20, true, false);
    }
}

coordinates translateBlockCordToRobotCord(coordinates blockCord)
{

    coordinates robotCord, robotReferenceCord;
    // robotReferenceCord << 0.525, 0.34, 1.5;
    robotReferenceCord << 0.475, 0.35, 1.5;

    robotCord << blockCord(0) - robotReferenceCord(0), robotReferenceCord(1) - blockCord(1), 0.50;
    cout << "blockCord: " << blockCord.transpose() << endl;
    cout << "robotCord: " << robotCord.transpose() << endl;
    return robotCord;
}
