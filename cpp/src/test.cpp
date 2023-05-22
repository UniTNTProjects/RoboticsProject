#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ur5");

    coordinates cord;
    rotMatrix rot;

    cord << 0.75,
        -0.25,
        0.05;
    rot << 1.0, 0.0, 0.0,
        0.0, 0.0, -1.0,
        0.0, 1.0, 0.0;

    Controller controller;
    controller = Controller();

    // controller.move_gripper_to(10);

    controller.move_to(cord, rot, 2);
}
