#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ur5");

    coordinates cord;
    rotMatrix rot;

    cord << 0.602441,
        0.661121,
        0.069559;
    rot << 0.614737, -0.355196, -0.704226,
        0.42184, -0.606364, 0.674072,
        -0.666445, -0.711448, -0.222918;

    Controller controller;
    controller = Controller();

    controller.move_gripper_to(10);

    controller.move_to(cord, rot, 20);
}
