#include "controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_ur5");

    Controller controller;

    while (true)
    {
        /* code */
        controller.move_gripper_to(10);
        controller.move_gripper_to(60);
    }
}
