#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ur5");

    coordinates cord0, cord1, cord2, cord3;
    rotMatrix rot;

    rot << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    cord0 << 0.5, -0.3, 0.5;
    cord1 << 0.1, -0.3, 0.5;
    cord2 << 0.5, -0.7, 0.5;
    cord3 << 0.5, -0.3, 0.7;

    //+z = down
    //+x = left
    //+y = backwards

    Controller controller;
    controller = Controller();

    // controller.move_gripper_to(10);

    controller.move_to(cord0, rot, 20);
    controller.move_to(cord1, rot, 20);
    controller.move_to(cord0, rot, 20);
    controller.move_to(cord2, rot, 20);
    controller.move_to(cord0, rot, 20);
    controller.move_to(cord3, rot, 20);
    controller.move_to(cord0, rot, 20);
}
