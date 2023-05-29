#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ur5");

    Controller controller = Controller(1000.);

    // controller.move_gripper_to(10);

    /* MOVEMENT DEMO */

    /*
        +x = left
        +y = backwards
        +z = down
    */

    coordinates cord0, cord1, cord2, cord3, cord4, cord5;
    rotMatrix rot;

    rot << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    cord0 << 0.2, -0.2, 0.5;
    cord1 << -0.4, -0.2, 0.5;
    cord2 << 0.4, -0.2, 0.5;
    cord3 << 0.2, 0.3, 0.5;
    cord4 << 0.2, -0.3, 0.5;
    cord5 << 0.2, -0.2, 0.7;

    controller.move_to(cord0, rot, 20);
    controller.move_to(cord1, rot, 20);
    controller.move_to(cord2, rot, 20);
    controller.move_to(cord0, rot, 20);
    controller.move_to(cord3, rot, 20);
    controller.move_to(cord4, rot, 20);
    controller.move_to(cord0, rot, 20);
    controller.move_to(cord5, rot, 20);
    controller.move_to(cord0, rot, 20);
}
