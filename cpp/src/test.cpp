#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ur5");

    coordinates cord;
    rotMatrix rot;

    cord << 0.5,
        -0.3,
        0.50;
    rot << 0.940665, -0.225949, 0.253172,
        0.254041, 0.963543, -0.0839563,
        -0.224972, 0.143291, 0.963771;

    /*
     cord << 0.75, -0.2, 0.2;
     rot << 1, 0, -1,
         0, -1, 1,
         -1, -1, 0;
         */

    Controller controller;
    controller = Controller();

    controller.move_gripper_to(10);

    controller.move_to(cord, rot, 10);

    /*
     cout << "cord:" << cord << endl;
     cout << "rot:" << rot << endl;
     Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(cord, rot);

     cout << "inverse_kinematics_res:" << inverse_kinematics_res << endl;
     jointValues inverse;
     inverse << 0.872527, 2.8073, 0.439959, -1.39867, 2.90766, -0.532892;
     cout << "inverse:" << inverse << endl;
     coordinates cord2;
     rotMatrix rot2;
     ur5Direct(inverse, cord2, rot2);
     cout << "cord:" << cord2 << endl;
     cout << "rot:" << rot2 << endl;
     */
}
