#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int *sort_ik_result(const Eigen::Matrix<double, 8, 6> &ik_result, const jointValues &initial_joints)
{
    multimap<double, int> m;
    for (int i = 0; i < 8; i++)
    {
        jointValues comp;
        comp << ik_result(i, 0), ik_result(i, 1), ik_result(i, 2),
            ik_result(i, 3), ik_result(i, 4), ik_result(i, 5);
        m.insert(pair<double, int>((comp - initial_joints).norm(), i));
    }

    int *list = (int *)malloc(sizeof(int) * 8);
    int i = 0;
    for (auto const &it : m)
    {
        list[i++] = it.second;
    }
    return list;
}

int main()
{
    jointValues init_joint;
    init_joint << 0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0;
    coordinates position;
    position << 0.5, 0.5, 0.5;
    rotMatrix rotation;
    rotation << 0.0, 0.0, 1.0,
        0.0, -1.0, 0.0,
        1.0, 0.0, 0.0;
    Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(position, rotation);
    // print inverse_kinematics_res
    for (int i = 0; i < 8; i++)
    {
        cout << inverse_kinematics_res(i, 0) << " " << inverse_kinematics_res(i, 1) << " " << inverse_kinematics_res(i, 2) << " " << inverse_kinematics_res(i, 3) << " " << inverse_kinematics_res(i, 4) << " " << inverse_kinematics_res(i, 5) << endl;
    }

    int *indexes = sort_ik_result(inverse_kinematics_res, init_joint);
    // print indexes
    for (int i = 0; i < 8; i++)
    {
        cout << indexes[i] << endl;
    }
}
