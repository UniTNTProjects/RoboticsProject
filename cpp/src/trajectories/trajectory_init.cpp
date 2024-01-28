#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "motion_trajectory.h"

using namespace Eigen;
using namespace std;

void ur5Trajectory(vector<double *> *Th, jointValues initial_position, jointValues final_position, int steps)
{

    Matrix<double, 6, 4> A;
    for (int i = 0; i < 6; i++)
    {
        Matrix<double, 4, 4> M;
        M << 1, 0, 0, 0,
            0, 1, 0, 0,
            1, 1, 1, 1,
            0, 1, 2, 3;

        Matrix<double, 4, 1> a, b;
        b << initial_position(i), 0, final_position(i), 0;

        a = M.inverse() * b;
        A.row(i) = a.transpose();
    }

    for (int j = 0; j < steps; j++)
    {
        for (int i = 0; i < 6; i++)
        {
            double t = (double)j / (double)steps;

            double q = A(i, 0) + A(i, 1) * t + A(i, 2) * t * t + A(i, 3) * t * t * t;
            Th->at(j)[i] = q;
        }
    }

    // cout << "Trajectory size: " << Th->size() << endl;
    // for (int i = 0; i < Th->size(); i++)
    // {
    //     cout << "Trajectory " << i << ": ";
    //     for (int j = 0; j < 6; j++)
    //     {
    //         cout << Th->at(i)[j] << " ";
    //     }
    //     cout << endl;
    // }
}

bool init_verify_trajectory(vector<double *> *Th, jointValues init_joint, jointValues final_joint, int steps, bool pick_or_place, const coordinates &requested_cord, const rotMatrix &requested_rotation, bool homing, bool side_pick, bool isGripping)
{
    ur5Trajectory(Th, init_joint, final_joint, steps);

    int *error_code = new int;
    *error_code = 0;
    bool valid = check_trajectory(*Th, steps, pick_or_place, error_code, requested_cord, requested_rotation, init_joint, homing, side_pick, isGripping);
    if (*error_code != 0)
    {
        if (error_code_debug)
        {
            cout << "error code: " << *error_code << " ";
        }
        // switch (*error_code)
        // {

        // case 1:
        //     cout << "crash" << endl;

        //     jointValues des_partial;
        //     des_partial << current_joints(0), current_joints(1), current_joints(2),
        //         5., current_joints(4), final_joint(5);
        //     if (move_to_joint(des_partial, steps, false, true))
        //     {
        //         des_partial(4) = 0.;
        //         if (move_to_joint(des_partial, steps, false, true))
        //         {
        //             return init_verify_trajectory(Th, des_partial, final_joint, steps, pick_or_place);
        //         }
        //     }

        //     break;
        // }
    }
    else
    {
        if (error_code_debug)
        {
            cout << "no error" << endl;
        }
    }

    return valid;
}
