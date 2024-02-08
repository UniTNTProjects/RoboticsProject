#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "motion_trajectory.h"

bool check_singularity_collision(jointValues joints)
{

    homoMatrix mat;
    mat << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    homoMatrix matrixes[6];
    matrixes[0] = T10f(joints(0));
    matrixes[1] = T21f(joints(1));
    matrixes[2] = T32f(joints(2));
    matrixes[3] = T43f(joints(3));
    matrixes[4] = T54f(joints(4));
    matrixes[5] = T65f(joints(5));

    for (int i = 0; i < 6; i++)
    {

        mat = mat * matrixes[i];
        float x_coord = mat(0, 3);
        float y_coord = mat(1, 3);
        float z_coord = mat(2, 3);

        if (z_coord < min_z or z_coord > max_z or y_coord > max_y)
        {
            if (debug_traj)
            {
                cout << "joint[" << i << "] out of bounds" << endl;
                cout << "joint[" << i << "]= (x: " << x_coord << ", y: " << y_coord << ", z: " << z_coord << ")" << endl;
            }
            return true;
        }
    }
    // cout << endl;
    return false;
}

bool check_self_collision(jointValues Th)
{
    homoMatrix mat;
    mat << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    homoMatrix matrixes[6];
    matrixes[0] = T10f(Th(0));
    matrixes[1] = T21f(Th(1));
    matrixes[2] = T32f(Th(2));
    matrixes[3] = T43f(Th(3));
    matrixes[4] = T54f(Th(4));
    matrixes[5] = T65f(Th(5));

    vector<coordinates> cords;
    for (int i = 0; i < 6; i++)
    {
        mat = mat * matrixes[i];
        float x_coord = mat(0, 3);
        float y_coord = mat(1, 3);
        float z_coord = mat(2, 3);
        coordinates cord;
        cord << x_coord, y_coord, z_coord;
        cords.push_back(cord);
    }

    // Check if gripper intersect with any joint

    coordinates gripper;
    gripper << cords[5](0), cords[5](1), cords[5](2);

    // add end effector to coordinates[5]

    for (int i = 0; i < 4; i++)
    {
        // Check if intersect the space between 2 joints

        if (gripper(0) < cords[i](0) && gripper(0) > cords[i + 1](0))
        {
            if (gripper(1) < cords[i](1) && gripper(1) > cords[i + 1](1))
            {
                if (gripper(2) < cords[i](2) && gripper(2) > cords[i + 1](2))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool check_trajectory(vector<double *> traj, int step, bool pick_or_place, int *error_code, const coordinates &requested_cord, const rotMatrix &requested_rotation, const jointValues &init_joint, bool homing, bool side_pick, bool isGripping)
{
    coordinates start_cord;
    rotMatrix start_rotation;
    ur5Direct(init_joint, start_cord, start_rotation);

    // cout << "check trajectory" << endl;
    for (int i = 0; i < step; i++)
    {
        jointValues joints;
        coordinates cord;
        rotMatrix rot;
        joints << traj[i][0], traj[i][1], traj[i][2],
            traj[i][3], traj[i][4], traj[i][5];

        ur5Direct(joints, cord, rot);
        if (debug_traj && i == step - 1)
        {

            cout << "last cord: " << cord.transpose() << endl;
            cout << "last joint: " << joints.transpose() << endl;
            cout << "last rotation: " << endl
                 << rot << endl
                 << endl;
        }

        if (check_self_collision(joints))
        {
            *error_code = 42;
            return false;
        }
        if (i == 0)
        {
            if ((cord - start_cord).norm() > 0.1)
            {
                if (debug_traj)
                {
                    cout << "*******" << endl;
                    cout << "first cord wrong" << endl;
                    cout << "cord: " << cord.transpose() << endl;
                    cout << "start_cord: " << start_cord.transpose() << endl;
                }
                *error_code = 12;
                return false;
            }
            if ((rot - start_rotation).norm() > 0.1)
            {
                if (debug_traj)
                {
                    cout << "*******" << endl;
                    cout << "first rotation wrong" << endl;
                    cout << "rot: " << rot << endl;
                    cout << "start_rotation: " << start_rotation << endl;
                }
                *error_code = 13;
                return false;
            }
        }

        if (i == step - 1)
        {
            if ((cord - requested_cord).norm() > 0.1)
            {
                if (debug_traj)
                {
                    cout << "*******" << endl;
                    cout << "last cord wrong" << endl;
                    cout << "cord: " << cord.transpose() << endl;
                    cout << "requested_cord: " << requested_cord.transpose() << endl;
                }
                *error_code = 14;
                return false;
            }
            if ((rot - requested_rotation).norm() > 0.1)
            {
                if (debug_traj)
                {
                    cout << "*******" << endl;
                    cout << "last rotation wrong" << endl;
                    cout << "rot: " << rot << endl;
                    cout << "requested_rotation: " << requested_rotation << endl;
                }
                *error_code = 15;
                return false;
            }
        }

        for (int i = 0; i < 6; i++)
        {
            if (isnan(joints(i)))
            {
                if (debug_traj)
                {
                    cout << "*******" << endl;
                    cout << "trajectory invalid 1" << endl;
                }
                *error_code = 21;
                return false;
            }
        }

        if (check_singularity_collision(joints))
        {
            *error_code = 19;
            if (side_pick)
                return true;
            return false;
        }

        // if ((joints(3) < -3.2 || (joints(3) > 0 && joints(3) < 3.1)) && ((joints(4) < -2 && joints(4) > -4) || (joints(4) > 2 && joints(4) < 5)))
        // {
        //     if (debug_traj)
        //     {
        //         cout << "*******" << endl;
        //         cout << "type 1 collision with itself" << endl;
        //     }
        //     *error_code = 1;
        //     return false;
        // }

        if (joints(2) > 2.7 || joints(2) < -2.7)
        {
            if (debug_traj)
            {
                cout << "*******" << endl;
                cout << "type 2 collision with itself" << endl;
            }
            *error_code = 2;
            return false;
        }

        // if (!side_pick && !pick_or_place && !homing && cord(2) > max_z_moving)
        // {
        //     if (debug_traj)
        //     {
        //         cout << "*******" << endl;
        //         cout << "moving too low, z wrong: " << cord.transpose() << endl;
        //     }
        //     *error_code = 5;
        //     return false;
        // }

        // if (!pick_or_place && (joints(5) < M_PI_4 || joints(5) > 7 * M_PI_4) && cord(1) > max_y_near_end_table)
        // {
        //     if (debug_traj)
        //     {
        //         cout << "*******" << endl;
        //         cout << "moving too low, z wrong: " << cord.transpose() << endl;
        //     }
        //     *error_code = 23;
        //     return false;
        // }

        // if (!pick_or_place && cord(2) > max_z_near_end_table && cord(1) > max_y_near_end_table)
        // {
        //     if (debug_traj)
        //     {
        //         cout << "*******" << endl;
        //         cout << "cord end table wrong: " << cord.transpose() << endl;
        //     }
        //     *error_code = 6;
        //     return false;
        // }

        if (pick_or_place && (fabs(traj[0][4] - joints(4)) > M_PI / 2 || fabs(traj[0][5] - joints(5)) > M_PI / 2))
        {
            if (debug_traj)
            {
                cout << "*******" << endl;
                cout << "trying strange rotation for pick or place" << endl;
                cout << "traj[0][4]: " << traj[0][4] << ", traj[0][5]: " << traj[0][5] << endl;
                cout << "joints(4): " << joints(4) << ", joints(5): " << joints(5) << endl;
                cout << fabs(traj[0][4] - joints(4)) << ", " << fabs(traj[0][5] - joints(5)) << endl;
            }
            *error_code = 7;
            return false;
        }

        // if (pick_or_place && (start_rotation - rot).norm() > 0.2)
        // {
        //     if (debug_traj)
        //     {
        //         cout << "*******" << endl;
        //         cout << "rotations are different" << endl;
        //         cout << "start_rot: " << start_rotation << endl;
        //         cout << "rot: " << rot << endl;
        //     }
        //     *error_code = 8;
        //     return false;
        // }

        if (joints.norm() == 0)
        {
            continue;
        }

        MatrixXd jacobian = ur5Jac(joints);
        // cout << "jacobian: " << jacobian << endl;

        if (joints(1) > 0.1 || joints(1) < -3.14)
        {
            if (debug_traj)
            {
                cout << "*******" << endl;
                cout << "joints(1) invalid:" << joints(1) << endl;
            }
            *error_code = 9;
            return false;
        }

        if (abs(jacobian.determinant()) < 0.00000001)
        {
            if (debug_traj)
            {
                cout << "*******" << endl;
                cout << "determinant of jacobian is " << abs(jacobian.determinant()) << endl;
            }
            *error_code = 10;
            return false;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (abs(svd.singularValues()(5)) < 0.0000001)
        {
            if (debug_traj)
            {
                cout << "*******" << endl;
                cout << "trajectory invalid 2" << endl;
            }
            cout << abs(svd.singularValues()(5)) << endl;
            *error_code = 11;
            return false;
        }
    }
    // cout << "trajectory valid" << endl;

    return true;
}