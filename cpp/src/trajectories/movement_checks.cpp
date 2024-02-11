#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "motion_trajectory.h"

bool check_singularity_collision(jointValues joints, bool side_pick)
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

        // cout << "joint[" << i << "]= (x: " << x_coord << ", y: " << y_coord << ", z: " << z_coord << ")" << endl;
        if (z_coord < min_z or z_coord > max_z or y_coord > max_y)
        {
            if (debug_traj)
            {
                cout << "joint[" << i << "] out of bounds" << endl;
                cout << "joint[" << i << "]= (x: " << x_coord << ", y: " << y_coord << ", z: " << z_coord << ")" << endl;
                cout << "min_z: " << min_z << ", max_z: " << max_z << ", max_y: " << max_y << endl;
            }
            return true;
        }
        if (side_pick)
        {
            if (z_coord > max_z_sidepick or z_coord < min_z or y_coord > max_y)
            {
                if (debug_traj)
                {
                    cout << "Side pick:" << endl;
                    cout << "joint[" << i << "] out of bounds" << endl;
                    cout << "joint[" << i << "]= (x: " << x_coord << ", y: " << y_coord << ", z: " << z_coord << ")" << endl;
                }
                return true;
            }
        }
    }
    // cout << endl;
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
            if (!side_pick && (cord - requested_cord).norm() > 0.1)
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
            if ((rot - requested_rotation).norm() > 0.2)
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
            if (side_pick && pick_or_place)
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

                mat = mat * matrixes[0];
                mat = mat * matrixes[1];
                mat = mat * matrixes[2];
                mat = mat * matrixes[3];
                float z3_coord = mat(2, 3);
                mat = mat * matrixes[4];
                mat = mat * matrixes[5];
                float z5_coord = mat(2, 3);

                if (z3_coord > z5_coord)
                {
                    if (debug_traj)
                    {
                        cout << "*******" << endl;
                        cout << "z3_coord: " << z3_coord << ", z5_coord: " << z5_coord << endl;
                    }
                    *error_code = 16;
                    return false;
                }
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

        if (check_singularity_collision(joints, side_pick))
        {
            *error_code = 19;
            return false;
        }

        if ((joints(3) < -3.2 || (joints(3) > 0 && joints(3) < 3.1)) && ((joints(4) < -2 && joints(4) > -4) || (joints(4) > 2 && joints(4) < 5)))
        {
            if (debug_traj)
            {
                cout << "*******" << endl;
                cout << "type 1 collision with itself" << endl;
            }
            *error_code = 1;
            return false;
        }

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

        if (!side_pick && !pick_or_place && !homing && cord(2) > max_z_moving)
        {
            if (debug_traj)
            {
                cout << "*******" << endl;
                cout << "moving too low, z wrong: " << cord.transpose() << endl;
            }
            *error_code = 5;
            return false;
        }

        if (side_pick)
        {
            if (!pick_or_place && cord(2) > max_z_moving_sidepick)
            {
                if (debug_traj)
                {
                    cout << "******" << endl;
                    cout << "moving too low sidepicking, z wrong: " << cord.transpose() << endl;
                }
                *error_code = 52;
                return false;
            }

            if (pick_or_place && cord(2) > max_z_sidepick)
            {
                if (debug_traj)
                {
                    cout << "*******" << endl;
                    cout << "moving too low sidepicking, z wrong: " << cord.transpose() << endl;
                }
                *error_code = 53;
                return false;
            }

            if (!pick_or_place && isGripping && cord(2) > max_z_moving_gripping_sidepick)
            {
                if (debug_traj)
                {
                    cout << "*******" << endl;
                    cout << "moving too low gripping sidepicking, z wrong: " << cord.transpose() << endl;
                }
                *error_code = 54;
                return false;
            }
        }

        if (pick_or_place && (norm_angle(joints(5)) < M_PI_4 || norm_angle(joints(5)) > 7 * M_PI_4) && cord(1) > max_y_near_end_table && cord(2) > max_z_near_end_table)
        {
            if (debug_traj)
            {
                cout << "*******" << endl;
                cout << "Moving too near end table with gripper facing wall" << cord.transpose() << endl;
            }
            *error_code = 23;
            return false;
        }

        if (!pick_or_place && cord(2) > max_z_near_end_table && cord(1) > max_y_near_end_table)
        {
            if (debug_traj)
            {
                cout << "*******" << endl;
                cout << "cord end table wrong: " << cord.transpose() << endl;
            }
            *error_code = 6;
            return false;
        }

        if (pick_or_place && (abs(start_cord(0) - cord(0))) > 0.2 && (abs(start_cord(1) - cord(1))) > 0.2)
        {
            if (debug_traj)
            {
                cout << "*******" << endl;
                cout << "cords x and y are different from start" << endl;
                cout << "start_cord: " << start_cord.transpose() << endl;
                cout << "cord: " << cord.transpose() << endl;
            }
            *error_code = 30;
            return false;
        }

        if (pick_or_place && abs(init_joint(4) - joints(4)) > 0.2)
        {
            if (debug_traj)
            {
                cout << "*******" << endl;
                cout << "rotations are different" << endl;
                cout << "start_rot: " << start_rotation << endl;
                cout << "rot: " << rot << endl;
            }
            *error_code = 8;
            return false;
        }

        if (joints.norm() == 0)
        {
            continue;
        }

        MatrixXd jacobian = ur5Jac(joints);
        // cout << "jacobian: " << jacobian << endl;

        if (joints(1) > 0.01 || joints(1) < -3.14)
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
