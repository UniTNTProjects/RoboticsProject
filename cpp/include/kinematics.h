#include <cmath>
#include <eigenMatrices.h>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

// Direct Kineamtics of UR5
void ur5Direct(const jointValues &th, coordinates &pe, rotMatrix &re);

Matrix4d computeEndEffectorPose(const coordinates &pe, const rotMatrix &re);

jointValues ur5InverseKinematicsGPT(const coordinates &pe, const rotMatrix &re);

// Inverse Kineamtics of UR5
Matrix<double, 8, 6> ur5Inverse(coordinates pe, rotMatrix re);
int *sort_inverse(Eigen::Matrix<double, 8, 6> &inverse_kinematics_res, const jointValues &initial_joints);

Matrix<double, 8, 6> ur5InverseTestJac(coordinates, rotMatrix);
// Inverse Kineamtics of UR5 - Pinocchio CLIK
jointValues ur5InversePinocchio(coordinates pe, rotMatrix re, jointValues current_joints);
// Jacobian of UR5
Matrix<double, 6, 6> ur5Jac(jointValues &Th);

static double norm_angle(double angle)
{
    if (angle > 0)
    {

        return fmod(angle, 2 * M_PI);
    }
    else
    {

        return 2 * M_PI - fmod(-angle, 2 * M_PI);
    }
}

static jointValues bestNormalization(const jointValues init, jointValues final)
{

    jointValues res;
    for (int i = 0; i < 6; i++)
    {
        if (i == 2 || i == 1)
        {
            // no need to normalize this joints, they are not in the range of [-pi,pi] or could cause problems if normalized
            res(i) = final(i);
        }
        else
        {

            int final_factor = final(i) / (2 * M_PI);
            int init_factor = init(i) / (2 * M_PI);

            final(i) = final(i) - (final_factor - init_factor) * 2 * M_PI;

            double diff = final(i) - init(i);
            if (diff > M_PI)
            {
                res(i) = final(i) - 2 * M_PI;
            }
            else if (diff < -M_PI)
            {
                res(i) = final(i) + 2 * M_PI;
            }
            else
            {
                res(i) = final(i);
            }
        }
    }

    // cout << "bestNormalization: " << res.transpose() << endl;
    // cout << "init: " << init.transpose() << endl;
    // cout << "final: " << final.transpose() << endl;
    // cout << "-------------------" << endl;

    return res;
}

// fixes eventual problems with the normalization of the joints, due to the limits of the rotations of the joints
// if joint A can move between -pi and pi and your value is pi+0.1, it will be normalized to -pi+0.1
// not optimal but necessary
static jointValues fixNormalization(jointValues joints)
{
    // joint 0 between -6.14 and 6.14
    if (joints(0) > 6.14)
    {
        joints(0) -= 2 * M_PI;
    }
    else if (joints(0) < -6.14)
    {
        joints(0) += 2 * M_PI;
    }

    // joint 1 between -3.14 and 0
    if (joints(1) > 0)
    {
        joints(1) -= 2 * M_PI;
    }
    else if (joints(1) < -3.14)
    {
        joints(1) += 2 * M_PI;
    }

    // joint 0 between ? and 6.14
    if (joints(5) > 6.14)
    {
        joints(5) -= 2 * M_PI;
    }
    else if (joints(5) < -6.14)
    {
        joints(5) += 2 * M_PI;
    }

    // cout << "fixNormalization: " << joints.transpose() << endl;

    return joints;
}

static double calculate_distance(const jointValues &first_vector, const jointValues &second_vector)
{
    jointValues second_norm = bestNormalization(first_vector, second_vector);
    return (first_vector - second_norm).norm();
}

static double calculate_distance_weighted(const jointValues &first_vector, const jointValues &second_vector)
{
    double distance_val = 0;
    double weight[6] = {1, 1, 1, 10, 5, 1};
    jointValues second_norm = bestNormalization(first_vector, second_vector);

    for (int i = 0; i < 6; i++)
    {
        distance_val += weight[i] * fabs(first_vector(i) - second_norm(i));
    }

    return distance_val;
}

static homoMatrix T10f(double th1)
{
    homoMatrix ret;
    ret << cos(th1), -sin(th1), 0, 0,
        sin(th1), cos(th1), 0, 0,
        0, 0, 1, D[0],
        0, 0, 0, 1;
    return ret;
}

static homoMatrix T21f(double th2)
{
    homoMatrix ret;
    ret << cos(th2), -sin(th2), 0, 0,
        0, 0, -1, 0,
        sin(th2), cos(th2), 0, 0,
        0, 0, 0, 1;
    return ret;
}

static homoMatrix T32f(double th3)
{
    homoMatrix ret;
    ret << cos(th3), -sin(th3), 0, A[1],
        sin(th3), cos(th3), 0, 0,
        0, 0, 1, D[2],
        0, 0, 0, 1;
    return ret;
}

static homoMatrix T43f(double th4)
{
    homoMatrix ret;
    ret << cos(th4), -sin(th4), 0, A[2],
        sin(th4), cos(th4), 0, 0,
        0, 0, 1, D[3],
        0, 0, 0, 1;
    return ret;
}

static homoMatrix T54f(double th5)
{
    homoMatrix ret;
    ret << cos(th5), -sin(th5), 0, 0,
        0, 0, -1, -D[4],
        sin(th5), cos(th5), 0, 0,
        0, 0, 0, 1;
    return ret;
}

static homoMatrix T65f(double th6)
{
    homoMatrix ret;
    ret << cos(th6), -sin(th6), 0, 0,
        0, 0, 1, D[5],
        -sin(th6), -cos(th6), 0, 0,
        0, 0, 0, 1;
    return ret;
}