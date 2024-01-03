#include <cmath>
#include <eigenMatrices.h>
#include <vector>

using namespace Eigen;
using namespace std;

// Direct Kineamtics of UR5
void ur5Direct(jointValues &th, coordinates &pe, rotMatrix &re);

Matrix4d computeEndEffectorPose(const coordinates &pe, const rotMatrix &re);

jointValues ur5InverseKinematicsGPT(const coordinates &pe, const rotMatrix &re);

// Inverse Kineamtics of UR5
Matrix<double, 8, 6> ur5Inverse(coordinates pe, rotMatrix re);

// Inverse Kineamtics of UR5 - Pinocchio CLIK
jointValues ur5InversePinocchio(coordinates pe, rotMatrix re, jointValues current_joints);

// Jacobian of UR5
Matrix<double, 6, 6> ur5Jac(jointValues &Th);

void ur5Trajectory(vector<double *> *Th, jointValues initial_position, jointValues final_position, int steps);

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
    else if (joints(0) < -6.14)
    {
        joints(5) += 2 * M_PI;
    }

    return joints;
}