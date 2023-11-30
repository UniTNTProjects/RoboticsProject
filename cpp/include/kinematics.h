#include <cmath>
#include <eigenMatrices.h>
#include <vector>

using namespace Eigen;
using namespace std;

// Direct Kineamtics of UR5
void ur5Direct(jointValues &th, coordinates &pe, rotMatrix &re);

// Inverse Kineamtics of UR5
Matrix<double, 8, 6> ur5Inverse(coordinates pe, rotMatrix re);

// Inverse Kineamtics of UR5 - Pinocchio CLIK
jointValues ur5InversePinocchio(coordinates pe, rotMatrix re);

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