#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <kinematics.h>

using namespace Eigen;
using namespace std;

void ur5Trajectory(vector<double *> *&Th, jointValues initial_position, jointValues final_position, double minT, double maxT, double dt)
{
    Matrix<double, 6, 4> A;
    for (int i = 0; i < 6; i++)
    {
        Matrix<double, 4, 4> M;
        M << 1, minT, minT * minT, minT * minT * minT,
            0, 1, 2 * minT, 3 * minT * minT,
            1, maxT, maxT * maxT, maxT * maxT * maxT,
            0, 1, 2 * maxT, 3 * maxT * maxT;

        Matrix<double, 4, 1> a, b;
        b << initial_position(i), 0, final_position(i), 0;

        a = M.inverse() * b;
        A.row(i) = a.transpose();
    }

    for (double t = minT; t <= maxT; t += dt)
    {
        double th[6];
        for (int i = 0; i < 6; i++)
        {
            double q = A(i, 0) + A(i, 1) * t + A(i, 2) * t * t + A(i, 3) * t * t * t;
            th[i] = q;
        }

        Th->push_back(th);
    }
    /*
        cout << "Trajectory size: " << Th->size() << endl;
        for (int i = 0; i < Th->size(); i++)
        {
            cout << "Trajectory " << i << ": ";
            for (int j = 0; j < 6; j++)
            {
                cout << Th->at(i)[j] << " ";
            }
            cout << endl;
        }
        */
}
