#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <kinematics.h>

using namespace Eigen;
using namespace std;

vector<Matrix<double, 1, 7>> ur5Trajectory(jointValues initial_position, jointValues final_position, double minT, double maxT, double dt)
{
    Matrix<double, 6, 4> A;
    for (int i = 0; i < 6; i++) {
        Matrix<double,4 ,4> M;
        M << 1, minT, minT * minT, minT * minT * minT,
             0, 1, 2 * minT, 3 * minT * minT,
             1, maxT, maxT * maxT, maxT * maxT * maxT,
             0, 1, 2 * maxT, 3 * maxT * maxT;

        Matrix<double, 4, 1> a, b;
        b << initial_position(i), 0, final_position(i), 0;

        a = M.inverse() * b;
        A.row(i) = a.transpose();
    }
    
    vector<Matrix<double, 1, 7>> Th;
    
    for (double t = minT; t <= maxT; t += dt) {
        Matrix<double, 1, 7> th;
        th(0) = t;
        for (int i = 0; i < 6; i++) {
            double q = A(i, 0) + A(i, 1) * t + A(i, 2) * t * t + A(i, 3) * t * t * t;
            th(i + 1) = q;
        }
        Th.push_back(th);
    }
    return Th;
}