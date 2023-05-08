#include <iostream>
#include <vector>
#include <cmath>
#include "eigenMatrices.h"

using namespace std;
using namespace Eigen;

MatrixXd ur5Jac(const vector<double>& Th) {
    vector<double> A = {0, -0.425, -0.3922, 0, 0, 0};
    vector<double> D = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

    double A1 = A[0], A2 = A[1], A3 = A[2], A4 = A[3], A5 = A[4], A6 = A[5];
    double D1 = D[0], D2 = D[1], D3 = D[2], D4 = D[3], D5 = D[4], D6 = D[5];

    double th1 = Th[0], th2 = Th[1], th3 = Th[2], th4 = Th[3], th5 = Th[4], th6 = Th[5];

    MatrixXd J(6, 6);

    J(0, 0) = D5 * (cos(th1) * cos(th5) + cos(th2 + th3 + th4) * sin(th1) * sin(th5)) + D3 * cos(th1) + D4 * cos(th1) - A3 * cos(th2 + th3) * sin(th1) - A2 * cos(th2) * sin(th1) - D5 * sin(th2 + th3 + th4) * sin(th1);
    J(1, 0) = D5 * (cos(th5) * sin(th1) - cos(th2 + th3 + th4) * cos(th1) * sin(th5)) + D3 * sin(th1) + D4 * sin(th1) + A3 * cos(th2 + th3) * cos(th1) + A2 * cos(th1) * cos(th2) + D5 * sin(th2 + th3 + th4) * cos(th1);
    J(2, 0) = 0;
    J(3, 0) = 0;
    J(4, 0) = 0;
    J(5, 0) = 1;

    J(0, 1) = -cos(th1) * (A3 * sin(th2 + th3) + A2 * sin(th2) + D5 * (sin(th2 + th3) * sin(th4) - cos(th2 + th3) * cos(th4)) - D5 * sin(th5) * (cos(th2 + th3) * sin(th4) + sin(th2 + th3) * cos(th4)));
    J(1, 1) = -sin(th1) * (A3 * sin(th2 + th3) + A2 * sin(th2) + D5 * (sin(th2 + th3) * sin(th4) - cos(th2 + th3) * cos(th4)) - D5 * sin(th5) * (cos(th2 + th3) * sin(th4) + sin(th2 + th3) * cos(th4)));
    J(2, 1) = A3 * cos(th2 + th3) - (D5 * sin(th2 + th3 + th4 + th5)) / 2 + A2 * cos(th2) + (D5 * sin(th2 + th3 + th4 - th5)) / 2 + D5 * sin(th2 + th3 + th4);
    J(3, 1) = sin(th1);
    J(4, 1) = -cos(th1);
    J(5, 1) = 0;

    J(0, 2) = cos(th1) * (D5 * cos(th2 + th3 + th4) - A3 * sin(th2 + th3) + D5 * sin(th2 + th3 + th4) * sin(th5));
    J(1, 2) = sin(th1) * (D5 * cos(th2 + th3 + th4) - A3 * sin(th2 + th3) + D5 * sin(th2 + th3 + th4) * sin(th5));
    J(2, 2) = A3 * cos(th2 + th3) - (D5 * sin(th2 + th3 + th4 + th5)) / 2 + (D5 * sin(th2 + th3 + th4 - th5)) / 2 + D5 * sin(th2 + th3 + th4);
    J(3, 2) = sin(th1);
    J(4, 2) = -cos(th1);
    J(5, 2) = 0;

    J(0, 3) = D5 * cos(th1) * (cos(th2 + th3 + th4) + sin(th2 + th3 + th4) * sin(th5));
    J(1, 3) = D5 * sin(th1) * (cos(th2 + th3 + th4) + sin(th2 + th3 + th4) * sin(th5));
    J(2, 3) = D5 * (sin(th2 + th3 + th4 - th5) / 2 + sin(th2 + th3 + th4) - sin(th2 + th3 + th4 + th5) / 2);
    J(3, 3) = sin(th1);
    J(4, 3) = -cos(th1);
    J(5, 3) = 0;

    J(0, 4) = -D5 * sin(th1) * sin(th5) - D5 * cos(th2 + th3 + th4) * cos(th1) * cos(th5);
    J(1, 4) = D5 * cos(th1) * sin(th5) - D5 * cos(th2 + th3 + th4) * cos(th5) * sin(th1);
    J(2, 4) = -D5 * (sin(th2 + th3 + th4 - th5) / 2 + sin(th2 + th3 + th4 + th5) / 2);
    J(3, 4) = sin(th2 + th3 + th4) * cos(th1);
    J(4, 4) = sin(th2 + th3 + th4) * sin(th1);
    J(5, 4) = -cos(th2 + th3 + th4);

    J(0, 5) = 0;
    J(1, 5) = 0;
    J(2, 5) = 0;
    J(3, 5) = cos(th5) * sin(th1) - cos(th2 + th3 + th4) * cos(th1) * sin(th5);
    J(4, 5) = -cos(th1) * cos(th5) - cos(th2 + th3 + th4) * sin(th1) * sin(th5);
    J(5, 5) = -sin(th2 + th3 + th4) * sin(th5);

    return J;
}



