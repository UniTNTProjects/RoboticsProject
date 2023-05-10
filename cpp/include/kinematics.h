#include <cmath>
#include "eigenMatrices.h"

using namespace Eigen;

//ur5 distance
const double A[] = {0, -0.425, -0.3922, 0, 0, 0};
const double D[] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

//Direct Kineamtics of UR5
void ur5Direct(jointValues th, coordinates pe, rotMatrix re);

//Inverse Kineamtics of UR5
jointValues ur5Inverse(coordinates pe, rotMatrix re);

//Jacobian of UR5
Matrix<double, 6, 6> ur5Jac(jointValues& Th);


