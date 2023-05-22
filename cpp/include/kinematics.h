#include <cmath>
#include <eigenMatrices.h>
#include <vector>

using namespace Eigen;
using namespace std;

// ur5 distance
const double A[] = {0, -0.425, -0.3922, 0, 0, 0};
const double D[] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

// Direct Kineamtics of UR5
void ur5Direct(jointValues &th, coordinates &pe, rotMatrix &re);

// Inverse Kineamtics of UR5
Matrix<double, 8, 6> ur5Inverse(coordinates pe, rotMatrix re);

// Jacobian of UR5
Matrix<double, 6, 6> ur5Jac(jointValues &Th);

vector<double*> ur5Trajectory(jointValues initial_position, jointValues final_position, double minT, double maxT, double dt);
