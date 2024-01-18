#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> jointValues;
typedef Eigen::Matrix<double, 4, 4> homoMatrix;
typedef Eigen::Matrix<double, 3, 1> coordinates; // x,y,z
typedef Eigen::Matrix<double, 3, 3> rotMatrix;

// // dh parameters
// // Vector of the A distance (expressed in metres)
// const double A[] = {0, -0.425, -0.3922, 0, 0, 0};
// // Vector of the D distance (expressed in metres)
// const double D[] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

// // dh parameters
// // Vector of the A distance (expressed in metres)
// const double A[] = {0, -0.425, -0.3922, 0, 0, 0};
// // Vector of the D distance (expressed in metres)
// const double D[] = {0.0892, 0, 0, 0.1093, 0.09475, 0.0825};

const double A[6] = {0, -0.425, -0.3922, 0, 0, 0};
const double D[6] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996 + 0.14};