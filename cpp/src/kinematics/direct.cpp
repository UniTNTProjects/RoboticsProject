#include <eigenMatrices.h>
#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;

//Direct Kinematics of the UR5
//Th: six joint angles
//pe: cartesian position of the end effector
//re: Rotation matrix of the end effector


//dh parameters
//Vector of the A distance (expressed in metres)
const double A[] = {0, -0.425, -0.3922, 0, 0, 0};
//Vector of the D distance (expressed in metres)
const double D[] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

//alfa = [0, pi/2, 0, 0, pi/2, -pi/2];


void ur5Direct(jointValues &th, coordinates &pe, rotMatrix &re) {

    homoMatrix T10m;
    //T10f(Th(1));

    homoMatrix T21m;
    //T21f(Th(2));

    homoMatrix T32m;
    //T32f(Th(3));

    homoMatrix T43m;
    //T43f(Th(4));

    homoMatrix T54m;
    //T54f(Th(5));

    homoMatrix T65m;
    //T65f(Th(6));

    homoMatrix T06;
    //end effector

    T10m << cos(th[0]), -sin(th[0]), 0, 0,
        sin(th[0]), cos(th[0]), 0, 0,
        0, 0, 1, D[0],
        0, 0, 0, 1;

    T21m << cos(th[1]), -sin(th[1]), 0, 0,
        0, 0, -1, 0,
        sin(th[1]), cos(th[1]), 0, 0,
        0, 0, 0, 1;

    T32m << cos(th[2]), -sin(th[2]), 0, A[1],
        sin(th[2]), cos(th[2]), 0, 0,
        0, 0, 1, D[2],
        0, 0, 0, 1;

    T43m << cos(th[3]), -sin(th[3]), 0, A[2],
        sin(th[3]), cos(th[3]), 0, 0,
        0, 0, 1, D[3],
        0, 0, 0, 1;

    T54m << cos(th[4]), -sin(th[4]), 0, 0,
        0, 0, -1, -D[4],
        sin(th[4]), cos(th[4]), 0, 0,
        0, 0, 0, 1;

    T65m << cos(th[5]), -sin(th[5]), 0, 0,
        0, 0, 1, D[5],
        -sin(th[5]), -cos(th[5]), 0, 0,
        0, 0, 0, 1;

    T06 =T10m*T21m*T32m*T43m*T54m*T65m;

    pe << T06(0,3), T06(1,3), T06(2,3);
    //(1:3,4);
    re << T06(0,0), T06(0,1), T06(0,2), 
        T06(1,0), T06(1,1), T06(1,2),
        T06(2,0), T06(2,1), T06(2,2);
    //(1:3, 1:3);

}