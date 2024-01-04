#include <cmath>
#include <iostream>
#include "kinematics.h"

#include <fstream>

using namespace Eigen;
using namespace std;

// Definizione dei parametri del robot UR5
// const double d1 = 0.0892;
// const double a2 = -0.425;
// const double a3 = -0.392;
// const double d4 = 0.1093;
// const double d5 = 0.09475;
// const double d6 = 0.0825;

// const double d1 = -0.32;
// const double a2 = -0.78;
// const double a3 = -2.1;
// const double d4 = -1.6;
// const double d5 = -1.57;
// const double d6 = 3.49;

// const double d1 = 0.1625;
// const double a2 = -0.425;
// const double a3 = -0.3922;
// const double d4 = 0.1333;
// const double d5 = 0.0997;
// const double d6 = 0.0996;

const double d1 = 0.0892;
const double a2 = -0.425;
const double a3 = -0.392;
const double d4 = 0.1093;
const double d5 = 0.09475;
const double d6 = 0.0825;

Matrix4d computeEndEffectorPose(const coordinates &pe, const rotMatrix &re)
{
    Matrix4d end_effector_pose;
    end_effector_pose.setIdentity(); // Inizializza con la matrice identità

    // Imposta la parte di rotazione della matrice di trasformazione
    end_effector_pose.block<3, 3>(0, 0) = re;

    // Imposta la parte di traslazione della matrice di trasformazione
    end_effector_pose.block<3, 1>(0, 3) = pe;

    return end_effector_pose;
}

// Funzione per la cinematica inversa del robot UR5
jointValues ur5InverseKinematicsGPT(const coordinates &pe, const rotMatrix &re)
{
    Matrix4d end_effector_pose = computeEndEffectorPose(pe, re);
    jointValues joint_angles;

    double px = end_effector_pose(0, 3);
    double py = end_effector_pose(1, 3);
    double pz = end_effector_pose(2, 3);

    // Calcolo di theta1
    joint_angles(0) = atan2(py, px);

    cout << "pz: " << pz << ", d1:" << d1 << ", diff:" << (pz - d1) << endl;
    // Calcolo di theta5
    double c5 = (pow(px, 2) + pow(py, 2) + pow(pz - d1, 2) - pow(d4, 2) - pow(d6, 2)) / (2 * d4 * d6);
    cout << "c5: " << c5 << endl;
    c5 = std::max(-1.0, std::min(1.0, c5));
    joint_angles(4) = atan2(sqrt(1 - pow(c5, 2)), c5);

    // Calcolo di theta6
    joint_angles(5) = atan2(end_effector_pose(2, 1), -end_effector_pose(2, 0));

    // Calcolo di theta3
    double r = sqrt(pow(px, 2) + pow(py, 2));
    double s = pz - d1;
    double D = (pow(r, 2) + pow(s, 2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);
    joint_angles(2) = atan2(sqrt(1 - pow(D, 2)), D);

    // Calcolo di theta2
    double theta2_temp = atan2(s, r);
    double alpha = atan2(a3 * sin(joint_angles(2)), a2 + a3 * cos(joint_angles(2)));
    joint_angles(1) = theta2_temp - alpha;

    // Calcolo di theta4
    joint_angles(3) = atan2(-end_effector_pose(0, 2), end_effector_pose(1, 2));

    cout << "joint_angles: " << joint_angles << endl;
    coordinates a;
    rotMatrix b;
    ur5Direct(joint_angles, a, b);

    cout << "cord: " << a << endl;
    cout << "rot: " << b << endl;
    return joint_angles;
}