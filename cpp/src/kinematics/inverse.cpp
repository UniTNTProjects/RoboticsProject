#include <complex.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <map>
#include "kinematics.h"

using namespace Eigen;
using namespace std;

Matrix<double, 8, 6> ur5Inverse(coordinates pe, rotMatrix re)
{

    std::complex<double> complex_converter(1.0, 0.0);
    // jointValues th;
    Matrix<double, 8, 6> th;

    homoMatrix T60;

    T60 << re(0), re(3), re(6), pe(0),
        re(1), re(4), re(7), pe(1),
        re(2), re(5), re(8), pe(2),
        0, 0, 0, 1;

    // Finding th1
    Matrix<double, 4, 1> p50;
    Matrix<double, 4, 1> mat;
    mat << 0.0, 0.0, -D[5], 1.0;
    p50 << T60 * mat;
    /*
     double th0_0 = atan2(p50[1], p50[0]) + acos(D[3] / (sqrt(pow(p50[1], 2) + pow(p50[0], 2)))) + M_PI/2;
     double th0_1 = atan2(p50[1], p50[0]) - acos(D[3] / (sqrt(pow(p50[1], 2) + pow(p50[0], 2)))) + M_PI/2;
     */
    double th0_0 = real(atan2(p50[1], p50[0]) * complex_converter + acos(D[3] / (sqrt(pow(p50[1], 2) + pow(p50[0], 2))) * complex_converter) + M_PI / 2);
    double th0_1 = real(atan2(p50[1], p50[0]) * complex_converter - acos(D[3] / (sqrt(pow(p50[1], 2) + pow(p50[0], 2))) * complex_converter) + M_PI / 2);

    // finding th5
    double th4_0 = real(acos((pe[0] * sin(th0_0) - pe[1] * cos(th0_0) - D[3]) / D[5] * complex_converter));
    double th4_1 = -th4_0;
    double th4_2 = real(acos((pe[0] * sin(th0_1) - pe[1] * cos(th0_1) - D[3]) / D[5] * complex_converter));
    double th4_3 = -th4_2;

    // related to th11 a th51
    homoMatrix T06 = T60.inverse();
    coordinates Xhat, Yhat;
    Xhat << T06(0, 0), T06(1, 0), T06(2, 0);
    Yhat << T06(0, 1), T06(1, 1), T06(2, 1);

    // finding th6
    double th5_0 = real(atan2(((-Xhat[1] * sin(th0_0) + Yhat[1] * cos(th0_0)) / sin(th4_0)), ((Xhat[0] * sin(th0_0) - Yhat[0] * cos(th0_0)) / sin(th4_0))) * complex_converter);
    // related to th11 a th52
    double th5_1 = real(atan2(((-Xhat[1] * sin(th0_0) + Yhat[1] * cos(th0_0)) / sin(th4_1)), ((Xhat[0] * sin(th0_0) - Yhat[0] * cos(th0_0)) / sin(th4_1))) * complex_converter);
    // related to th12 a th53
    double th5_2 = real(atan2(((-Xhat[1] * sin(th0_1) + Yhat[1] * cos(th0_1)) / sin(th4_2)), ((Xhat[0] * sin(th0_1) - Yhat[0] * cos(th0_1)) / sin(th4_2))) * complex_converter);
    // related to th12 a th54
    double th5_3 = real(atan2(((-Xhat[1] * sin(th0_1) + Yhat[1] * cos(th0_1)) / sin(th4_3)), ((Xhat[0] * sin(th0_1) - Yhat[0] * cos(th0_1)) / sin(th4_3))) * complex_converter);

    // finding th3
    homoMatrix T41m = T10f(th0_0).inverse() * T60 * T65f(th5_0).inverse() * T54f(th4_0).inverse();
    coordinates p41_1;
    p41_1 << T41m(0, 3), T41m(1, 3), T41m(2, 3);
    double p41xz_1 = sqrt(pow(p41_1[0], 2) + pow(p41_1[2], 2));

    T41m = T10f(th0_0).inverse() * T60 * T65f(th5_1).inverse() * T54f(th4_1).inverse();
    coordinates p41_2;
    p41_2 << T41m(0, 3), T41m(1, 3), T41m(2, 3);
    double p41xz_2 = sqrt(pow(p41_2[0], 2) + pow(p41_2[2], 2));

    T41m = T10f(th0_1).inverse() * T60 * T65f(th5_2).inverse() * T54f(th4_2).inverse();
    coordinates p41_3;
    p41_3 << T41m(0, 3), T41m(1, 3), T41m(2, 3);
    double p41xz_3 = sqrt(pow(p41_3[0], 2) + pow(p41_3[2], 2));

    T41m = T10f(th0_1).inverse() * T60 * T65f(th5_3).inverse() * T54f(th4_3).inverse();
    coordinates p41_4;
    p41_4 << T41m(0, 3), T41m(1, 3), T41m(2, 3);
    double p41xz_4 = sqrt(pow(p41_4[0], 2) + pow(p41_4[2], 2));

    // Computation of the 8 possible values for th3
    double th2_0 = real(acos((pow(p41xz_1, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]) * complex_converter));
    double th2_1 = real(acos((pow(p41xz_2, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]) * complex_converter));
    double th2_2 = real(acos((pow(p41xz_3, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]) * complex_converter));
    double th2_3 = real(acos((pow(p41xz_4, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]) * complex_converter));

    double th2_4 = -th2_0;
    double th2_5 = -th2_1;
    double th2_6 = -th2_2;
    double th2_7 = -th2_3;

    // Computation of eight possible value for th2
    double th1_0 = real(atan2(-p41_1[2], -p41_1[0]) - asin((-A[2] * sin(th2_0)) / p41xz_1));
    double th1_1 = real(atan2(-p41_2[2], -p41_2[0]) - asin((-A[2] * sin(th2_1)) / p41xz_2));
    double th1_2 = real(atan2(-p41_3[2], -p41_3[0]) - asin((-A[2] * sin(th2_2)) / p41xz_3));
    double th1_3 = real(atan2(-p41_4[2], -p41_4[0]) - asin((-A[2] * sin(th2_3)) / p41xz_4));

    double th1_4 = real(atan2(-p41_1[2], -p41_1[0]) - asin((A[2] * sin(th2_0)) / p41xz_1));
    double th1_5 = real(atan2(-p41_2[2], -p41_2[0]) - asin((A[2] * sin(th2_1)) / p41xz_2));
    double th1_6 = real(atan2(-p41_3[2], -p41_3[0]) - asin((A[2] * sin(th2_2)) / p41xz_3));
    double th1_7 = real(atan2(-p41_4[2], -p41_4[0]) - asin((A[2] * sin(th2_3)) / p41xz_4));

    // find th4
    homoMatrix T43m = T32f(th2_0).inverse() * T21f(th1_0).inverse() * T10f(th0_0).inverse() * T60 * T65f(th5_0).inverse() * T54f(th4_0).inverse();
    coordinates Xhat43;
    Xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
    // cout << "th3_0:\n" << Xhat43 << endl;
    // cout << endl;
    double th3_0 = real(atan2(Xhat43(1), Xhat43(0)) * complex_converter);

    T43m = T32f(th2_1).inverse() * T21f(th1_1).inverse() * T10f(th0_0).inverse() * T60 * T65f(th5_1).inverse() * T54f(th4_1).inverse();
    Xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
    // cout << "th3_1:\n" << Xhat43 << endl;
    // cout << endl;
    double th3_1 = real(atan2(Xhat43(1), Xhat43(0)) * complex_converter);

    T43m = T32f(th2_2).inverse() * T21f(th1_2).inverse() * T10f(th0_1).inverse() * T60 * T65f(th5_2).inverse() * T54f(th4_2).inverse();
    Xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
    // cout << "th3_2:\n" << Xhat43 << endl;
    // cout << endl;
    double th3_2 = real(atan2(Xhat43(1), Xhat43(0)) * complex_converter);

    T43m = T32f(th2_3).inverse() * T21f(th1_3).inverse() * T10f(th0_1).inverse() * T60 * T65f(th5_3).inverse() * T54f(th4_3).inverse();
    Xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
    // cout << "th3_3:\n" << Xhat43 << endl;
    // cout << endl;
    double th3_3 = real(atan2(Xhat43(1), Xhat43(0)) * complex_converter);

    T43m = T32f(th2_4).inverse() * T21f(th1_4).inverse() * T10f(th0_0).inverse() * T60 * T65f(th5_0).inverse() * T54f(th4_0).inverse();
    Xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
    // cout << "th3_4:\n" << Xhat43 << endl;
    // cout << endl;
    double th3_4 = real(atan2(Xhat43(1), Xhat43(0)) * complex_converter);

    T43m = T32f(th2_5).inverse() * T21f(th1_5).inverse() * T10f(th0_0).inverse() * T60 * T65f(th5_1).inverse() * T54f(th4_1).inverse();
    Xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
    // cout << "th3_5:\n" << Xhat43 << endl;
    // cout << endl;
    double th3_5 = real(atan2(Xhat43(1), Xhat43(0)) * complex_converter);

    T43m = T32f(th2_6).inverse() * T21f(th1_6).inverse() * T10f(th0_1).inverse() * T60 * T65f(th5_2).inverse() * T54f(th4_2).inverse();
    Xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
    // cout << "th3_6:\n" << Xhat43 << endl;
    // cout << endl;
    double th3_6 = real(atan2(Xhat43(1), Xhat43(0)) * complex_converter);

    T43m = T32f(th2_7).inverse() * T21f(th1_7).inverse() * T10f(th0_1).inverse() * T60 * T65f(th5_3).inverse() * T54f(th4_3).inverse();
    Xhat43 << T43m(0, 0), T43m(1, 0), T43m(2, 0);
    // cout << "th3_7:\n" << Xhat43 << endl;
    // cout << endl;
    double th3_7 = real(atan2(Xhat43(1), Xhat43(0)) * complex_converter);

    th << (th0_0), (th1_0), (th2_0), (th3_0), (th4_0), (th5_0),
        (th0_0), (th1_1), (th2_1), (th3_1), (th4_1), (th5_1),
        (th0_1), (th1_2), (th2_2), (th3_2), (th4_2), (th5_2),
        (th0_1), (th1_3), (th2_3), (th3_3), (th4_3), (th5_3),
        (th0_0), (th1_4), (th2_4), (th3_4), (th4_0), (th5_0),
        (th0_0), (th1_5), (th2_5), (th3_5), (th4_1), (th5_1),
        (th0_1), (th1_6), (th2_6), (th3_6), (th4_2), (th5_2),
        (th0_1), (th1_7), (th2_7), (th3_7), (th4_3), (th5_3);

    return th;
}

int *sort_inverse(Eigen::Matrix<double, 8, 6> &inverse_kinematics_res, const jointValues &initial_joints)
{
    multimap<double, int> sorted_inverse;

    for (int i = 0; i < 8; i++)
    {
        jointValues inverse_i;
        inverse_i << inverse_kinematics_res(i, 0), inverse_kinematics_res(i, 1), inverse_kinematics_res(i, 2),
            inverse_kinematics_res(i, 3), inverse_kinematics_res(i, 4), inverse_kinematics_res(i, 5);

        // double diff = (inverse_i - initial_joints).norm(); // doesnt take into account the angle normalization
        double diff = calculate_distance_weighted(initial_joints, inverse_i);

        // // facing back of the table
        // if (norm_angle(inverse_i(0)) > 3.7 && norm_angle(inverse_i(0)) < 5.7)
        // {
        //     diff += 100;
        // }
        // cout << "diff: " << diff << ", i: " << i << endl;
        sorted_inverse.insert(pair<double, int>(diff, i));
    }

    int *sorted_indexes = new int[8];
    int i = 0;

    for (std::multimap<double, int>::iterator it = sorted_inverse.begin(); it != sorted_inverse.end(); ++it)
    {
        sorted_indexes[i++] = it->second;
        // cout << "value of sort:" << it->first << ", " << it->second << endl;
    }

    return sorted_indexes;
}
