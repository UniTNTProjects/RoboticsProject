#include <eigenMatrices.h>
#include <kinematics.h>
#include <complex.h>
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

//Inverse Kineamtics of UR5

jointValues inverse(coordinates pe, rotMatrix re) {

std::complex<double> complex_converter(1.0, 0.0);
jointValues th;

//dh parameters
//Vector of the A distance (expressed in metres)
const double A[] = {0, -0.425, -0.3922, 0, 0, 0};
//Vector of the D distance (expressed in metres)
const double D[] = {0.1625, 0, 0, 0.1333, 0.0997, 0.0996};

homoMatrix T60;
T60 << re[0], re[3], re[6], pe[0];
        re[1],  re[4], re[7], pe[1];
        re[2], re[5], re[8], pe[3];
        0, 0, 0, 1;
 
//funzioni
auto T10f = [&](double th1) ->homoMatrix {
    homoMatrix ret; ret << cos(th1), -sin(th1), 0, 0,
    sin(th1), cos(th1), 0, 0,
    0, 0, 1, D[0],
    0, 0, 0, 1;
    return ret;
};

auto T21f = [&](double th2) ->homoMatrix{
    homoMatrix ret; ret <<cos(th2), -sin(th2), 0, 0,
    0, 0, -1, 0,
    sin(th2), cos(th2), 0, 0,
    0, 0, 0, 1;
    return ret;
};

auto T32f = [&](double th3) ->homoMatrix{
    homoMatrix ret; ret << cos(th3), -sin(th3), 0, A[1],
    sin(th3), cos(th3), 0, 0,
    0, 0, 1, D[2],
    0, 0, 0, 1;
    return ret;
};

auto T43f = [&](double th4) ->homoMatrix{
    homoMatrix ret; ret <<cos(th4), -sin(th4), 0, A[2];
    sin(th4), cos(th4), 0, 0;
    0, 0, 1, D[3];
    0, 0, 0, 1;
    return ret;
};

auto T54f = [&](double th5) ->homoMatrix{
    homoMatrix ret; ret <<cos(th5), -sin(th5), 0, 0;
    0, 0, -1, -D[4];
    sin(th5), cos(th5), 0, 0;
    0, 0, 0, 1;
    return ret;
};

auto T65f =  [&](double th6) ->homoMatrix{
    homoMatrix ret; ret <<cos(th6), -sin(th6), 0, 0;
    0, 0, 1, D[5];
    -sin(th6), -cos(th6), 0, 0;
    0, 0, 0, 1;
    return ret;
};


 
 //Finding th1
 Eigen::Matrix<double, 4, 1>p50;
 Eigen::Matrix<double, 4, 1>mat;
 mat << 0.0, 0.0, -D[5], 1.0;
 p50 << T60 * mat;
 th(0,0) = real(atan2(p50[1], p50[0])) * complex_converter + acos(D[3] / (sqrt(pow(p50[1], 2) + pow(p50[0], 2)))) + pi/2;
 th(0,1) = real(atan2(p50[1], p50[0])) * complex_converter + acos(D[3] / (sqrt(pow(p50[1], 2) + pow(p50[0], 2)))) + pi/2;
 
 //finding th5
 th(4,0) = real(acos((pe[0] * sin(th(0,0)) - pe[1] * cos(th(0,0)) - D[3]) / D[5] * complex_converter));
 th(4,1) = -real(acos((pe[0] * sin(th(0,0)) - pe[1] * cos(th(0,0)) - D[3]) / D[5] * complex_converter));
 th(4,2) = real(acos((pe[0] * sin(th(0,1)) - pe[1] * cos(th(0,1)) - D[3]) / D[5] * complex_converter));
 th(4,3) = -real(acos((pe[0] * sin(th(0,1)) - pe[1] * cos(th(0,1)) - D[3]) / D[5] * complex_converter));
 

  
//related to th11 a th51
T06 = T60.inverse();
coordinates Xhat, Yhat;
Xhat << T06(0,0), T06(1,0), T06(2,0);
Yhat << T06(0,1), T06(1,1), T06(2,1);
  
//finding th6
th(5,0) = real(atan2(((-Xhat[1] * sin(th(0,0)) + Yhat[1] * cos(th(0,0)))) / sin(th(4,0))), ((Xhat[0] * sin(th(0,0)) - Yhat[0] * cos(th(0,0)))/sin(th(4,0))) * complex_converter);
//related to th11 a th52
th(5,1) = real(atan2(((-Xhat[1] * sin(th(0,0)) + Yhat[1] * cos(th(0,0))) / sin(th(4,1))), ((Xhat[0] * sin(th(0,0)) - Yhat[0] * cos(th(0,0)))/sin(th(4,1)))) * complex_converter);
//related to th12 a th53
th(5,2) = real(atan2(((-Xhat[1] * sin(th(0,1)) + Yhat[1] * cos(th(0,1))) / sin(th(4,2))), ((Xhat[0] * sin(th(0,1)) - Yhat[0] * cos(th(0,1)))/sin(th(4,2)))) * complex_converter);
//related to th12 a th54
th(5,3) = real(atan2(((-Xhat[1] * sin(th(0,1)) + Yhat[1] * cos(th(0,1))) / sin(th(4,3))), ((Xhat[0] * sin(th(0,1)) - Yhat[0] * cos(th(0,1)))/sin(th(4,3)))) * complex_converter);
  
 
//finding th3
homoMatrix T41m = T10f(th(0,0)).inverse() * T60 * T65f(th(5,0)).inverse() * T54f(th(4,0)).inverse();
coordinates p41_1;
p41_1 << T41m(0,3), T41m(1,3), T41m(2,3);
double p41xz_1 = sqrt(pow(p41_1[0], 2) + pow(p41_1[2], 2));
 
T41m = T10f(th(0,0)).inverse() * T60 * T65f(th(5,1)).inverse() * T54f(th(4,1)).inverse();
p41_2 = T41m(0,3), T41m(1,3), T41m(2,3);
p41xz_2 = sqrt(pow(p41_2[0], 2) + pow(p41_2[2]));
 
T41m = T10f(th(0,1)).inverse() * T60 * T65f(th(5,2)).inverse() * T54f(th(4,2)).inverse();
p41_3 = T41m(0,3), T41m(1,3), T41m(2,3);
p41xz_3 = sqrt(pow(p41_3[0], 2) + pow(p41_3[2], 2));
 
T41m = T10f(th(0,1)).inverse() * T60 * T65f(th(5,3)).inverse() * T54f(th(4,3)).inverse();
p41_4 = T41m(0,3), T41m(1,3), T41m(2,3);
double p41xz_4 = sqrt(pow(p41_4[0], 2) + pow(p41_4[2], 2));
 
//Computation of the 8 possible values for th3    
th(2,0) = real(acos((pow(p41xz_1, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]) * complex_converter));
th(2,1) = real(acos((pow(p41xz_1, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]) * complex_converter));
th(2,2) = real(acos((pow(p41xz_1, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]) * complex_converter));
th(2,3) = real(acos((pow(p41xz_1, 2) - pow(A[1], 2) - pow(A[2], 2)) / (2 * A[1] * A[2]) * complex_converter));
 
th(2,4) = -th(2,0);
th(2,5) = -th(2,1);
th(2,6) = -th(2,2);
th(2,7) = -th(2,3);
 
 //Computation of eight possible value for th2
th(1,0) = real(atan2(-p41_1[2], -p41_1[0]) - asin((-A[2] * sin(th(2,0))) / p41xz_1));
th(1,1) = real(atan2(-p41_2[2], -p41_2[0]) - asin((-A[2] * sin(th(2,1))) / p41xz_2));
th(1,2) = real(atan2(-p41_3[2], -p41_3[0]) - asin((-A[2] * sin(th(2,2))) / p41xz_3));
th(1,3) = real(atan2(-p41_4[2], -p41_4[0]) - asin((-A[2] * sin(th(2,3))) / p41xz_4));
 
th(1,4) = real(atan2(-p41_1[2], -p41_1[0]) - asin((A[2] * sin(th3_1)) / p41xz_1));
th(1,5) = real(atan2(-p41_2[2], -p41_2[0]) - asin((A[2] * sin(th3_2)) / p41xz_2));
th(1,6) = real(atan2(-p41_3[2], -p41_3[0]) - asin((A[2] * sin(th3_3)) / p41xz_3));
th(1,7) = real(atan2(-p41_4[2], -p41_4[0]) - asin((A[2] * sin(th3_4)) / p41xz_4));
 
 //find th4
 homoMatrix T43m = T32f(th(2,0)).inverse() * T21f(th(1,0)).inverse() * T10f(th(0,0)).inverse() * T60 * T65f(th(5,0)).inverse() * T54f(th(4,0)).inverse();
 coordinates Xhat43;
 Xhat43 << T43m(0,0), T43m(0,1), T43m(0,2);
th(3,0) = real(atan2(Xhat43[1], Xhat43[0]) * complex_converter);
 
 T43m = T32f(th(2,1)).inverse() * T21f(th(1,1)).inverse() * T10f(th(0,0)).inverse() * T60 * T65f(th(5,1)).inverse() * T54f(th(4,1)).inverse();
 Xhat43 << T43m(0,0), T43m(0,1), T43m(0,2);
th(3,1) = real(atan2(Xhat43[1], Xhat43[0]) * complex_converter);
 
 T43m = T32f(th(2,2)).inverse() * T21f(th(1,2)).inverse() * T10f(th(0,1)).inverse() * T60 * T65f(th(5,2)).inverse() * T54f(th(4,2)).inverse();
 Xhat43 << T43m(0,0), T43m(0,1), T43m(0,2);
th(3,2) = real(atan2(Xhat43[1], Xhat43[0]) * complex_converter);
 
 T43m = T32f(th(2,3)).inverse() * T21f(th(1,3)).inverse() * T10f(th(0,1)).inverse() * T60 * T65f(th(5,3)).inverse() * T54f(th(4,3)).inverse();
 Xhat43 << T43m(0,0), T43m(0,1), T43m(0,2);
th(3,3) = real(atan2(Xhat43[1], Xhat43[0]) * complex_converter);
 
 T43m = T32f(th(2,4)).inverse() * T21f(th(1,4)).inverse() * T10f(th(0,0)).inverse() * T60 * T65f(th(5,0)).inverse() * T54f(th(4,0)).inverse();
 Xhat43 << T43m(0,0), T43m(0,1), T43m(0,2);
th(3,4) = real(atan2(Xhat43[1], Xhat43[0]) * complex_converter);
 
 T43m = T32f(th(2,5)).inverse() * T21f(th(1,5)).inverse() * T10f(th(0,0)).inverse() * T60 * T65f(th(5,1)).inverse() * T54f(th(4,1)).inverse();
 Xhat43 << T43m(0,0), T43m(0,1), T43m(0,2);
th(3,5) = real(atan2(Xhat43[1], Xhat43[0]) * complex_converter);
 
 T43m = T32f(th(2,6)).inverse() * T21f(th(1,6)).inverse() * T10f(th(0,1)).inverse() * T60 * T65f(th(5,2)).inverse() * T54f(th(4,2)).inverse();
 Xhat43 << T43m(0,0), T43m(0,1), T43m(0,2);
th(3,6) = real(atan2(Xhat43[1], Xhat43[0]) * complex_converter);
 
 T43m = T32f(th(2,7)).inverse() * T21f(th(1,7)).inverse() * T10f(th(0,1)).inverse() * T60 * T65f(th(5,3)).inverse() * T54f(th(4,3)).inverse();
 Xhat43 << T43m(0,0), T43m(0,1), T43m(0,2);
th(3,7) = real(atan2(Xhat43[1], Xhat43[0]) * complex_converter);
 
 
 th << th(0,0), th(1,0), th(2,0), th(3,0), th(4,0), th(5,0),
     th(0,0), th(1,1), th(2,1), th(3,1), th(4,1), th(5,1),
     th(0,1), th(1,2), th(2,2), th(3,2), th(4,2), th(5,2),
     th(0,1), th(1,3), th(2,3), th(3,3), th(4,3), th(5,3),
     th(0,0), th(1,4), th(2,4), th(3,4), th(4,0), th(5,0),
     th(0,0), th(1,5), th(2,5), th(3,5), th(4,1), th(5,1),
     th(0,1), th(1,6), th(2,6), th(3,6), th(4,2), th(5,2),
     th(0,1), th(1,7), th(2,7), th(3,7), th(4,3), th(5,3);

return th;
    
}