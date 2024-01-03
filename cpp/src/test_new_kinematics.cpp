#include "controller.h"
#include <computer_vision/GetPoints.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>

using namespace std;
double sum(vector<double> vec)
{
    double sum = 0;
    for (int i = 0; i < vec.size(); i++)
    {
        sum += vec[i];
    }
    return sum;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ur5");
    Controller controller = Controller(125., true);

    coordinates cord, cordCalc;
    cord << 0.2, 0.2, 0.5; // cord_calc: -0.222603 -0.196894  0.672859
    cord << -0.2, -0.2, 0.5;
    rotMatrix rotDefault;
    rotDefault << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;

    // array of rotMatrix
    rotMatrix rotArray[2];
    rotArray[0] << 1, 0, 0, 0, 0, -1, 0, 1, 0;
    rotArray[1] << 1, -1, -1, 1, 0, -1, -1, 1, 0;

    // cord << -0.222603, -0.196894, 0.5; //0.241954 0.181796 0.670908
    // controller.move_to_pinocchio(cord, rotArray[1], 20, false, true);

    // controller.move_to_pinocchio(cord, rotDefault, 20, false, true);

    // for (int i = 0; i < 5; i++)
    // {
    //     cordCalc << cord(0) + 0.1 * i, cord(1), cord(2);
    //     controller.move_to_pinocchio(cordCalc, rotArray[0], 20, false, true);
    // }

    // cord << 0.3, -0.2, 0.5;

    // for (int i = 0; i < 5; i++)
    // {
    //     cordCalc << cord(0), cord(1) + 0.1 * i, cord(2);
    //     controller.move_to_pinocchio(cordCalc, rotArray[0], 20, false, true);
    // }

    // 1.18161 -1.02071 -1.99415 -1.71151 -1.56712 -5.86731
    // 1.18024 -1.02427 -1.99623 -1.69189  -1.5708 -5.89263
    cord << -0.20, -0.20, 0.6;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            cordCalc << cord(0) + 0.1 * i, cord(1) + 0.1 * j, cord(2);
            controller.move_to(cordCalc, rotDefault, 20, false, false);
        }
    }
}
/*

jointValues a, b, c;

a << 0.5, -0.5, 4, 3 * M_PI + 0.5, -4 * M_PI - 0.5, 0;
b << a(0) + M_PI + 0.5, a(1) + M_PI - 0.5, a(2) + 3 * M_PI + 0.5, a(3) + M_PI + 0.5, a(4) + M_PI + 0.5, a(5) + M_PI + 0.5;
c = bestNormalization(a, b);

cout << a.transpose() << endl;
cout << b.transpose() << endl;
cout << c.transpose() << endl;

for (int i = 0; i < 6; i++)
{
    cout << a(i) - c(i) << " ";
}
cout << endl;

for (int i = 0; i < 6; i++)
{
    cout << a(i) - b(i) << " ";
}
cout << endl;
*/

/*
ros::init(argc, argv, "test_ur5");

Controller controller = Controller(1000., false);
rotMatrix rotDefault;
rotDefault << -1, 0, 0,
    0, -1, 0,
    0, 0, 1;

int order[] = {0, 5, 4, 3, 2};

for (int elem : order)
{
    controller.move_to(controller.defaultCordArray[elem], rotDefault, 20, false, true);
    controller.sleep();
}
*/
// ofstream myfile1, myfile2, myfile3;
// myfile1.open("cosine.txt");

// rotMatrix rotation;
// rotation << -1, 0, 0, 0, -1, 0, 0, 0, 1;

// coordinates cord;
// cord << -0.15, 0.15, 1.0;

// const int dim_x = 20;
// const int dim_y = 20;
// const int dim_z = 10;

// for (int p = 0; p < dim_z; p++)
// {
//     for (int i = 0; i < dim_x; i++)
//     {
//         for (int j = 0; j < dim_y; j++)
//         {
//             coordinates position;
//             position << cord(0) + 0.05 * i, cord(1) - 0.05 * j, cord(2) + 0.05 * p;

//             Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(position, rotation);
//             // cout << "inverse kinematics matrix:\n"<< inverse_kinematics_res << endl;
//             vector<double> mean_error = vector<double>();
//             for (int k = 0; k < 8; k++)
//             {
//                 coordinates cord;
//                 rotMatrix rot;
//                 jointValues th;
//                 th << inverse_kinematics_res(k, 0), inverse_kinematics_res(k, 1), inverse_kinematics_res(k, 2),
//                     inverse_kinematics_res(k, 3), inverse_kinematics_res(k, 4), inverse_kinematics_res(k, 5);
//                 ur5Direct(th, cord, rot);

//                 // Evaluate distance between rotation matrices using quaternions

//                 Eigen::Quaterniond q1(rot);
//                 Eigen::Quaterniond q2(rotation);

//                 double error_rot = q1.angularDistance(q2);
//                 mean_error.push_back(error_rot);
//             }
//             double min = *min_element(mean_error.begin(), mean_error.end());
//             myfile1 << min << " ";
//         }
//         myfile1 << endl;
//     }
//     myfile1 << cord(2) + 0.05 * p << endl;
//     cout << p << endl;
// }
// myfile1.close();
// // myfile2.open("example5.txt");

// for (int p = 0; p < dim_z; p++)
// {
//     for (int i = 0; i < dim_x; i++)
//     {
//         for (int j = 0; j < dim_y; j++)
//         {
//             if (pos[i][j][p] < acceptable_error_rot)
//             {
//                 pos[i][j][p] = 0;
//             }

//             myfile2 << pos[i][j][p] << " ";
//         }
//         myfile2 << endl;
//     }
//     myfile2 << cord(2) + 0.01 * p << endl;
//     cout << "sis" << endl;
// }

// myfile2.close();
// myfile3.open("example6.txt");

// for (int p = 0; p < dim_z; p++)
// {
//     for (int i = 0; i < dim_x; i++)
//     {
//         for (int j = 0; j < dim_y; j++)
//         {
//             if (pos[i][j][p] >= acceptable_error_rot)
//             {
//                 pos[i][j][p] = 1;
//             }

//             myfile3 << pos[i][j][p] << " ";
//         }
//         myfile3 << endl;
//     }
//     myfile3 << cord(2) + 0.01 * p << endl;
//     cout << "sus" << endl;
// }

// myfile3.close();

/*
#include "controller.h"
#include <computer_vision/GetPoints.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int main(int argc, char **argv)
{

    rotMatrix rotation;

    rotation << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;

    coordinates cord;
    cord << -0.5, 0.5, 0.45;

    double acceptable_error_cord = 0.1;
    double acceptable_error_rot = 0.01;
    const int dim_x = 100;
    const int dim_y = 100;
    const int dim_z = 10;

    double pos[100][100][10];

    for (int i = 0; i < dim_x; i++)
    {
        for (int j = 0; j < dim_y; j++)
        {
            for (int k = 0; k < dim_z; k++)
            {
                pos[i][j][k] = 5.;
            }
        }
    }

    for (int p = 0; p < dim_z; p++)
    {
        for (int i = 0; i < dim_x; i++)
        {
            for (int j = 0; j < dim_y; j++)
            {
                coordinates position;
                position << cord(0) + 0.01 * i, cord(1) - 0.01 * j, cord(2) + 0.01 * p;

                Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(position, rotation);
                // cout << "inverse kinematics matrix:\n"<< inverse_kinematics_res << endl;

                for (int k = 0; k < 8; k++)
                {
                    coordinates cord;
                    rotMatrix rot;
                    jointValues th;
                    th << inverse_kinematics_res(k, 0), inverse_kinematics_res(k, 1), inverse_kinematics_res(k, 2), inverse_kinematics_res(k, 3), inverse_kinematics_res(k, 4), inverse_kinematics_res(k, 5);
                    ur5Direct(th, cord, rot);

                    for (int z = 0; z < 3; z++)
                    {

                        for (int l = 0; l < 3; l++)
                        {
                            float error_rot = fabs(rot.coeff(z, l) - rotation.coeff(z, l));

                            if (error_rot < pos[i][j][p])
                            {
                                pos[i][j][p] = error_rot;
                            }
                        }
                    }
                }
                cout << pos[i][j][p] << " ";
            }
            cout << endl;
        }
        cout << cord(2) + 0.01 * p << endl;
    }

    for (int p = 0; p < dim_z; p++)
    {
        for (int i = 0; i < dim_x; i++)
        {
            for (int j = 0; j < dim_y; j++)
            {
                if (pos[i][j][p] < acceptable_error_rot)
                {
                    pos[i][j][p] = 0;
                }

                cout << pos[i][j][p] << " ";
            }
            cout << endl;
        }
        cout << cord(2) + 0.01 * p << endl;
    }

    for (int p = 0; p < dim_z; p++)
    {
        for (int i = 0; i < dim_x; i++)
        {
            for (int j = 0; j < dim_y; j++)
            {
                if (pos[i][j][p] >= acceptable_error_rot)
                {
                    pos[i][j][p] = 1;
                }

                cout << pos[i][j][p] << " ";
            }
            cout << endl;
        }
        cout << cord(2) + 0.01 * p << endl;
    }
}

*/

// std::ofstream myfile;
// myfile.open("/home/squinkis/ros_ws/src/locosim/RoboticsProject/cpp/src/test.txt", std::ios::out | std::ios::trunc);

// if (!myfile.is_open())
//     cout << "Unable to open file";
// else
//     cout << "File opened successfully\n";

// rotMatrix rotation;

// rotation << -1, 0, 0,
//     0, -1, 0,
//     0, 0, 1;

// coordinates cord;
// cord << -1.5, 1.5, 0.25;

// double acceptable_error_cord = 0.1;
// double acceptable_error_rot = 0.01;
// int dim_x = 300;
// int dim_y = 300;
// int dim_z = 50;
// double pos[dim_x][dim_y][dim_z];

// for (int i = 0; i < dim_x; i++)
// {
//     for (int j = 0; j < dim_y; j++)
//     {
//         for (int k = 0; k < dim_z; k++)
//         {
//             pos[i][j][k] = 0.0;
//         }
//     }
// }
// for (int p = 0; p < dim_z; p++)
// {
//     for (int i = 0; i < dim_x; i++)
//     {
//         for (int j = 0; j < dim_y; j++)
//         {
//             coordinates position;
//             position << cord(0) + 0.01 * i, cord(1) - 0.01 * j, cord(2) + 0.01 * p;

//             Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(position, rotation);
//             // cout << "inverse kinematics matrix:\n"<< inverse_kinematics_res << endl;

//             for (int k = 0; k < 8; k++)
//             {
//                 coordinates cord;
//                 rotMatrix rot;
//                 jointValues th;
//                 th << inverse_kinematics_res(k, 0), inverse_kinematics_res(k, 1), inverse_kinematics_res(k, 2), inverse_kinematics_res(k, 3), inverse_kinematics_res(k, 4), inverse_kinematics_res(k, 5);
//                 ur5Direct(th, cord, rot);
//                 /*
//                 cout << "cord:" << cord.transpose() << endl
//                      << endl;
//                 cout << "rot:\n"
//                      << rot << endl;
//                 cout << "----------\n";
//                 */
//                 if (!pos[i][j][p])
//                 {
//                     for (int z = 0; z < 3; z++)
//                     {
//                         if (fabs(cord(z) - position(z)) > acceptable_error_cord)
//                         {

//                             break;
//                         }
//                         bool break_flag = false;
//                         for (int l = 0; l < 3; l++)
//                         {
//                             if (fabs(rot.coeff(z, l) - rotation.coeff(z, l)) > acceptable_error_rot)
//                             {
//                                 break_flag = true;
//                                 break;
//                             }
//                         }
//                         if (break_flag)
//                         {
//                             break;
//                         }

//                         for (int l = 1; l < 3; l++)
//                         {
//                             pos[i][j][p] = fabs(rot.coeff(z, l) - rotation.coeff(z, l));
//                         }
//                     }
//                 }
//             }

//             myfile << pos[i][j][p] << " ";
//         }
//         myfile << endl;
//     }
//     myfile << cord(2) + 0.01 * p << endl;
//     cout << "p: \r" << p;
// }
// myfile.close();

/*


for (int i = 0; i < 1; i++)
{
    for (int j = 0; j < 3; j++)
    {
        coordinates cord;
        cord << 0.0 - 0.1 * i, -0.25 + 0.1 * j, 0.5;
        controller.move_to(cord, rotDefault, 20, false, false);
        controller.sleep();

        coordinates cord2;
        cord2 << 0.0 - 0.1 * i, -0.25 + 0.1 * j, 0.7;
        controller.move_to(cord2, rotDefault, 20, true, false);

        controller.move_to(cord, rotDefault, 20, true, false);
    }
}*/

/*
Requested move to     0 -0.05   0.5
inverse_kinematics_res:
      0          0          0          0          0          0          0          0
-3.09869   -2.30499   -3.09869   -2.30499  -0.836604 -0.0429064  -0.836604 -0.0429064
2.50418    2.50418    2.50418    2.50418   -2.50418   -2.50418   -2.50418   -2.50418
-0.976292     1.3716  -0.976292     1.3716    1.76999    -2.1653    1.76999    -2.1653
2.56138   -2.56138    2.56138   -2.56138    2.56138   -2.56138    2.56138   -2.56138
-1.5708     1.5708    -1.5708     1.5708    -1.5708     1.5708    -1.5708     1.5708
*/
