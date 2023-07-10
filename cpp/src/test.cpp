#include "controller.h"
#include <computer_vision/GetPoints.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int main(int argc, char **argv)
{
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

    rotMatrix rotation;

    rotation << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;

    coordinates cord;
    cord << -1.5, 1.5, 0.25;

    double acceptable_error_cord = 0.1;
    double acceptable_error_rot = 0.01;
    int dim_x = 300;
    int dim_y = 300;
    int dim_z = 50;
    bool pos[dim_x][dim_y][dim_z];

    for (int i = 0; i < dim_x; i++)
    {
        for (int j = 0; j < dim_y; j++)
        {
            for (int k = 0; k < dim_z; k++)
            {
                pos[i][j][k] = false;
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
                    /*
                    cout << "cord:" << cord.transpose() << endl
                         << endl;
                    cout << "rot:\n"
                         << rot << endl;
                    cout << "----------\n";
                    */
                    if (!pos[i][j][p])
                    {
                        for (int z = 0; z < 3; z++)
                        {
                            if (fabs(cord(z) - position(z)) > acceptable_error_cord)
                            {

                                break;
                            }
                            bool break_flag = false;
                            for (int l = 0; l < 3; l++)
                            {
                                if (fabs(rot.coeff(z, l) - rotation.coeff(z, l)) > acceptable_error_rot)
                                {
                                    break_flag = true;
                                    break;
                                }
                            }
                            if (break_flag)
                            {
                                break;
                            }

                            if (z == 2)
                            {
                                pos[i][j][p] = true;
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
}
