#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>

using namespace std;

coordinates translateBlockCordToRobotCord(coordinates blockCord);

int main(int argc, char **argv)
{
    cout << "Init test_ur5" << endl;

    ros::init(argc, argv, "test_ur5");
    Controller controller = Controller(250.);

    coordinates cord, cord2;
    cord << 0.2, 0.2, 0.5;
    rotMatrix rotDefault;
    rotDefault << -1, 0, 0,
        0, -1, 0,
        0, 0, 1;

    float startX = 0;
    float startY = 0;
    float endX = 1;
    float endY = 0.8;

    int n_x = (endX - startX) / 0.05;
    int n_y = (endY - startY) / 0.05;

    bool reached_cords[n_x][n_y], reached_cords_side_pick[n_x][n_y];
    for (int i = 0; i < n_x; i++)
    {
        for (int j = 0; j < n_y; j++)
        {
            reached_cords[i][j] = false;
            reached_cords_side_pick[i][j] = false;
        }
    }

    vector<double *> reached_cords_joints;

    jointValues start_joints;
    reached_cords_joints.push_back(new double[6]{0.468962, -1.19586, -2.19467, -1.32038, -1.57079, 1.11343});

    for (int z = 0; z < 40; z++)
    {
        cout << "``````````````````````\n\n"
             << z << "``````````````````````\n\n";
        for (int i = 0; i < n_x; i++)
        {
            for (int j = 0; j < n_y; j++)
            {

                cout << "``````````````````````\n\n"
                     << "i: " << i << ", j: " << j << "\n``````````````````````\n\n";
                // random pick from reached_cords_joints
                double *start_joints_arr = reached_cords_joints[rand() % reached_cords_joints.size()];
                start_joints << start_joints_arr[0], start_joints_arr[1], start_joints_arr[2], start_joints_arr[3], start_joints_arr[4], start_joints_arr[5];

                cord << startX + i * 0.05, startY + j * 0.05, 0.5;
                cord = translateBlockCordToRobotCord(cord);

                cord2 = cord;
                cord2(2) = 0.85;
                vector<pair<coordinates, rotMatrix>> poses_rots;
                poses_rots.push_back(make_pair(cord, rotDefault));
                poses_rots.push_back(make_pair(cord2, rotDefault));
                if (!reached_cords_side_pick[i][j])
                {
                    double *test_res_side_pick = controller.test_traj_multiple_side_pick(poses_rots, new bool[2]{false, true}, new bool[2]{false, false}, new bool[2]{false, false}, new bool[2]{false, false}, new bool[2]{true, true}, start_joints, false);
                    if (test_res_side_pick != NULL)
                    {
                        reached_cords_side_pick[i][j] = true;
                        reached_cords_joints.push_back(test_res_side_pick);
                    }
                }

                if (!reached_cords[i][j])
                {
                    double *test_res = controller.test_traj_multiple(poses_rots, new bool[2]{false, true}, new bool[2]{false, false}, new bool[2]{false, false}, new bool[2]{false, false}, new bool[2]{false, false}, start_joints, false);
                    if (test_res != NULL)
                    {
                        reached_cords[i][j] = true;
                        reached_cords_joints.push_back(test_res);
                    }
                }
            }
        }
    }
    cout << "Unreached cords: " << endl;
    for (int i = 0; i < n_x; i++)
    {
        for (int j = 0; j < n_y; j++)
        {
            if (!reached_cords[i][j])
            {
                cout << "-----" << endl;
                cout << cord.transpose() << endl;
                cord << startX + i * 0.05, startY + j * 0.05, 0.5;
                cord = translateBlockCordToRobotCord(cord);
                cout << "(" << cord(0) << "," << cord(1) << "," << reached_cords[i][j] << ")" << endl;
            }
        }
    }

    cout << "Unreached cords side-pick: " << endl;
    for (int i = 0; i < n_x; i++)
    {
        for (int j = 0; j < n_y; j++)
        {
            if (!reached_cords_side_pick[i][j])
            {
                cout << "-----" << endl;
                cout << cord.transpose() << endl;
                cord << startX + i * 0.05, startY + j * 0.05, 0.5;
                cord = translateBlockCordToRobotCord(cord);
                cout << "(" << cord(0) << "," << cord(1) << "," << reached_cords[i][j] << ")" << endl;
            }
        }
    }

    cout << "Reached cords Matrix: \n";
    for (int i = 0; i < n_x; i++)
    {
        for (int j = 0; j < n_y; j++)
        {
            cout << reached_cords[i][j] << ", ";
        }
        cout << endl;
    }

    cout << "Reached cords side pick Matrix: \n";
    for (int i = 0; i < n_x; i++)
    {
        for (int j = 0; j < n_y; j++)
        {
            cout << reached_cords_side_pick[i][j] << ", ";
        }
        cout << endl;
    }
}

coordinates translateBlockCordToRobotCord(coordinates blockCord)
{

    coordinates robotCord, robotReferenceCord;
    // robotReferenceCord << 0.525, 0.34, 1.5;
    robotReferenceCord << 0.5, 0.35, 1.5;

    robotCord << blockCord(0) - robotReferenceCord(0), robotReferenceCord(1) - blockCord(1), 0.72;
    return robotCord;
}
