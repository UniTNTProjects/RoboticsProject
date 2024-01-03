#include "controller.h"
#include <computer_vision/GetPoints.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>

using namespace std;

bool trajectory_multiple_positions(vector<vector<double *>> *th_sum, vector<pair<coordinates, rotMatrix>> *positions, int n_positions, int n, jointValues init_joint, vector<bool> order, int steps);
int *sort_inverse(Eigen::Matrix<double, 8, 6> &inverse_kinematics_res, const jointValues &initial_joints);
bool init_verify_trajectory(vector<double *> *Th, jointValues init_joint, jointValues final_joint, int steps, bool pick_or_place);
double calculate_distance_weighted(const jointValues &first_vector, const jointValues &second_vector);
bool check_trajectory(vector<double *> traj, int step, bool pick_or_place, jointValues init_joints);

const double max_x = 1;
const double max_y = 0.4;
const double max_z = 0.8;

const double min_x = -1;
const double min_y = -1;
const double min_z = -1;

const double max_y_near_end_table = 0.18;
const double max_z_near_end_table = 0.58;

const double max_z_moving = 0.73;

const bool debug_traj = true;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test");
    const coordinates defaultCordArray[6] = {
        (coordinates() << 0.35, 0.03, 0.55).finished(),
        (coordinates() << 0.01, 0.03, 0.55).finished(),
        (coordinates() << -0.33, 0.03, 0.55).finished(),
        (coordinates() << -0.33, -0.29, 0.55).finished(),
        (coordinates() << 0.01, -0.29, 0.55).finished(),
        (coordinates() << 0.35, -0.29, 0.55).finished(),

    };

    const rotMatrix rotDefault = (rotMatrix() << -1, 0, 0,
                                  0, -1, 0,
                                  0, 0, 1)
                                     .finished();

    int steps = 20;

    vector<vector<double *>> th_sum = vector<vector<double *>>();
    for (int j = 0; j < 4; j++)
    {
        vector<double *> trajectory = vector<double *>();
        for (int i = 0; i < steps; i++)
        {
            trajectory.push_back(new double[6]);
        }
        th_sum.push_back(trajectory);
    }

    vector<pair<coordinates, rotMatrix>> positions = vector<pair<coordinates, rotMatrix>>();

    positions.push_back(make_pair(defaultCordArray[0], rotDefault));
    positions.push_back(make_pair(defaultCordArray[5], rotDefault));
    positions.push_back(make_pair(defaultCordArray[4], rotDefault));
    positions.push_back(make_pair(defaultCordArray[3], rotDefault));

    coordinates startCord;
    startCord << 0.40, 0.03, 0.55;

    Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(startCord, rotDefault);

    int index = 0;
    jointValues joint_to_check;
    joint_to_check << inverse_kinematics_res(index, 0), inverse_kinematics_res(index, 1), inverse_kinematics_res(index, 2),
        inverse_kinematics_res(index, 3), inverse_kinematics_res(index, 4), inverse_kinematics_res(index, 5);

    trajectory_multiple_positions(&th_sum, &positions, positions.size(), 0, joint_to_check, vector<bool>{false, false, false, false}, steps);

    for (int i = 0; i < th_sum.size(); i++)
    {
        for (int j = 0; j < th_sum[i].size(); j++)
        {
            cout << "th_sum[" << i << "][" << j << "]: ";
            for (int k = 0; k < 6; k++)
            {
                cout << th_sum[i][j][k] << " ";
            }
            cout << endl;
        }
    }
}

bool trajectory_multiple_positions(vector<vector<double *>> *th_sum, vector<pair<coordinates, rotMatrix>> *positions, int n_positions, int n, jointValues init_joint, vector<bool> order, int steps)
{
    cout << "n: " << n << endl;
    if (n == n_positions)
    {
        cout << "trajectory verified" << endl;
        return true;
    }
    coordinates cord = (*positions)[n].first;
    rotMatrix rotation = (*positions)[n].second;

    Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(cord, rotation);

    int *indexes = sort_inverse(inverse_kinematics_res, init_joint);

    for (int i = 0; i < 8; i++)
    {
        int index = indexes[i];
        jointValues joint_to_check;
        joint_to_check << inverse_kinematics_res(index, 0), inverse_kinematics_res(index, 1), inverse_kinematics_res(index, 2),
            inverse_kinematics_res(index, 3), inverse_kinematics_res(index, 4), inverse_kinematics_res(index, 5);

        joint_to_check = bestNormalization(init_joint, joint_to_check);

        // cout << "joint_to_check: " << joint_to_check.transpose() << endl;
        if (init_verify_trajectory(&(th_sum->at(n)), init_joint, joint_to_check, steps, order[n]))
        {
            // cout << "trajectory verified" << endl;

            if (trajectory_multiple_positions(th_sum, positions, n_positions, n + 1, joint_to_check, order, steps))
            {
                return true;
            }
        }
    }

    return false;
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

        // facing back of the table
        if (norm_angle(inverse_i(0)) > 3.7 && norm_angle(inverse_i(0)) < 5.7)
        {
            diff += 100;
        }
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

bool init_verify_trajectory(vector<double *> *Th, jointValues init_joint, jointValues final_joint, int steps, bool pick_or_place)
{
    ur5Trajectory(Th, init_joint, final_joint, steps);

    return check_trajectory(*Th, steps, pick_or_place, init_joint);
}

double calculate_distance_weighted(const jointValues &first_vector, const jointValues &second_vector)
{
    double distance_val = 0;
    double weight[6] = {1, 1, 1, 10, 5, 1};
    jointValues second_norm = bestNormalization(first_vector, second_vector);

    for (int i = 0; i < 6; i++)
    {
        distance_val += weight[i] * fabs(first_vector(i) - second_norm(i));
    }

    return distance_val;
}

bool check_trajectory(vector<double *> traj, int step, bool pick_or_place, jointValues init_joints)
{
    coordinates start_cord;
    rotMatrix start_rotation;
    ur5Direct(init_joints, start_cord, start_rotation);

    // cout << "check trajectory" << endl;
    for (int i = 0; i < step; i++)
    {
        coordinates cord;
        rotMatrix rot;
        jointValues joints;
        joints << traj[i][0], traj[i][1], traj[i][2],
            traj[i][3], traj[i][4], traj[i][5];

        ur5Direct(joints, cord, rot);
        if (debug_traj)
        {

            if (i == step - 1)
            {
                cout << "last cord: " << cord.transpose() << endl;
                cout << "last joint: " << joints.transpose() << endl;
                cout << "last rotation: " << endl
                     << rot << endl;
            }
        }
        if (!(cord(0) > min_x && cord(0) < max_x && cord(1) > min_y && cord(1) < max_y && cord(2) > min_z && cord(2) < max_z))
        {
            if (debug_traj)
            {
                cout << "coordinates of trajectory does not fit in the workspace:" << cord.transpose() << endl;
            }
            return false;
        }

        if (!pick_or_place && cord(2) > max_z_moving)
        {
            if (debug_traj)
            {
                cout << "cord z wrong: " << cord.transpose() << endl;
            }
            return false;
        }

        if (!pick_or_place && cord(2) > max_z_near_end_table && cord(1) > max_y_near_end_table)
        {
            if (debug_traj)
            {
                cout << "cord end table wrong: " << cord.transpose() << endl;
            }
            return false;
        }

        if (pick_or_place && (fabs(traj[0][4] - joints(4)) > M_PI / 2 || fabs(traj[0][5] - joints(5)) > M_PI / 2))
        {
            if (debug_traj)
            {
                cout << "trying strange rotation for pick or place" << endl;
                cout << "traj[0][4]: " << traj[0][4] << ", traj[0][5]: " << traj[0][5] << endl;
                cout << "joints(4): " << joints(4) << ", joints(5): " << joints(5) << endl;
                cout << fabs(traj[0][4] - joints(4)) << ", " << fabs(traj[0][5] - joints(5)) << endl;
            }
            return false;
        }

        if (pick_or_place && (start_rotation - rot).norm() > 0.1)
        {
            if (debug_traj)
            {
                cout << "rotations are different" << endl;
                cout << "start_rot: " << start_rotation << endl;
                cout << "rot: " << rot << endl;
            }
            return false;
        }

        if (joints.norm() == 0)
        {
            continue;
        }

        MatrixXd jacobian = ur5Jac(joints);
        // cout << "jacobian: " << jacobian << endl;

        if (joints(1) > 0 || joints(1) < -3.14)
        {
            if (debug_traj)
            {
                cout << "joints(1) invalid:" << joints(1) << endl;
            }
            return false;
        }

        if (abs(jacobian.determinant()) < 0.000001)
        {
            if (debug_traj)
            {
                cout << "----------\ntrajectory invalid 1" << endl;
                cout << "determinant of jacobian is " << abs(jacobian.determinant()) << "\n-------------\n"
                     << endl;
            }
            return false;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (abs(svd.singularValues()(5)) < 0.0000001)
        {
            if (debug_traj)
            {
                cout << "trajectory invalid 2" << endl;
            }
            cout << abs(svd.singularValues()(5)) << endl;
            return false;
        }
    }
    cout << "trajectory valid" << endl;

    return true;
}
