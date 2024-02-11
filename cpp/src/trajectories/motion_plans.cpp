#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "motion_trajectory.h"

using namespace Eigen;
using namespace std;

vector<double *> up_tray(jointValues startJoint, bool side_pick, bool isGripping)
{
    coordinates startCord;
    rotMatrix startRot;
    ur5Direct(startJoint, startCord, startRot);

    double new_z = startCord(2) - 0.2;

    if (isGripping)
    {
        new_z = startCord(2) - 0.07;
    }

    if (new_z < 0.5)
    {
        new_z = 0.5;
    }
    // if difference in new_z and currentPos(2) is too small, don't move up
    if (abs(new_z - startCord(2)) < 0.03)
    {
        // return empty vector
        return vector<double *>();
    }
    coordinates aboveCurrent;
    aboveCurrent << startCord(0), startCord(1), new_z;

    vector<double *> up_traj = calc_direct_traj(aboveCurrent, startRot, true, false, startJoint, side_pick, false);
    if (up_traj.size() > 0)
    {
        return up_traj;
    }
    else
    {
        return vector<double *>();
    }
}

vector<double *> up_and_move(const coordinates &position, const rotMatrix &rotation, int steps, jointValues startJoint, bool side_pick, bool isGripping)
{

    cout << "### Calc up&move: " << endl;

    vector<double *> up_traj = up_tray(startJoint, side_pick, isGripping);
    if (up_traj.size() > 0)
    {
        coordinates aboveCurrentpos;
        rotMatrix rot;

        jointValues aboveCurrentJoint;
        aboveCurrentJoint << up_traj[up_traj.size() - 1][0], up_traj[up_traj.size() - 1][1], up_traj[up_traj.size() - 1][2],
            up_traj[up_traj.size() - 1][3], up_traj[up_traj.size() - 1][4], up_traj[up_traj.size() - 1][5];
        ur5Direct(aboveCurrentJoint, aboveCurrentpos, rot);

        coordinates aboveNext;
        aboveNext << position(0), position(1), aboveCurrentpos(2);

        vector<pair<coordinates, rotMatrix>> positions = vector<pair<coordinates, rotMatrix>>();

        positions.push_back(make_pair(aboveNext, rotation));
        positions.push_back(make_pair(position, rotation));

        bool pick_or_place[] = {false, true};
        bool homing[] = {false, false};
        bool side_pick_array[] = {false, side_pick};
        bool up_and_move_flag[] = {false, false};
        bool move_to_near_axis_flag[] = {false, false};

        vector<double *> move_tray = calc_traj(position, rotation, false, false, false, false, aboveCurrentJoint, side_pick, isGripping, false);
        if (move_tray.size() > 0)
        {
            up_traj.insert(up_traj.end(), move_tray.begin(), move_tray.end());
            cout << "### Success\n"
                 << endl;
            return up_traj;
        }
        else
        {
            vector<double *> move_and_down_tray = calc_traj_multiple(positions, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag, aboveCurrentJoint, side_pick_array, isGripping);
            if (move_and_down_tray.size() > 0)
            {
                up_traj.insert(up_traj.end(), move_and_down_tray.begin(), move_and_down_tray.end());
                cout << "### Success 2 \n"
                     << endl;
                return up_traj;
            }
        }
    }
    cout << "§§§ Failed\n"
         << endl;
    return vector<double *>();
}

vector<double *> reset_main_joint(const coordinates &position, const rotMatrix &rotation, int steps, jointValues startJoint, bool side_pick, bool isGripping)
{
    jointValues reset_values;
    reset_values = mainJointResetValues;
    reset_values(0) = startJoint(0);
    reset_values(3) = startJoint(3);
    reset_values(4) = startJoint(4);
    reset_values(5) = startJoint(5);

    if (startJoint(2) > 0)
    {
        reset_values(2) = 2.2;
    }
    else
    {
        reset_values(2) = -2.2;
    }

    double angle;

    angle = atan2(position(1), position(0));

    if (position(1) > 0)
    {
        angle = angle;
    }
    else
    {
        angle -= M_PI / 2;
    }

    // check if joint 1 in on the right or left of the robot
    if (startJoint(1) < -M_PI / 2)
    {
        angle += M_PI;
        // cout << "adding 180" << endl;
    }

    if (debug_traj)
    {
        cout << "--------------\nnorm_angle(angle): " << norm_angle(angle) << endl;
        cout << "norm angle in degrees: " << norm_angle(angle) * 180 / M_PI << endl;
    }
    if (norm_angle(angle) > 30 * M_PI / 180 && norm_angle(angle) < M_PI - 30 * M_PI / 180)
    {
        if (debug_traj)
        {
            cout << "reset near end of table" << endl;
        }
        if (startJoint(1) < -M_PI / 2)
        {
            reset_values(1) -= 35 * M_PI / 180;
            reset_values(2) += 0.3;
        }
        else
        {
            reset_values(1) += 35 * M_PI / 180;
            reset_values(2) -= 0.3;
        }
    }

    if (position(0) < 0)
    {

        while (angle > startJoint(0))
        {
            angle -= 2 * M_PI;
        }

        while (angle + 2 * M_PI < startJoint(0))
        {
            angle += 2 * M_PI;
        }

        cout << "reset anti clockwise" << endl;
        cout.flush();
    }
    else
    {
        while (angle < startJoint(0))
        {
            angle += 2 * M_PI;
        }
        while (angle - 2 * M_PI > startJoint(0))
        {
            angle -= 2 * M_PI;
        }

        cout << "reset clockwise" << endl;
        cout.flush();
    }

    if (angle < 6.14 && angle > -6.14)
    {
        jointValues reset_values2;
        reset_values2 = reset_values;
        reset_values2(0) = angle;

        if (debug_traj)
        {
            cout << "reset_values part 1: " << reset_values.transpose() << endl;

            cout << "reset_values part 2: " << reset_values2.transpose() << endl;

            coordinates cord_calc;
            rotMatrix rotation_calc;
            ur5Direct(reset_values2, cord_calc, rotation_calc);

            cout << "cord_calc: " << cord_calc.transpose() << endl;
        }

        vector<jointValues> joints = vector<jointValues>();

        joints.push_back(reset_values);
        joints.push_back(reset_values2);

        bool pick_or_place[] = {false, false};
        bool homing[] = {true, false};
        bool side_pick_array[] = {false, side_pick};

        vector<double *> reset_traj = calc_direct_traj_multiple_joint(joints, pick_or_place, homing, startJoint, side_pick_array, isGripping);
        if (reset_traj.size() > 0)
        {
            return reset_traj;
        }
    }

    return vector<double *>();
}

vector<double *> calc_direct_traj_joint(const jointValues endJoint, bool pick_or_place, bool homing, jointValues startJoint, bool side_pick, bool isGripping)
{
    coordinates startPos;
    rotMatrix startRot;
    ur5Direct(startJoint, startPos, startRot);

    coordinates position;
    rotMatrix rotation;
    ur5Direct(endJoint, position, rotation);

    vector<double *> trajectory = vector<double *>();
    for (int i = 0; i < steps; i++)
    {
        trajectory.push_back(new double[6]);
    }

    if (init_verify_trajectory(&trajectory, startJoint, endJoint, steps, pick_or_place, position, rotation, homing, side_pick, isGripping))
    {
        if (debug_traj)
        {
            cout << "trajectory verified" << endl;
            cout << "startJoint: " << startJoint.transpose() << endl;
        }

        return trajectory;
    }

    return vector<double *>();
}

vector<double *> calc_direct_traj_multiple_joint(vector<jointValues> endJoints, bool *pick_or_place, bool *homing, jointValues startJoint, bool *side_pick, bool isGripping)
{
    vector<double *> trajectory_tot = vector<double *>();
    for (int i = 0; i < endJoints.size(); i++)
    {
        jointValues currentJoints;
        if (i == 0)
        {
            currentJoints = startJoint;
        }
        else
        {
            currentJoints = endJoints[i - 1];
        }
        vector<double *> direct_traj = calc_direct_traj_joint(endJoints[i], pick_or_place[i], homing[i], currentJoints, side_pick[i], isGripping);
        if (direct_traj.size() > 0)
        {
            for (int j = 0; j < direct_traj.size(); j++)
            {
                trajectory_tot.push_back(direct_traj[j]);
            }
        }
        else
        {
            return vector<double *>();
        }
    }
    return trajectory_tot;
}

vector<double *> calc_direct_traj(const coordinates &position, const rotMatrix &rotation, bool pick_or_place, bool homing, jointValues startJoint, bool side_pick, bool isGripping)
{
    coordinates startPos;
    rotMatrix startRot;
    ur5Direct(startJoint, startPos, startRot);

    Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(position, rotation);

    int *indexes = sort_inverse(inverse_kinematics_res, startJoint);

    for (int i = 0; i < 8; i++)
    {
        int index = indexes[i];
        jointValues joint_to_check;
        joint_to_check << inverse_kinematics_res(index, 0), inverse_kinematics_res(index, 1), inverse_kinematics_res(index, 2),
            inverse_kinematics_res(index, 3), inverse_kinematics_res(index, 4), inverse_kinematics_res(index, 5);

        joint_to_check = fixNormalization(bestNormalization(startJoint, joint_to_check));

        vector<double *> trajectory = vector<double *>();
        for (int i = 0; i < steps; i++)
        {
            trajectory.push_back(new double[6]);
        }

        if (init_verify_trajectory(&trajectory, startJoint, joint_to_check, steps, pick_or_place, position, rotation, homing, side_pick, isGripping))
        {
            if (debug_traj)
            {
                cout << "trajectory verified" << endl;
                cout << "joint_to_check: " << joint_to_check.transpose() << endl;
            }

            return trajectory;
        }
    }

    return vector<double *>();
}

vector<double *> calc_direct_traj_multiple(vector<pair<coordinates, rotMatrix>> poses_rots, bool *pick_or_place, bool *homing, jointValues startJoint, bool *side_pick, bool isGripping)
{
    vector<double *> trajectory_tot = vector<double *>();
    for (int i = 0; i < poses_rots.size(); i++)
    {
        jointValues currentJoints;
        if (i == 0)
        {
            currentJoints = startJoint;
        }
        else
        {
            currentJoints << trajectory_tot[trajectory_tot.size() - 1][0], trajectory_tot[trajectory_tot.size() - 1][1], trajectory_tot[trajectory_tot.size() - 1][2],
                trajectory_tot[trajectory_tot.size() - 1][3], trajectory_tot[trajectory_tot.size() - 1][4], trajectory_tot[trajectory_tot.size() - 1][5];
        }
        vector<double *> direct_traj = calc_direct_traj(poses_rots[i].first, poses_rots[i].second, pick_or_place[i], homing[i], currentJoints, side_pick[i], isGripping);
        if (direct_traj.size() > 0)
        {
            for (int j = 0; j < direct_traj.size(); j++)
            {
                trajectory_tot.push_back(direct_traj[j]);
            }
        }
        else
        {
            return vector<double *>();
        }
    }
    return trajectory_tot;
}

vector<double *> calc_traj(const coordinates &position, const rotMatrix &rotation, bool pick_or_place, bool homing, bool up_and_move_flag, bool move_to_near_axis_flag, jointValues startJoint, bool side_pick, bool isGripping, bool reset_flag)
{
    coordinates startPos;
    rotMatrix startRot;
    ur5Direct(startJoint, startPos, startRot);

    vector<double *> direct_traj = calc_direct_traj(position, rotation, pick_or_place, homing, startJoint, side_pick, isGripping);
    if (direct_traj.size() > 0)
    {
        cout << "### Success\n"
             << endl;
        return direct_traj;
    }
    else
    {
        cout << "§§§ Failed\n"
             << endl;
        if (position(0) * startPos(0) < 0 && !reset_flag)
        {
            cout << "### Calc reset&move: " << endl;

            vector<double *> reset_traj = reset_main_joint(position, rotation, steps, startJoint, side_pick, isGripping);
            if (reset_traj.size() > 0)
            {
                jointValues reset_joint;
                reset_joint << reset_traj[reset_traj.size() - 1][0], reset_traj[reset_traj.size() - 1][1], reset_traj[reset_traj.size() - 1][2],
                    reset_traj[reset_traj.size() - 1][3], reset_traj[reset_traj.size() - 1][4], reset_traj[reset_traj.size() - 1][5];
                vector<double *> traj_after_reset = calc_traj(position, rotation, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag, reset_joint, side_pick, isGripping, true);
                if (traj_after_reset.size() > 0)
                {
                    for (int i = 0; i < traj_after_reset.size(); i++)
                    {
                        reset_traj.push_back(traj_after_reset[i]);
                    }
                    cout << "### Success\n"
                         << endl;
                    return reset_traj;
                }
            }

            cout << "§§§ Failed\n"
                 << endl;
        }

        if (!up_and_move_flag)
        {
            vector<double *> up_and_move_traj = up_and_move(position, rotation, steps, startJoint, side_pick, isGripping);
            if (up_and_move_traj.size() > 0)
            {
                return up_and_move_traj;
            }
        }
        if (!move_to_near_axis_flag)
        {
            vector<double *> move_to_near_axis_traj = move_to_near_axis(position, rotation, pick_or_place, homing, startJoint, side_pick, isGripping);
            if (move_to_near_axis_traj.size() > 0)
            {
                return move_to_near_axis_traj;
            }
        }
        if (!homing)
        {
            vector<double *> move_through_homing_traj = move_through_homing(position, rotation, startJoint, side_pick, isGripping);
            if (move_through_homing_traj.size() > 0)
            {
                return move_through_homing_traj;
            }
        }
    }

    return vector<double *>();
}

vector<double *> calc_traj_multiple(vector<pair<coordinates, rotMatrix>> poses_rots, bool *pick_or_place, bool *homing, bool *up_and_move_flag, bool *move_to_near_axis_flag, jointValues startJoint, bool *side_pick, bool isGripping)
{
    vector<double *> trajectory_tot = vector<double *>();
    for (int i = 0; i < poses_rots.size(); i++)
    {
        jointValues currentJoints;
        if (i == 0)
        {
            currentJoints = startJoint;
        }
        else
        {
            currentJoints << trajectory_tot[trajectory_tot.size() - 1][0], trajectory_tot[trajectory_tot.size() - 1][1], trajectory_tot[trajectory_tot.size() - 1][2],
                trajectory_tot[trajectory_tot.size() - 1][3], trajectory_tot[trajectory_tot.size() - 1][4], trajectory_tot[trajectory_tot.size() - 1][5];
        }
        vector<double *> direct_traj = calc_traj(poses_rots[i].first, poses_rots[i].second, pick_or_place[i], homing[i], up_and_move_flag[i], move_to_near_axis_flag[i], currentJoints, side_pick[i], isGripping, false);
        if (direct_traj.size() > 0)
        {
            for (int j = 0; j < direct_traj.size(); j++)
            {
                trajectory_tot.push_back(direct_traj[j]);
            }
        }
        else
        {
            return vector<double *>();
        }
    }
    return trajectory_tot;
}

vector<double *> move_through_homing(coordinates final_cord, rotMatrix rot, jointValues startJoint, bool isGripping, bool side_pick)
{
    cout << "### Calc move through homing: " << endl;
    coordinates first_home;
    coordinates last_home;

    // cout << "final cord" << final_cord.transpose() << endl;

    coordinates startCord;
    rotMatrix startRot;
    ur5Direct(startJoint, startCord, startRot);

    first_home = nearHoming(startCord);
    last_home = nearHoming(final_cord);

    int homeOrder[] = {0, 5, 4, 3, 2, 1};
    int arrayDim = sizeof(homeOrder) / sizeof(homeOrder[0]);

    // if (debug_traj)
    // {

    //     for (int i = 0; i < arrayDim; i++)
    //     {
    //         cout << "homeOrder[" << i << "]: " << defaultCordArray[homeOrder[i]].transpose() << endl;
    //     }
    // }

    int first_home_index = -1;
    int last_home_index = -1;
    for (int i = 0; i < arrayDim; i++)
    {
        if (first_home(1) == defaultCordArray[homeOrder[i]](1) && first_home(0) == defaultCordArray[homeOrder[i]](0))
        {
            first_home_index = i;
        }
        if (last_home(1) == defaultCordArray[homeOrder[i]](1) && last_home(0) == defaultCordArray[homeOrder[i]](0))
        {
            last_home_index = i;
        }
    }
    // if (debug_traj)
    // {
    //     cout << "first_home_index: " << first_home_index << endl;
    //     cout << "last_home_index: " << last_home_index << endl;
    // }

    vector<pair<coordinates, rotMatrix>> positions = vector<pair<coordinates, rotMatrix>>();
    coordinates current = first_home;
    int current_index = first_home_index;
    while (current_index != last_home_index)
    {
        positions.push_back(make_pair(current, rot));
        if (current_index < last_home_index)
        {
            current_index++;
            current = defaultCordArray[homeOrder[current_index]];
        }
        else
        {
            current_index--;
            current = defaultCordArray[homeOrder[current_index]];
        }
    }

    positions.push_back(make_pair(last_home, rot));
    positions.push_back(make_pair(final_cord, rot));

    if (debug_traj)
    {
        cout << "current_cord" << startCord.transpose() << endl;
        cout << "positions.size(): " << positions.size() << endl;
        for (int i = 0; i < positions.size(); i++)
        {
            cout << "positions[" << i << "]: " << positions[i].first.transpose() << endl;
        }
    }

    bool *pick_or_place = new bool[positions.size()];
    bool *homing = new bool[positions.size()];
    bool *side_picks = new bool[positions.size()];

    for (int i = 0; i < positions.size(); i++)
    {
        pick_or_place[i] = false;
        homing[i] = false;
        side_picks[i] = false;
    }
    side_picks[positions.size() - 1] = side_pick;

    vector<double *> move_through_homing_traj = calc_direct_traj_multiple(positions, pick_or_place, homing, startJoint, side_picks, isGripping);
    if (move_through_homing_traj.size() > 0)
    {
        cout << "### Success\n"
             << endl;
        return move_through_homing_traj;
    }
    else
    {
        cout << "§§§ Failed\n"
             << endl;
        return vector<double *>();
    }
}

vector<double *> move_to_near_axis(const coordinates &position, const rotMatrix &rotation, bool pick_or_place, bool homing, jointValues startJoint, bool side_pick, bool isGripping)
{
    cout << "### Calc move to near axis: " << endl;
    coordinates currentPos;
    rotMatrix currentRot;
    ur5Direct(startJoint, currentPos, currentRot);

    coordinates nearAxis;
    if (abs(currentPos(1) - position(1)) < abs(currentPos(0) - position(0)))
    {
        nearAxis << currentPos(0), position(1), currentPos(2);
    }
    else
    {
        nearAxis << position(0), currentPos(1), currentPos(2);
    }

    vector<pair<coordinates, rotMatrix>> positions = vector<pair<coordinates, rotMatrix>>();

    positions.push_back(make_pair(nearAxis, rotation));
    positions.push_back(make_pair(position, rotation));

    bool pick_or_place_array[] = {false, pick_or_place};
    bool homing_array[] = {homing, homing};
    bool side_pick_array[] = {false, side_pick};

    vector<double *> move_to_near_axis_traj = calc_direct_traj_multiple(positions, pick_or_place_array, homing_array, startJoint, side_pick_array, isGripping);
    if (move_to_near_axis_traj.size() > 0)
    {
        cout << "### Success\n"
             << endl;
        return move_to_near_axis_traj;
    }
    else
    {
        cout << "§§§ Failed\n"
             << endl;
        return vector<double *>();
    }
}

coordinates nearHomingRec(coordinates current_cord, coordinates defaultCord, double &nearhomingdist, coordinates &nearhomingcord)
{
    // cout << "defaultCord: " << defaultCord.transpose() << endl;
    double dist = sqrt(pow(current_cord(0) - defaultCord(0), 2) + pow(current_cord(1) - defaultCord(1), 2));
    if (nearhomingdist == -1 || dist < nearhomingdist)
    {
        nearhomingdist = dist;
        nearhomingcord = defaultCord;
    }
    return nearhomingcord;
}

coordinates nearHoming(coordinates cord)
{

    // cout << "nearHoming" << endl;

    double nearhomingdist = -1;
    coordinates nearhomingcord;
    for (coordinates defaultCord : defaultCordArray)
    {
        nearhomingcord = nearHomingRec(cord, defaultCord, nearhomingdist, nearhomingcord);
    }
    if (debug_traj)
    {
        cout << "nearhomingcord: " << nearhomingcord.transpose() << endl;
    }
    return nearhomingcord;
}

rotMatrix get_rotation(double angle)
{
    // transform the angle in radians(0-2pi)
    angle = angle * M_PI / 180;

    coordinates cord;
    rotMatrix rot;
    jointValues joints;
    joints << -0.00322469, -0.549783, -2.21282, 5.89289, -3.13054, 0;
    joints(4) = angle;
    ur5Direct(joints, cord, rot);
    return rot;
}