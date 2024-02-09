#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <algorithm>

using namespace std;

double *Controller::test_traj_multiple(vector<pair<coordinates, rotMatrix>> poses_rots, bool *pick_or_place, bool *homing, bool *up_and_move_flag, bool *move_to_near_axis_flag, bool *side_picks_flag, jointValues start_joints, bool isGripping)
{
    vector<double *> trajectory = calc_traj_multiple(poses_rots, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag, start_joints, side_picks_flag, isGripping);
    cout << "Trajectory size: " << trajectory.size() << endl;
    if (trajectory.size() > 0)
    {
        return trajectory[trajectory.size() - 1];
    }
    return NULL;
}

double *Controller::test_traj(coordinates position, rotMatrix rotation, bool pick_or_place, bool homing, bool up_and_move_flag, bool move_to_near_axis_flag, jointValues start_joints, bool isGripping)
{
    vector<double *> trajectory = calc_traj(position, rotation, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag, start_joints, false, isGripping, false);
    cout << "Trajectory size: " << trajectory.size() << endl;
    if (trajectory.size() > 0)
    {
        return trajectory[trajectory.size() - 1];
    }
    return NULL;
}

double *Controller::test_traj_multiple_side_pick(vector<pair<coordinates, rotMatrix>> poses_rots, bool *pick_or_place, bool *homing, bool *up_and_move_flag, bool *move_to_near_axis_flag, bool *side_picks_flag, jointValues start_joints, bool isGripping)
{
    pair<coordinates, rotMatrix> last_pose = poses_rots[poses_rots.size() - 1];
    cout << "last_pose: " << last_pose.first.transpose() << endl;
    cout << "last_pose_rotation: " << last_pose.second << endl;
    Quaterniond q = Quaterniond(last_pose.second);
    // set rotation
    Vector3d euler = q.normalized().toRotationMatrix().eulerAngles(2, 0, 2);
    Vector3d side_pick_euler;
    side_pick_euler << -M_PI_2, 0, M_PI;

    AngleAxisd toSidePick_Z(M_PI, Vector3d::UnitZ());
    AngleAxisd toSidePick_Y(-M_PI_2, Vector3d::UnitX());

    Quaterniond side_pick_rot = q * toSidePick_Z * toSidePick_Y;
    // Overwrite last pose
    cout << "side_pick_euler: " << side_pick_euler << endl;
    for (int i = 0; i < poses_rots.size(); i++)
    {
        if (side_picks_flag[i])
        {
            cout << " --------------- Side pick\n"
                 << endl
                 << "BLOCK POS: " << poses_rots[i].first.transpose() << endl;

            coordinates block_pos = poses_rots[i].first;
            coordinates side_pick_pos;
            side_pick_pos << block_pos(0) + sin(euler(2)) * 0.02, block_pos(1) + cos(euler(2)) * 0.02, block_pos(2) - 0.05;
            poses_rots[i].second = side_pick_rot.normalized().toRotationMatrix();
            poses_rots[i].first = side_pick_pos;
        }
    }
    vector<double *> trajectory = calc_traj_multiple(poses_rots, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag, start_joints, side_picks_flag, isGripping);
    cout << "Trajectory size: " << trajectory.size() << endl;
    if (trajectory.size() > 0)
    {
        return trajectory[trajectory.size() - 1];
    }
    return NULL;
}

double *Controller::test_traj_side_pick(coordinates position, rotMatrix rotation, bool pick_or_place, bool homing, bool up_and_move_flag, bool move_to_near_axis_flag, jointValues start_joints, bool isGripping)
{
    Quaterniond q = Quaterniond(rotation);
    // set rotation
    Vector3d euler = q.normalized().toRotationMatrix().eulerAngles(2, 0, 2);
    Vector3d side_pick_euler;
    side_pick_euler << -M_PI_2, 0, M_PI;

    AngleAxisd toSidePick_Z(M_PI, Vector3d::UnitZ());
    AngleAxisd toSidePick_Y(-M_PI_2, Vector3d::UnitX());

    Quaterniond side_pick_rot = q * toSidePick_Z * toSidePick_Y;
    // Overwrite last pose
    cout << " --------------- Side pick\n"
         << endl
         << "BLOCK POS: " << position.transpose() << endl;

    coordinates block_pos = position;
    coordinates side_pick_pos;
    side_pick_pos << block_pos(0) + sin(euler(2)) * 0.02, block_pos(1) + cos(euler(2)) * 0.02, block_pos(2) - 0.05;
    rotation = side_pick_rot.normalized().toRotationMatrix();
    position = side_pick_pos;

    vector<double *> trajectory = calc_traj(position, rotation, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag, start_joints, true, isGripping, false);
    cout << "Trajectory size: " << trajectory.size() << endl;
    if (trajectory.size() > 0)
    {
        return trajectory[trajectory.size() - 1];
    }
    return NULL;
}

void Controller::print_current_pos_rot()
{
    coordinates cord;
    rotMatrix rot;
    jointValues joints;
    joints = current_joints;
    ur5Direct(joints, cord, rot);
    cout << "cord: " << cord << endl;
    cout << "rot: " << rot << endl;
}

void Controller::sleep()
{
    ros::Duration(1.0).sleep();
}

Controller::Controller(double loop_frequency) : loop_rate(loop_frequency)
{
    this->loop_frequency = loop_frequency;
}
