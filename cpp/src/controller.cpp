#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <algorithm>

using namespace std;

void Controller::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_initialized = true;
    // cout << "joint_state_callback" << endl;
    int n = (real_robot) ? 6 : 8;

    if (n > 6)
    {
        current_gripper << msg->position[1], msg->position[2];
    }
    current_joints << msg->position[4], msg->position[3], msg->position[0], msg->position[5], msg->position[6], msg->position[7];
    // cout << "current_joints " << current_joints << endl;
}

Controller::Controller(double loop_frequency, bool start_homing) : loop_rate(loop_frequency)
{
    this->loop_frequency = loop_frequency;
    node.getParam("/real_robot", real_robot);
    node.getParam("/gripper_sim", gripper_sim);

    sub_joint_state = node.subscribe("/ur5/joint_states", 1, &Controller::joint_state_callback, this);
    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);
    pub_gripper_diameter = node.advertise<std_msgs::Int32>("/ur5/gripper_controller/command", 1);
    permission_pub = node.advertise<std_msgs::Bool>("/computer_vision/permission", 1);

    while (!joint_initialized)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (start_homing)
    {

        move_to(defaultCordArray[0], rotDefault, steps, false, true, false, false);
    }
}

coordinates Controller::nearHomingRec(coordinates current_cord, coordinates defaultCord, double &nearhomingdist, coordinates &nearhomingcord)
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

coordinates Controller::nearHoming(coordinates cord)
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

coordinates Controller::advanceNearHomingRec(coordinates current_cord, coordinates defaultCord, coordinates final_cord, double &nearhomingdist, coordinates &nearhomingcord)
{
    double dist = sqrt(pow(current_cord(0) - defaultCord(0), 2) + pow(current_cord(1) - defaultCord(1), 2));
    double dist2 = sqrt(pow(final_cord(0) - defaultCord(0), 2) + pow(final_cord(1) - defaultCord(1), 2));
    if (nearhomingdist == -1 || (dist + dist2) < nearhomingdist)
    {
        nearhomingdist = dist + dist2;
        nearhomingcord = defaultCord;
    }
    return nearhomingcord;
}

void Controller::advanceNearHoming(coordinates &cord, rotMatrix &rot, coordinates final_cord)
{

    // cout << "advanceNearHoming" << endl;

    coordinates current_cord;
    ur5Direct(current_joints, current_cord, rot);
    double nearhomingdist = -1;
    coordinates nearhomingcord;
    for (coordinates defaultCord : defaultCordArray)
    {
        nearhomingcord = advanceNearHomingRec(current_cord, defaultCord, final_cord, nearhomingdist, nearhomingcord);
    }

    // cout << "nearhomingcord: " << nearhomingcord << endl;
    cord = nearhomingcord;
}

void Controller::send_state(const jointValues &joint_pos)
{

    std_msgs::Float64MultiArray joint_state_msg_array;
    if (real_robot)
    {
        joint_state_msg_array.data.resize(6);
    }
    else
    {
        joint_state_msg_array.data.resize(8);
        joint_state_msg_array.data[6] = current_gripper(0);
        joint_state_msg_array.data[7] = current_gripper(1);
    }

    for (int i = 0; i < 6; i++)
    {
        joint_state_msg_array.data[i] = joint_pos(i);
    }
    /*
    for (int i = 0; i < 8; i++)
    {
        cout << "joint_state_msg_array.data[" << i << "] " << joint_state_msg_array.data[i] << endl;

    }
    */

    pub_des_jstate.publish(joint_state_msg_array);
}

void Controller::sent_gripper_diameter(const int diameter)
{
    if (test_fast_mode)
    {
        cout << "fast test mode, skip moving gripper" << endl;
        return;
    }
    std::cout << "moving gripper to diameter " << diameter << std::endl;
    std_msgs::Int32 gripper_diameter_msg;
    gripper_diameter_msg.data = diameter;
    pub_gripper_diameter.publish(gripper_diameter_msg);
    ros::Duration(sleep_time_after_gripper).sleep();
    cout << "gripper moved\n-----------------------\n"
         << endl;
}

jointValues Controller::get_joint_state()
{
    ros::spinOnce();
    return current_joints;
}

GripperStateVector Controller::get_gripper_state()
{
    ros::spinOnce();
    return current_gripper;
}

pair<coordinates, rotMatrix> Controller::get_position()
{
    coordinates cord;
    rotMatrix rotation;
    ur5Direct(current_joints, cord, rotation);
    return make_pair(cord, rotation);
}

void Controller::init_filter(void)
{
    if (debug_traj)
    {
        cout << "init linear filter" << endl;
    }
    filter_1 = current_joints;
    filter_2 = current_joints;
}

jointValues Controller::second_order_filter(const jointValues &input, const double rate, const double settling_time)
{

    double dt = 1 / rate;
    double gain = dt / (0.1 * settling_time + dt);
    filter_1 = (1 - gain) * filter_1 + gain * input;
    filter_2 = (1 - gain) * filter_2 + gain * filter_1;
    return filter_2;
}

bool Controller::move_to_near_axis(const coordinates &position, const rotMatrix &rotation, int steps, bool pick_or_place, bool homing)
{
    cout << "^^^^^^\nTrying move to near axis" << endl;
    coordinates currentPos = get_position().first;
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

    vector<vector<double *>> th_sum = vector<vector<double *>>();
    for (int j = 0; j < positions.size(); j++)
    {
        vector<double *> trajectory = vector<double *>();
        for (int i = 0; i < steps; i++)
        {
            trajectory.push_back(new double[6]);
        }
        th_sum.push_back(trajectory);
    }

    jointValues init_joint;
    init_joint = current_joints;

    if (trajectory_multiple_positions(&th_sum, &positions, positions.size(), 0, init_joint, vector<bool>{false, false}, steps))
    {
        vector<double *> th_sum_vector = vector<double *>();
        for (int i = 0; i < positions.size(); i++)
        {
            for (int j = 0; j < steps; j++)
            {
                th_sum_vector.push_back(th_sum[i][j]);
            }
        }
        if (move_inside(steps * positions.size(), &th_sum_vector))
        {
            return true;
        }
    }

    cout << "\nMove to near axis failed" << endl;
    return false;
}

bool Controller::up_and_move(const coordinates &position, const rotMatrix &rotation, int steps, jointValues init_joint)
{
    cout << "^^^^^^\nTrying up and move" << endl;
    coordinates currentPos = get_position().first;
    double new_z = currentPos(2) - 0.15;
    if (new_z < 0.55)
    {
        new_z = 0.55;
    }
    // if difference in new_z and currentPos(2) is too small, don't move up
    if (abs(new_z - currentPos(2)) < 0.03)
    {
        return false;
    }
    coordinates aboveCurrent, aboveNext;
    aboveCurrent << currentPos(0), currentPos(1), new_z;
    aboveNext << position(0), position(1), new_z;

    if (move_to(aboveCurrent, rotation, steps, true, false, true, false))
    {
        init_joint = current_joints;
        vector<pair<coordinates, rotMatrix>> positions = vector<pair<coordinates, rotMatrix>>();

        positions.push_back(make_pair(aboveNext, rotation));
        positions.push_back(make_pair(position, rotation));

        vector<vector<double *>> th_sum = vector<vector<double *>>();
        for (int j = 0; j < positions.size(); j++)
        {
            vector<double *> trajectory = vector<double *>();
            for (int i = 0; i < steps; i++)
            {
                trajectory.push_back(new double[6]);
            }
            th_sum.push_back(trajectory);
        }

        if (trajectory_multiple_positions(&th_sum, &positions, positions.size(), 0, init_joint, vector<bool>{false, true}, steps))
        {
            vector<double *> th_sum_vector = vector<double *>();
            for (int i = 0; i < positions.size(); i++)
            {
                for (int j = 0; j < steps; j++)
                {
                    th_sum_vector.push_back(th_sum[i][j]);
                }
            }

            // // print trajectory
            // for (int i = 0; i < th_sum_vector.size(); i++)
            // {
            //     cout << "th_sum_vector[" << i << "]: " << th_sum_vector[i][0] << ", " << th_sum_vector[i][1] << ", " << th_sum_vector[i][2] << ", " << th_sum_vector[i][3] << ", " << th_sum_vector[i][4] << ", " << th_sum_vector[i][5] << endl;
            // }

            if (move_inside(steps * positions.size(), &th_sum_vector))
            {
                return true;
            }
        }
    }

    cout << "\nUp and move failed" << endl;
    return false;
}

bool Controller::reset_main_joint(const coordinates &position, const rotMatrix &rotation, int steps, bool homing)
{
    jointValues reset_values;
    reset_values = mainJointResetValues;
    reset_values(0) = current_joints(0);
    reset_values(3) = current_joints(3);
    reset_values(4) = current_joints(4);
    reset_values(5) = current_joints(5);

    if (current_joints(2) > 0)
    {
        reset_values(2) = 2.2;
    }
    else
    {
        reset_values(2) = -2.2;
    }

    double angle;

    angle = atan2(position(1), position(0));

    // fancy trignometry to get the angle between the coordinates to reach and the point x=0 y=0
    if (position(0) > 0)
    {
        if (position(1) > 0)
        {
            angle += M_PI;
        }
        else
        {
            angle += M_PI * 3 / 2;
        }
    }
    else
    {
        if (position(1) > 0)
        {
            angle = angle;
        }
        else
        {
            angle -= M_PI / 2;
        }
    }

    // check if joint 1 in on the right or left of the robot
    if (current_joints(1) < -M_PI / 2)
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
        if (current_joints(1) < -M_PI / 2)
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

        while (angle > current_joints(0))
        {
            angle -= 2 * M_PI;
        }

        while (angle + 2 * M_PI < current_joints(0))
        {
            angle += 2 * M_PI;
        }

        cout << "reset anti clockwise" << endl;
    }
    else
    {
        while (angle < current_joints(0))
        {
            angle += 2 * M_PI;
        }
        while (angle - 2 * M_PI > current_joints(0))
        {
            angle -= 2 * M_PI;
        }

        cout << "reset clockwise" << endl;
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

        vector<vector<double *>> th_sum = vector<vector<double *>>();
        for (int j = 0; j < joints.size(); j++)
        {
            vector<double *> trajectory = vector<double *>();
            for (int i = 0; i < steps; i++)
            {
                trajectory.push_back(new double[6]);
            }
            th_sum.push_back(trajectory);
        }

        jointValues init_joint;
        init_joint = current_joints;

        if (trajectory_multiple_positions_joints(&th_sum, &joints, joints.size(), 0, init_joint, vector<bool>{false, false}, steps))
        {
            vector<double *> th_sum_vector = vector<double *>();
            for (int i = 0; i < joints.size(); i++)
            {
                for (int j = 0; j < steps; j++)
                {
                    th_sum_vector.push_back(th_sum[i][j]);
                }
            }
            if (move_inside(steps * joints.size(), &th_sum_vector))
            {
                return true;
            }
        }
    }

    return false;
}

bool Controller::move_to(const coordinates &position, const rotMatrix &rotation, int steps, bool pick_or_place, bool homing, bool up_and_move_flag, bool move_to_near_axis_flag)
{

    cout << "\n######################\n";
    cout << "Requested move to " << position.transpose() << endl;
    cout << "Current position: " << get_position().first.transpose() << endl;
    cout << "Current joints: " << current_joints.transpose() << endl;
    // if x have a different signì
    if (position(0) * get_position().first(0) < 0)
    {

        cout << "   reset required" << endl;

        if (reset_main_joint(position, rotation, steps, homing))
        {
            if (debug_traj)
            {
                cout << "   reset done" << endl;
            }
        }
        else
        {

            cout << "   !!!!!!!\n   reset failed\n  !!!!!!!!" << endl;
        }
    }

    jointValues init_joint = current_joints;

    Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(position, rotation);
    // cout << "inverse_kinematics_res:\n " << inverse_kinematics_res.transpose() << endl;

    int *indexes = sort_inverse(inverse_kinematics_res, init_joint);

    for (int i = 0; i < 8; i++)
    {
        // check if the trajectory is valid
        int index = indexes[i];
        jointValues joint_to_check;
        joint_to_check << inverse_kinematics_res(index, 0), inverse_kinematics_res(index, 1), inverse_kinematics_res(index, 2),
            inverse_kinematics_res(index, 3), inverse_kinematics_res(index, 4), inverse_kinematics_res(index, 5);

        joint_to_check = fixNormalization(bestNormalization(init_joint, joint_to_check));
        coordinates cord_calc;
        rotMatrix rotation_calc;
        ur5Direct(joint_to_check, cord_calc, rotation_calc);
        // cout << "cord_calc: " << cord_calc.transpose() << endl;
        /*
        cout << "joint_to_check: " << joint_to_check << endl;
        cout << "init_joint: " << init_joint << endl;
        cout << "init gripper" << current_gripper << endl;
        */

        // cout << "joint_to_check normalized: " << joint_to_check.transpose() << endl;

        vector<double *> trajectory = vector<double *>();
        for (int i = 0; i < steps; i++)
        {
            trajectory.push_back(new double[6]);
        }

        if (init_verify_trajectory(&trajectory, init_joint, joint_to_check, steps, pick_or_place, position, rotation, homing))
        {
            if (debug_traj)
            {
                cout << "trajectory verified" << endl;
                cout << "joint_to_check: " << joint_to_check.transpose() << endl;
            }
            return move_inside(steps, &trajectory);
        }
    }
    if (debug_traj)
    {
        cout << "No valid trajectory found" << endl;
    }

    if (!up_and_move_flag)
    {
        if (up_and_move(position, rotation, steps, init_joint))
        {
            return true;
        }
    }

    // if (!move_to_near_axis_flag)
    // {
    //     if (move_to_near_axis(position, rotation, steps, pick_or_place, homing))
    //     {
    //         return true;
    //     }
    // }

    if (!homing)
    {

        cout << "^^^^^^\nTrying move through homing" << endl;
        if (move_through_homing(position, rotation))
        {
            return true;
        }
        cout << "\nMove through homing failed" << endl;
    }
    cout << "\n°°°°°°°°°°\n"
         << "Move to failed"
         << "\n°°°°°°°°°°\n"
         << endl;

    ros::Duration(2.0).sleep();

    return false;
}

bool Controller::move_to_joint(jointValues joint_to_reach, int steps, bool pick_or_place, bool homing)
{
    jointValues init_joint;
    init_joint = current_joints;

    coordinates cord_calc;
    rotMatrix rotation_calc;
    ur5Direct(joint_to_reach, cord_calc, rotation_calc);

    vector<double *> trajectory = vector<double *>();
    for (int i = 0; i < steps; i++)
    {
        trajectory.push_back(new double[6]);
    }

    if (init_verify_trajectory(&trajectory, init_joint, joint_to_reach, steps, pick_or_place, cord_calc, rotation_calc, homing))
    {
        if (debug_traj)
        {
            cout << "trajectory verified" << endl;
        }
        return move_inside(steps, &trajectory);
    }

    if (debug_traj)
    {
        cout << "No valid trajectory found" << endl;
    }
    return false;
}

bool Controller::move_inside(int steps, vector<double *> *trajectory)
{
    cout << "steps: " << steps << endl;
    coordinates cord;
    rotMatrix rotation;
    jointValues final_joint;
    final_joint << (*trajectory)[steps - 1][0], (*trajectory)[steps - 1][1], (*trajectory)[steps - 1][2],
        (*trajectory)[steps - 1][3], (*trajectory)[steps - 1][4], (*trajectory)[steps - 1][5];
    ur5Direct(final_joint, cord, rotation);

    init_filter();

    cout << "\n------------\nStart moving to position: " << cord.transpose() << endl;
    cout << "Final joint: " << final_joint.transpose() << endl;
    bool use_filter = true;
    if (test_fast_mode)
    {
        use_filter = false;
    }

    if (!use_filter)
    {
        cout << "__________\n NOT USING FILTER\n__________"
             << endl;
    }
    for (int j = 0; j < steps; j++)
    {
        // get i element of variable "trajectory" and put it in new variable
        jointValues des_not_linear;
        double *traj_j = (*trajectory)[j];
        des_not_linear << traj_j[0], traj_j[1], traj_j[2],
            traj_j[3], traj_j[4], traj_j[5];

        // des_not_linear = bestNormalization(current_joints, des_not_linear);

        // cout << "des_not_linear: " << des_not_linear << endl;
        //  send the trajectory
        int counter = 0;

        while (ros::ok() && this->acceptable_error < calculate_distance(current_joints, des_not_linear))
        {
            // cout << "error: " << calculate_distance(current_joints, des_not_linear) << endl;

            jointValues des_optimal, des_normalized;
            des_normalized = bestNormalization(current_joints, des_not_linear);
            des_optimal = fixNormalization(des_normalized);

            // if (des_normalized(0) > 6.14 || des_normalized(0) < -6.14)
            // {
            //     cout << "des_optimal: " << des_optimal.transpose() << endl;
            //     jointValues des_reset_main_axis;
            //     des_reset_main_axis << des_optimal(0), mainJointResetValues(1), mainJointResetValues(2),
            //         current_joints(3), current_joints(4), current_joints(5);
            //     cout << "##########################" << endl;
            //     cout << "special move to joints:" << des_reset_main_axis.transpose() << endl;
            //     move_to_joint(des_reset_main_axis, steps, false, true);
            // }

            jointValues q_des = fixNormalization(bestNormalization(current_joints, second_order_filter(des_optimal, loop_frequency, 0.5)));
            // cout << "q_des: " << q_des << endl;
            if (use_filter)
            {
                send_state(q_des);
            }
            else
            {
                send_state(des_optimal);
            }
            this->loop_rate.sleep();

            ros::spinOnce();
            counter++;
            if (counter >= 1000)
            {
                cout << "error: " << calculate_distance(current_joints, des_not_linear) << endl;
                cout << "Cant reach after" << counter << " iterations" << endl;

                coordinates cord;
                rotMatrix rotation;
                ur5Direct(current_joints, cord, rotation);
                cout << "current cord: " << cord.transpose() << endl;

                ur5Direct(des_not_linear, cord, rotation);
                cout << "des_not_linear cord: " << cord.transpose() << endl;

                if (use_filter)
                {
                    cout << "current_joints: " << current_joints.transpose() << endl;
                    cout << "q_des: " << q_des.transpose() << endl;
                }
                else
                {
                    cout << "current_joints: " << current_joints.transpose() << endl;
                    cout << "des_not_linear: " << des_not_linear.transpose() << endl;
                    cout << "des_optimal: " << des_optimal.transpose() << endl;
                }

                break;
            }
        }
    }

    cout << "Done\n------------\n"
         << endl;
    ros::Duration(sleep_time_after_movement).sleep();
    return true;
}

bool Controller::move_through_homing(coordinates final_cord, rotMatrix rot)
{
    coordinates first_home;
    coordinates last_home;

    // cout << "final cord" << final_cord.transpose() << endl;

    coordinates current_cord;
    ur5Direct(current_joints, current_cord, rot);

    first_home = nearHoming(current_cord);
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

    vector<vector<double *>> th_sum = vector<vector<double *>>();
    for (int j = 0; j < positions.size(); j++)
    {
        vector<double *> trajectory = vector<double *>();
        for (int i = 0; i < steps; i++)
        {
            trajectory.push_back(new double[6]);
        }
        th_sum.push_back(trajectory);
    }
    if (debug_traj)
    {
        cout << "current_cord" << current_cord.transpose() << endl;
        cout << "positions.size(): " << positions.size() << endl;
        for (int i = 0; i < positions.size(); i++)
        {
            cout << "positions[" << i << "]: " << positions[i].first.transpose() << endl;
        }
    }

    vector<bool> order = vector<bool>();
    for (int i = 0; i < positions.size(); i++)
    {
        order.push_back(false);
    }

    if (trajectory_multiple_positions(&th_sum, &positions, positions.size(), 0, current_joints, order, this->steps))
    {
        cout << "-------------------------\n"
             << endl;
        for (int i = 0; i < positions.size(); i++)
        {

            for (int j = 0; j < steps; j++)
            {
                jointValues joint_to_check;
                joint_to_check << th_sum[i][j][0], th_sum[i][j][1], th_sum[i][j][2],
                    th_sum[i][j][3], th_sum[i][j][4], th_sum[i][j][5];

                coordinates cord;
                rotMatrix rotation;
                ur5Direct(joint_to_check, cord, rotation);

                // cout << "cord: " << cord.transpose() << endl;
            }
        }

        // cout << "move through homing" << endl;

        // convert th_sum from vector<vector<double *>> to vector<double *> in order to do a single move
        vector<double *> th_sum_vector = vector<double *>();
        for (int i = 0; i < positions.size(); i++)
        {
            for (int j = 0; j < steps; j++)
            {
                th_sum_vector.push_back(th_sum[i][j]);
            }
        }
        return move_inside(steps * positions.size(), &th_sum_vector);
    }
    return false;
}

void Controller::move_gripper_to(const int diameter)
{

    sent_gripper_diameter(diameter);
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