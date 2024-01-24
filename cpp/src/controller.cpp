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
        move_to(defaultCordArray[0], rotDefault, false, true, false, false);
    }
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

bool Controller::move_inside(vector<double *> *trajectory)
{
    int steps = trajectory->size();
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

            jointValues q_des = fixNormalization(bestNormalization(current_joints, second_order_filter(des_optimal, loop_frequency, 0.5)));
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

bool Controller::move_to(const coordinates &position, const rotMatrix &rotation, bool pick_or_place, bool homing, bool up_and_move_flag, bool move_to_near_axis_flag)
{
    cout << "Requested move_to with position: " << position.transpose() << endl;

    vector<double *> trajectory = calc_traj(position, rotation, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag, current_joints);
    if (trajectory.size() > 0)
    {
        if (move_inside(&trajectory))
        {
            return true;
        }
    }
    cout << "\n°°°°°°°°°°\n"
         << "Move to §§§ Failed\n"
         << "\n°°°°°°°°°°\n"
         << endl;

    ros::Duration(10.0).sleep();
    return false;
}

bool Controller::move_to_multiple(vector<pair<coordinates, rotMatrix>> poses_rots, bool *pick_or_place, bool *homing, bool *up_and_move_flag, bool *move_to_near_axis_flag)
{
    vector<double *> trajectory = calc_traj_multiple(poses_rots, pick_or_place, homing, up_and_move_flag, move_to_near_axis_flag, current_joints);
    if (trajectory.size() > 0)
    {
        if (move_inside(&trajectory))
        {
            return true;
        }
    }
    cout << "\n°°°°°°°°°°\n"
         << "Move to §§§ Failed\n"
         << "\n°°°°°°°°°°\n"
         << endl;

    ros::Duration(10.0).sleep();
    return false;
}


