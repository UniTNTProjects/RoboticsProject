#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

void Controller::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    int n = (real_robot) ? 6 : 8;

    for (int i = 0; i < n; i++)
    {
        if (i >= 6)
        {
            std::cout << "current gripper n°" << i - 6 << ": " << msg->position[i] << std::endl;
            current_gripper(i - 6) = msg->position[i];
        }
        else
        {
            std::cout << "current joint n°" << i << ": " << msg->position[i] << std::endl;
            current_joints(i) = msg->position[i];
        }
    }
}

Controller::Controller()
{

    node.getParam("/real_robot", real_robot);
    node.getParam("/gripper_sim", gripper_sim);

    sub_joint_state = node.subscribe("/ur5/joint_states", 1, &Controller::joint_state_callback, this);
    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);
    pub_gripper_diameter = node.advertise<std_msgs::Int32>("/ur5/gripper_controller/command", 1);

    ros::Rate loop_rate(loop_frequency);
}

void Controller::send_state(const JointStateVector &joint_pos)
{
    std::cout << "q_des " << joint_pos.transpose() << std::endl;

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
        joint_state_msg_array.data[i] = joint_pos[i];
    }

    pub_des_jstate.publish(joint_state_msg_array);
}

void Controller::sent_gripper_diameter(const int diameter)
{
    std::cout << "moving gripper to diameter " << diameter << std::endl;
    std_msgs::Int32 gripper_diameter_msg;
    gripper_diameter_msg.data = diameter;
    pub_gripper_diameter.publish(gripper_diameter_msg);
    ros::Duration(3.0).sleep();
}

JointStateVector Controller::get_joint_state()
{
    ros::spinOnce();
    return current_joints;
}

GripperStateVector Controller::get_gripper_state()
{
    ros::spinOnce();
    return current_gripper;
}

void Controller::move_to(const int wip)
{
    JointStateVector q_des;
    q_des << 1, 1, 1, 1, 1, 1;
    send_state(q_des);
}

void Controller::move_gripper_to(const int diameter)
{
    sent_gripper_diameter(diameter);
}