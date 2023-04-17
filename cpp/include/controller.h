#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>

typedef Eigen::Matrix<double, 6, 1> JointStateVector;
typedef Eigen::Matrix<double, 2, 1> GripperStateVector;

class Controller
{
private:
    ros::NodeHandle node;

    bool real_robot;
    bool gripper_sim;

    double loop_time = 0.;
    double loop_frequency = 1000.;
    ros::Publisher pub_des_jstate;
    ros::Publisher pub_gripper_diameter;
    ros::Subscriber sub_joint_state;

    JointStateVector current_joints;
    GripperStateVector current_gripper;

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void send_state(const JointStateVector &joint_pos);
    void sent_gripper_diameter(const int diameter);

public:
    Controller();
    JointStateVector get_joint_state();
    GripperStateVector get_gripper_state();
    void move_to(const int wip);
    void move_gripper_to(const int diameter);
};

#endif
