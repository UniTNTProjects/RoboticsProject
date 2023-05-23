#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>
#include "kinematics.h"

typedef Eigen::Matrix<double, 2, 1> GripperStateVector;

class Controller
{
private:
    ros::NodeHandle node;
    ros::Rate loop_rate;

    bool real_robot;
    bool gripper_sim;

    jointValues filter_1;
    jointValues filter_2;

    double loop_frequency;
    ros::Publisher pub_des_jstate;
    ros::Publisher pub_gripper_diameter;
    ros::Subscriber sub_joint_state;

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void send_state(const jointValues &joint_pos);
    void sent_gripper_diameter(const int diameter);
    bool check_trajectory(vector<double *> traj, int step);
    jointValues second_order_filter(const jointValues &input, const double rate, const double settling_time);
    void init_filter(void);
    double compute_error(const jointValues &first_vector, const jointValues &second_vector);
    int *sort_ik_result(const Eigen::Matrix<double, 8, 6> &ik_result, const jointValues &initial_joints);

public:
    Controller(double loop_frequency);
    jointValues get_joint_state();
    GripperStateVector get_gripper_state();
    bool move_to(const coordinates &position, const rotMatrix &rotation, int steps);
    void move_gripper_to(const int diameter);
    void print_current_pos_rot();
};

#endif
