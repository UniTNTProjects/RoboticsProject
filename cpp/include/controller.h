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

    double acceptable_error = 0.05;

    double max_x = 1;
    double max_y = 1;
    double max_z = 0.8;

    double min_x = -1;
    double min_y = -1;
    double min_z = -1;

    jointValues current_joints;
    GripperStateVector current_gripper;
    bool joint_initialized = false;

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void send_state(const jointValues &joint_pos);
    void sent_gripper_diameter(const int diameter);
    bool check_trajectory(vector<double *> traj, int step);
    jointValues second_order_filter(const jointValues &input, const double rate, const double settling_time);
    void init_filter(void);
    double calculate_distance(const jointValues &first_vector, const jointValues &second_vector);
    int *sort_inverse(Eigen::Matrix<double, 8, 6> &inverse_kinematics_res, const jointValues &initial_joints);
    double norm_angle(double angle);

public:
    Controller(double loop_frequency);
    jointValues get_joint_state();
    GripperStateVector get_gripper_state();
    pair<coordinates, rotMatrix> get_position();
    bool move_to(const coordinates &position, const rotMatrix &rotation, int steps);
    void move_gripper_to(const int diameter);
    void print_current_pos_rot();
};

#endif
