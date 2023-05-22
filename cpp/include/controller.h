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

    double loop_time = 0.;
    double loop_frequency = 1000.;
    ros::Publisher pub_des_jstate;
    ros::Publisher pub_gripper_diameter;
    ros::Subscriber sub_joint_state;

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void send_state(const jointValues &joint_pos);
    void sent_gripper_diameter(const int diameter);
    bool check_trajectory(vector<double *> traj, int step);
    jointValues linear_filter_calc(const jointValues &joints_des);
    void init_linear_filter(void);
    double compute_error(const jointValues &first_vector, const jointValues &second_vector);

public:
    Controller();
    jointValues get_joint_state();
    GripperStateVector get_gripper_state();
    bool move_to(const coordinates &position, const rotMatrix &rotation, int steps);
    void move_gripper_to(const int diameter);
};

#endif
