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
    bool test_fast_mode = false;

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

    double sleep_time_after_movement = 0.5;
    double sleep_time_after_gripper = 4;

    double max_x = 1;
    double max_y = 0.4;
    double max_z = 0.8;

    double min_x = -1;
    double min_y = -1;
    double min_z = -1;

    double max_y_near_end_table = 0.18;
    double max_z_near_end_table = 0.58;

    double max_z_moving = 0.73;

    jointValues home_position;

    jointValues current_joints;
    GripperStateVector current_gripper;
    bool joint_initialized = false;

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void send_state(const jointValues &joint_pos);
    void sent_gripper_diameter(const int diameter);
    bool check_trajectory(vector<double *> traj, int step, bool pick_or_place);
    jointValues second_order_filter(const jointValues &input, const double rate, const double settling_time);
    void init_filter(void);
    double calculate_distance(const jointValues &first_vector, const jointValues &second_vector);
    int *sort_inverse(Eigen::Matrix<double, 8, 6> &inverse_kinematics_res, const jointValues &initial_joints);
    double norm_angle(double angle);
    bool move_with_steps(const jointValues &values, const bool order[6]);
    bool move_inside(int steps, bool pick_or_place, vector<double *> *trajectory);
    bool init_verify_trajectory(vector<double *> *Th, jointValues init_joint, jointValues final_joint, int steps, bool pick_or_place);
    bool trajectory_multiple_positions(vector<vector<double *>> *th_sum, vector<pair<coordinates, rotMatrix>> *positions, int n_positions, int n, jointValues init_joint, vector<bool> order);

public:
    Controller(double loop_frequency);
    jointValues get_joint_state();
    GripperStateVector get_gripper_state();
    pair<coordinates, rotMatrix> get_position();
    bool move_to(const coordinates &position, const rotMatrix &rotation, int steps, bool pick_or_place, bool homing);
    void move_gripper_to(const int diameter);
    void print_current_pos_rot();

    const int steps = 30;
};

#endif
