#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "motion_trajectory.h"
#include <sensor_msgs/JointState.h>
#include "ros/ros.h"
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>

typedef Eigen::Matrix<double, 2, 1> GripperStateVector;

class Controller
{
private:
    const bool test_fast_mode = false;

    ros::NodeHandle node;
    ros::Rate loop_rate;

    bool real_robot;
    bool gripper_sim;
    bool isGripping = false;
    bool side_pick = false;

    jointValues filter_1;
    jointValues filter_2;

    double loop_frequency;
    ros::Publisher pub_des_jstate;
    ros::Publisher pub_gripper_diameter;
    ros::Subscriber sub_joint_state;

    const double acceptable_error = 0.005;

    const double sleep_time_after_movement = 0.5;
    const double sleep_time_after_gripper = 3.0;

    jointValues current_joints;
    GripperStateVector current_gripper;
    bool joint_initialized = false;

    const rotMatrix rotDefault = (rotMatrix() << -1, 0, 0,
                                  0, -1, 0,
                                  0, 0, 1)
                                     .finished();

    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void send_state(const jointValues &joint_pos);
    void sent_gripper_diameter(const int diameter);
    jointValues second_order_filter(const jointValues &input, const double rate, const double settling_time);
    void init_filter(void);
    bool move_inside(vector<double *> *trajectory);

public:
    Controller(double loop_frequency, bool start_homing);
    jointValues get_joint_state();
    GripperStateVector get_gripper_state();
    pair<coordinates, rotMatrix> get_position();
    int move_to(const coordinates &position, const rotMatrix &rotation, bool pick_or_place, bool homing, bool up_and_move_flag, bool move_to_near_axis_flag, bool side_pick_flag);
    int move_to_multiple(vector<pair<coordinates, rotMatrix>>, bool *, bool *, bool *, bool *, bool *);
    void move_gripper_to(const int diameter);
    void print_current_pos_rot();
    void sleep();
    void setGripping(bool isGripping, bool side_pick);

    // testing
    ros::Publisher permission_pub;
};

#endif