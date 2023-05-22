#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

int *sort_ik_result(const Eigen::Matrix<double, 8, 6> &ik_result, const jointValues &initial_joints);

static double v_ref;
static jointValues linear_fil;

void Controller::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    cout << "joint_state_callback" << endl;
    int n = (real_robot) ? 6 : 8;

    for (int i = 0; i < n; i++)
    {
        if (i >= 6)
        {
            // std::cout << "current gripper n°" << i - 6 << ": " << msg->position[i] << std::endl;
            current_gripper(i - 6) = msg->position[i];
        }
        else
        {
            // std::cout << "current joint n°" << i << ": " << msg->position[i] << std::endl;
            current_joints(i) = msg->position[i];
        }
    }
}

Controller::Controller() : loop_rate(1000.)
{
    node.getParam("/real_robot", real_robot);
    node.getParam("/gripper_sim", gripper_sim);

    sub_joint_state = node.subscribe("/ur5/joint_states", 1, &Controller::joint_state_callback, this);
    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);
    pub_gripper_diameter = node.advertise<std_msgs::Int32>("/ur5/gripper_controller/command", 1);
}

void Controller::send_state(const jointValues &joint_pos)
{
    // std::cout << "q_des " << joint_pos.transpose() << std::endl;

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
    cout << "gripper moved" << endl;
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

void Controller::init_linear_filter(void)
{
    cout << "init linear filter" << endl;
    linear_fil = get_joint_state();
    v_ref = 0.0;
}

jointValues Controller::linear_filter_calc(const jointValues &joints_des)
{
    double v_des = 0.6;
    v_ref += (v_des - v_ref) * 0.005;
    linear_fil += 1 / loop_frequency * v_ref * (joints_des - linear_fil) / (joints_des - linear_fil).norm();

    return linear_fil;
}

bool Controller::move_to(const coordinates &position, const rotMatrix &rotation, int steps)
{
    double *trajectory;

    cout << "position: " << position << endl;
    cout << "rotation: " << rotation << endl;

    jointValues init_joint = get_joint_state();
    Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(position, rotation);
    cout << "inverse_kinematics_res:\n " << inverse_kinematics_res << endl;

    int *indexes = sort_ik_result(inverse_kinematics_res, init_joint);

    for (int i = 0; i < 8; i++)
    {
        // check if the trajectory is valid
        int index = indexes[i];
        jointValues joint_to_check;
        joint_to_check << inverse_kinematics_res(index, 0), inverse_kinematics_res(index, 1), inverse_kinematics_res(index, 2),
            inverse_kinematics_res(index, 3), inverse_kinematics_res(index, 4), inverse_kinematics_res(index, 5);

        trajectory = ur5Trajectory(init_joint, joint_to_check, steps);

        if (check_trajectory(trajectory, steps))
        {
            init_linear_filter();

            for (int j = 0; j < steps; j++)
            {
                jointValues des_not_linear;
                des_not_linear << trajectory[j * 6], trajectory[j * 6 + 1], trajectory[j * 6 + 2],
                    trajectory[j * 6 + 3], trajectory[j * 6 + 4], trajectory[j * 6 + 5];

                // send the trajectory
                cout << "sending trajectory" << endl;

                while (ros::ok() && 0.05 < compute_error(des_not_linear, get_joint_state()))
                {
                    cout << "error: " << compute_error(des_not_linear, get_joint_state()) << endl;
                    jointValues q_des = linear_filter_calc(des_not_linear);
                    send_state(q_des);
                    loop_rate.sleep();
                    ros::spinOnce();
                }
            }

            free(trajectory);
            return true;
        }
        else
        {
            free(trajectory);
        }
    }

    return false;
}

int *sort_ik_result(const Eigen::Matrix<double, 8, 6> &ik_result, const jointValues &initial_joints)
{
    multimap<double, int> m;
    for (int i = 0; i < 8; i++)
    {
        jointValues comp;
        comp << ik_result(i, 0), ik_result(i, 1), ik_result(i, 2),
            ik_result(i, 3), ik_result(i, 4), ik_result(i, 5);
        m.insert(pair<double, int>((comp - initial_joints).norm(), i));
    }

    int *list = (int *)malloc(sizeof(int) * 8);
    int i = 0;
    for (auto const &it : m)
    {
        list[i++] = it.second;
    }
    return list;
}

bool Controller ::check_trajectory(double *traj, int step)
{
    cout << "check trajectory" << endl;
    for (int i = 0; i < step; i++)
    {
        coordinates cord;
        rotMatrix rot;
        jointValues joints;
        joints << traj[i * 6], traj[i * 6 + 1], traj[i * 6 + 2], traj[i * 6 + 3], traj[i * 6 + 4], traj[i * 6 + 5];
        ur5Direct(joints, cord, rot);

        if (joints.norm() == 0)
        {
            continue;
        }

        MatrixXd jacobian = ur5Jac(joints);
        cout << "jacobian: " << jacobian << endl;
        /*
        if (abs(jacobian.determinant()) < 0.0001)
        {
            cout << "trajectory invalid 1" << endl;
            cout << abs(jacobian.determinant()) << endl;
            return false;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (abs(svd.singularValues()(5)) < 0.00001)
        {
            cout << "trajectory invalid 2" << endl;
            cout << abs(svd.singularValues()(5)) << endl;
            return false;
        }
        */
    }
    cout << "trajectory checked" << endl;
    return true;
}

double norm_angle(double angle)
{
    if (angle > 0)
        angle = fmod(angle, 2 * M_PI);
    else
        angle = 2 * M_PI - fmod(-angle, 2 * M_PI);
    return angle;
}

double Controller::compute_error(const jointValues &first_vector, const jointValues &second_vector)
{
    jointValues current_normed, desired_normed;
    for (int i = 0; i < 6; i++)
    {
        current_normed(i) = norm_angle(second_vector(i));
        desired_normed(i) = norm_angle(first_vector(i));
    }
    return (current_normed - desired_normed).norm();
}

void Controller::move_gripper_to(const int diameter)
{

    coordinates cord;
    rotMatrix rot;
    jointValues joints;
    joints = get_joint_state();
    ur5Direct(joints, cord, rot);
    cout << "cord: " << cord << endl;
    cout << "rot: " << rot << endl;

    sent_gripper_diameter(diameter);
}