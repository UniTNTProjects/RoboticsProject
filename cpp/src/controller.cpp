#include "controller.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

static jointValues current_joints;
static GripperStateVector current_gripper;

static bool joint_initialized = false;

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

Controller::Controller() : loop_rate(1000.)
{
    node.getParam("/real_robot", real_robot);
    node.getParam("/gripper_sim", gripper_sim);

    sub_joint_state = node.subscribe("/ur5/joint_states", 1, &Controller::joint_state_callback, this);
    pub_des_jstate = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);
    pub_gripper_diameter = node.advertise<std_msgs::Int32>("/ur5/gripper_controller/command", 1);

    while (!joint_initialized)
    {
        ros::spinOnce();
        loop_rate.sleep();
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

void Controller::init_filter(void)
{
    cout << "init linear filter" << endl;
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

bool Controller::move_to(const coordinates &position, const rotMatrix &rotation, int steps)
{
    jointValues init_joint = current_joints;
    Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(position, rotation);
    // cout << "inverse_kinematics_res:\n " << inverse_kinematics_res << endl;

    int *indexes = sort_ik_result(inverse_kinematics_res, init_joint);

    for (int i = 0; i < 8; i++)
    {
        // check if the trajectory is valid
        int index = indexes[i];
        jointValues joint_to_check;
        joint_to_check << inverse_kinematics_res(index, 0), inverse_kinematics_res(index, 1), inverse_kinematics_res(index, 2),
            inverse_kinematics_res(index, 3), inverse_kinematics_res(index, 4), inverse_kinematics_res(index, 5);

        /*
        cout << "joint_to_check: " << joint_to_check << endl;
        cout << "init_joint: " << init_joint << endl;
        cout << "init gripper" << current_gripper << endl;
        */

        vector<double *> trajectory = vector<double *>();
        for (int i = 0; i < steps; i++)
        {
            trajectory.push_back(new double[6]);
        }

        ur5Trajectory(&trajectory, init_joint, joint_to_check, steps);

        if (check_trajectory(trajectory, steps))
        {
            init_filter();

            cout << "Start moving to position: " << position.transpose() << endl;
            for (int j = 0; j < steps; j++)
            {
                // get i element of variable "trajectory" and put it in new variable
                jointValues des_not_linear;
                double *traj_j = trajectory[j];
                des_not_linear << traj_j[0], traj_j[1], traj_j[2],
                    traj_j[3], traj_j[4], traj_j[5];

                // send the trajectory

                while (ros::ok() && 0.05 < compute_error(des_not_linear, current_joints))
                {
                    // cout << "error: " << compute_error(des_not_linear, current_joints) << endl;
                    jointValues q_des = second_order_filter(des_not_linear, loop_frequency, 1);
                    // cout << "q_des: " << q_des << endl;
                    send_state(q_des);
                    loop_rate.sleep();

                    ros::spinOnce();
                }
            }

            cout << "Done\n------------\n"
                 << endl;
            ros::Duration(0.2).sleep();
            return true;
        }
    }
    return false;
}

int *Controller::sort_ik_result(const Eigen::Matrix<double, 8, 6> &ik_result, const jointValues &initial_joints)
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

bool Controller ::check_trajectory(vector<double *> traj, int step)
{
    cout << "check trajectory" << endl;
    for (int i = 0; i < step; i++)
    {
        coordinates cord;
        rotMatrix rot;
        jointValues joints;
        double *traj_i = traj[i];
        joints << traj_i[0], traj_i[1], traj_i[2],
            traj_i[3], traj_i[4], traj_i[5];

        if (joints.norm() == 0)
        {
            continue;
        }

        MatrixXd jacobian = ur5Jac(joints);
        // cout << "jacobian: " << jacobian << endl;
        /*
        if (joints(2) > 0.74 && joints(1) > -0.4)
        {
            cout << "----------\ntrajectory invalid 0" << endl;
            cout << "not valid values\n-------------\n"
                 << endl;
            return false;
        }
        */

        if (abs(jacobian.determinant()) < 0.000001)
        {
            cout << "----------\ntrajectory invalid 1" << endl;
            cout << "determinant of jacobian is " << abs(jacobian.determinant()) << "\n-------------\n"
                 << endl;
            return false;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (abs(svd.singularValues()(5)) < 0.0000001)
        {
            cout << "trajectory invalid 2" << endl;
            cout << abs(svd.singularValues()(5)) << endl;
            return false;
        }
    }
    cout << "trajectory valid" << endl;
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