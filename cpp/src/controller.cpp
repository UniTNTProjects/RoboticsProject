#include "controller.h"
#include <computer_vision/GetPoints.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>

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
    get_ins = node.serviceClient<computer_vision::GetPoints>("computer_vision/Points");

    home_position << -0.465794, -1.46997, -2.16267, -1.07975, -1.5708, 2.03659;
    cordDefault0 << 0.85, 0.32, 1.;
    cordDefault1 << 0.51, 0.32, 1.;
    cordDefault2 << 0.17, 0.32, 1.;
    cordDefault3 << 0.85, 0.64, 1.;
    cordDefault4 << 0.51, 0.64, 1.;
    cordDefault5 << 0.17, 0.64, 1.;
    robotReferenceCord << 0.5, 0.35, 1.46;

    while (!joint_initialized)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    if (start_homing)
    {

        coordinates cord;
        rotMatrix rotation;
        ur5Direct(this->home_position, cord, rotation);
        move_to(cord, rotation, steps, false, true);
    }
}

coordinates Controller::nearHomingRec(coordinates current_cord, coordinates defaultCord, double &nearhomingdist, coordinates &nearhomingcord)
{
    defaultCord << defaultCord(0) - robotReferenceCord(0), robotReferenceCord(1) - defaultCord(1), robotReferenceCord(2) - defaultCord(2);
    double dist = sqrt(pow(current_cord(0) - defaultCord(0), 2) + pow(current_cord(1) - defaultCord(1), 2));
    if (nearhomingdist == -1 || dist < nearhomingdist)
    {
        nearhomingdist = dist;
        nearhomingcord = defaultCord;
    }
    return nearhomingcord;
}

void Controller::nearHoming(coordinates &cord, rotMatrix &rot)
{

    cout << "nearHoming" << endl;

    coordinates current_cord;
    ur5Direct(current_joints, current_cord, rot);
    double nearhomingdist = -1;
    coordinates nearhomingcord;
    coordinates defaultCordArray[6] = {cordDefault0, cordDefault1, cordDefault2, cordDefault3, cordDefault4, cordDefault5};
    for (coordinates defaultCord : defaultCordArray)
    {
        nearhomingcord = nearHomingRec(current_cord, defaultCord, nearhomingdist, nearhomingcord);
    }

    cout << "nearhomingcord: " << nearhomingcord << endl;
    cord = nearhomingcord;
}

coordinates Controller::advanceNearHomingRec(coordinates current_cord, coordinates defaultCord, coordinates final_cord, double &nearhomingdist, coordinates &nearhomingcord)
{
    defaultCord << defaultCord(0) - robotReferenceCord(0), robotReferenceCord(1) - defaultCord(1), robotReferenceCord(2) - defaultCord(2);
    double dist = sqrt(pow(current_cord(0) - defaultCord(0), 2) + pow(current_cord(1) - defaultCord(1), 2));
    double dist2 = sqrt(pow(final_cord(0) - defaultCord(0), 2) + pow(final_cord(1) - defaultCord(1), 2));
    if (nearhomingdist == -1 || (dist + dist2) < nearhomingdist)
    {
        nearhomingdist = dist + dist2;
        nearhomingcord = defaultCord;
    }
    return nearhomingcord;
}

void Controller::advanceNearHoming(coordinates &cord, rotMatrix &rot, coordinates final_cord)
{

    cout << "advanceNearHoming" << endl;

    coordinates current_cord;
    ur5Direct(current_joints, current_cord, rot);
    double nearhomingdist = -1;
    coordinates nearhomingcord;
    coordinates defaultCordArray[6] = {cordDefault0, cordDefault1, cordDefault2, cordDefault3, cordDefault4, cordDefault5};
    for (coordinates defaultCord : defaultCordArray)
    {
        nearhomingcord = advanceNearHomingRec(current_cord, defaultCord, final_cord, nearhomingdist, nearhomingcord);
    }

    cout << "nearhomingcord: " << nearhomingcord << endl;
    cord = nearhomingcord;
}

bool Controller::move_with_steps(const jointValues &values, const bool order[6])
{

    vector<int> joint_number_second_given;
    jointValues firstPos, secondPos;
    for (int i = 0; i < 6; i++)
    {
        if (order[i])
        {
            firstPos(i) = current_joints(i);
            secondPos(i) = values(i);
        }
        else
        {
            firstPos(i) = values(i);
            secondPos(i) = values(i);
        }
    }
    vector<double *> trajectory = vector<double *>();
    for (int i = 0; i < steps; i++)
    {
        trajectory.push_back(new double[6]);
    }

    vector<double *> trajectory2 = vector<double *>();
    for (int i = 0; i < steps; i++)
    {
        trajectory2.push_back(new double[6]);
    }

    if (init_verify_trajectory(&trajectory, current_joints, firstPos, this->steps, false) && init_verify_trajectory(&trajectory2, firstPos, secondPos, this->steps, false))
    {

        if (move_inside(this->steps, false, &trajectory))
        {
            return move_inside(this->steps, false, &trajectory2);
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
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

bool Controller::trajectory_multiple_positions(vector<vector<double *>> *th_sum, vector<pair<coordinates, rotMatrix>> *positions, int n_positions, int n, jointValues init_joint, vector<bool> order)
{
    cout << "n: " << n << endl;
    if (n == n_positions)
    {
        return true;
    }
    coordinates cord = (*positions)[n].first;
    rotMatrix rotation = (*positions)[n].second;

    Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(cord, rotation);

    int *indexes = sort_inverse(inverse_kinematics_res, init_joint);

    for (int i = 0; i < 8; i++)
    {
        int index = indexes[i];
        jointValues joint_to_check;
        joint_to_check << inverse_kinematics_res(index, 0), inverse_kinematics_res(index, 1), inverse_kinematics_res(index, 2),
            inverse_kinematics_res(index, 3), inverse_kinematics_res(index, 4), inverse_kinematics_res(index, 5);
        // cout << "joint_to_check: " << joint_to_check.transpose() << endl;
        if (init_verify_trajectory(&(th_sum->at(n)), init_joint, joint_to_check, this->steps, order[n]))
        {
            // cout << "trajectory verified" << endl;

            if (trajectory_multiple_positions(th_sum, positions, n_positions, n + 1, joint_to_check, order))
            {
                return true;
            }
        }
    }

    return false;
}

bool Controller::up_and_move(const coordinates &position, const rotMatrix &rotation, int steps, jointValues init_joint)
{
    cout << "Trying up and move" << endl;
    coordinates currentPos = get_position().first;
    double new_z = currentPos(2) - 0.25;
    if (new_z < 0.45)
    {
        new_z = 0.45;
    }
    coordinates aboveCurrent, aboveNext;
    aboveCurrent << currentPos(0), currentPos(1), new_z;
    aboveNext << position(0), position(1), new_z;

    vector<pair<coordinates, rotMatrix>> positions = vector<pair<coordinates, rotMatrix>>();
    positions.push_back(make_pair(aboveCurrent, rotation));
    positions.push_back(make_pair(aboveNext, rotation));
    positions.push_back(make_pair(position, rotation));

    vector<vector<double *>> th_sum = vector<vector<double *>>();
    for (int j = 0; j < 3; j++)
    {
        vector<double *> trajectory = vector<double *>();
        for (int i = 0; i < steps; i++)
        {
            trajectory.push_back(new double[6]);
        }
        th_sum.push_back(trajectory);
    }

    if (trajectory_multiple_positions(&th_sum, &positions, 3, 0, init_joint, vector<bool>{true, false, true}))
    {
        move_inside(steps, true, &(th_sum[0]));
        move_inside(steps, false, &(th_sum[1]));
        move_inside(steps, true, &(th_sum[2]));
        return true;
    }

    cout << "Up and move failed" << endl;
    return false;
}

bool Controller::move_to(const coordinates &position, const rotMatrix &rotation, int steps, bool pick_or_place, bool homing)
{
    cout << "Requested move to " << position.transpose() << endl;

    jointValues init_joint = current_joints;
    Eigen::Matrix<double, 8, 6> inverse_kinematics_res = ur5Inverse(position, rotation);
    // cout << "inverse_kinematics_res:\n " << inverse_kinematics_res.transpose() << endl;

    int *indexes = sort_inverse(inverse_kinematics_res, init_joint);

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

        if (init_verify_trajectory(&trajectory, init_joint, joint_to_check, steps, pick_or_place))
        {
            cout << "trajectory verified" << endl;
            cout << "joint_to_check: " << joint_to_check.transpose() << endl;
            return move_inside(steps, pick_or_place, &trajectory);
        }
    }

    cout << "No valid trajectory found" << endl;

    if (!homing)
    {
        cout << "Trying homing" << endl;
        coordinates cord;
        rotMatrix rotation;
        // nearHoming(cord, rotation);
        advanceNearHoming(cord, rotation, position);
        // ur5Direct(this->home_position, cord, rotation);
        if (move_to(cord, rotation, steps, false, true))
        {
            return move_to(position, rotation, steps, pick_or_place, true);
        }
    }

    if (up_and_move(position, rotation, steps, init_joint))
    {
        return true;
    }

    cout << "Trying with 2 steps" << endl;
    for (int i = 0; i < 8; i++)
    {
        int index = indexes[i];
        jointValues joint_to_check;
        joint_to_check << inverse_kinematics_res(index, 0), inverse_kinematics_res(index, 1), inverse_kinematics_res(index, 2),
            inverse_kinematics_res(index, 3), inverse_kinematics_res(index, 4), inverse_kinematics_res(index, 5);
        bool configurations_of_orders[5][6] = {
            {false, false, false, true, true, false},
            {false, false, false, false, true, true},
            {false, false, false, true, true, true},
            {true, true, true, false, false, false},
            {true, true, false, false, false, false}};
        for (int i = 0; i < 5; i++)
        {
            bool *order = configurations_of_orders[i];

            if (move_with_steps(joint_to_check, order))
            {
                return true;
            }
        }
    }
    cout << "Move with 2 steps failed" << endl;

    return false;
}

bool Controller::init_verify_trajectory(vector<double *> *Th, jointValues init_joint, jointValues final_joint, int steps, bool pick_or_place)
{
    ur5Trajectory(Th, init_joint, final_joint, steps);

    return check_trajectory(*Th, steps, pick_or_place);
}

bool Controller::move_inside(int steps, bool pick_or_place, vector<double *> *trajectory)
{
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
        cout << "\n....\n NOT USING FILTER\n....\n"
             << endl;
    }
    for (int j = 0; j < steps; j++)
    {
        // get i element of variable "trajectory" and put it in new variable
        jointValues des_not_linear;
        double *traj_j = (*trajectory)[j];
        des_not_linear << traj_j[0], traj_j[1], traj_j[2],
            traj_j[3], traj_j[4], traj_j[5];

        // cout << "des_not_linear: " << des_not_linear << endl;
        //  send the trajectory

        while (ros::ok() && this->acceptable_error < calculate_distance(des_not_linear, current_joints))
        {
            // cout << "error: " << calculate_distance(des_not_linear, current_joints) << endl;
            jointValues q_des = second_order_filter(des_not_linear, loop_frequency, 0.5);
            // cout << "q_des: " << q_des << endl;
            if (use_filter)
            {
                send_state(q_des);
            }
            else
            {
                send_state(des_not_linear);
            }
            this->loop_rate.sleep();

            ros::spinOnce();
        }
    }

    cout << "Done\n------------\n"
         << endl;
    ros::Duration(sleep_time_after_movement).sleep();
    return true;
}

int *Controller::sort_inverse(Eigen::Matrix<double, 8, 6> &inverse_kinematics_res, const jointValues &initial_joints)
{
    multimap<double, int> sorted_inverse;

    for (int i = 0; i < 8; i++)
    {
        jointValues inverse_i;
        inverse_i << inverse_kinematics_res(i, 0), inverse_kinematics_res(i, 1), inverse_kinematics_res(i, 2),
            inverse_kinematics_res(i, 3), inverse_kinematics_res(i, 4), inverse_kinematics_res(i, 5);

        // double diff = (inverse_i - initial_joints).norm(); // doesnt take into account the angle normalization
        double diff = calculate_distance_weighted(inverse_i, initial_joints);

        // facing back of the table
        if (norm_angle(inverse_i(0)) > 3.7 && norm_angle(inverse_i(0)) < 5.7)
        {
            diff += 100;
        }
        cout << "diff: " << diff << ", i: " << i << endl;
        sorted_inverse.insert(pair<double, int>(diff, i));
    }

    int *sorted_indexes = new int[8];
    int i = 0;

    for (std::multimap<double, int>::iterator it = sorted_inverse.begin(); it != sorted_inverse.end(); ++it)
    {
        sorted_indexes[i++] = it->second;
        // cout << "value of sort:" << it->first << ", " << it->second << endl;
    }

    return sorted_indexes;
}

bool Controller::check_trajectory(vector<double *> traj, int step, bool pick_or_place)
{
    coordinates start_cord;
    rotMatrix start_rotation;
    ur5Direct(current_joints, start_cord, start_rotation);

    // cout << "check trajectory" << endl;
    for (int i = 0; i < step; i++)
    {
        coordinates cord;
        rotMatrix rot;
        jointValues joints;
        joints << traj[i][0], traj[i][1], traj[i][2],
            traj[i][3], traj[i][4], traj[i][5];

        ur5Direct(joints, cord, rot);
        /*
        if (i == step - 1)
        {
            cout << "last cord: " << cord.transpose() << endl;
            cout << "last joint: " << joints.transpose() << endl;
            cout << "last rotation: " << endl
                 << rot << endl;
        }
        */
        if (!(cord(0) > min_x && cord(0) < max_x && cord(1) > min_y && cord(1) < max_y && cord(2) > min_z && cord(2) < max_z))
        {
            // cout << "coordinates of trajectory does not fit in the workspace:" << cord.transpose() << endl;
            return false;
        }

        if (!pick_or_place && cord(2) > max_z_moving)
        {
            // cout << "cord z wrong: " << cord.transpose() << endl;
            return false;
        }

        if (!pick_or_place && cord(2) > max_z_near_end_table && cord(1) > max_y_near_end_table)
        {
            // cout << "cord end table wrong: " << cord.transpose() << endl;
            return false;
        }

        if (pick_or_place && (fabs(traj[0][4] - joints(4)) > M_PI / 2 || fabs(traj[0][5] - joints(5)) > M_PI / 2))
        {
            /*
            cout << "trying strange rotation for pick or place" << endl;
            cout << "traj[0][4]: " << traj[0][4] << ", traj[0][5]: " << traj[0][5] << endl;
            cout << "joints(4): " << joints(4) << ", joints(5): " << joints(5) << endl;
            cout << fabs(traj[0][4] - joints(4)) << ", " << fabs(traj[0][5] - joints(5)) << endl;
            */
            return false;
        }

        if (pick_or_place && (start_rotation - rot).norm() > 0.1)
        {
            /*
            cout << "rotations are different" << endl;
            cout << "start_rot: " << start_rotation << endl;
            cout << "rot: " << rot << endl;
            */
            return false;
        }

        if (joints.norm() == 0)
        {
            continue;
        }

        MatrixXd jacobian = ur5Jac(joints);
        // cout << "jacobian: " << jacobian << endl;

        if (joints(1) > 0 || joints(1) < -3.14)
        {
            // cout << "joints(1) > -0.2" << endl;
            return false;
        }

        if (abs(jacobian.determinant()) < 0.000001)
        {
            /*
            cout << "----------\ntrajectory invalid 1" << endl;
            cout << "determinant of jacobian is " << abs(jacobian.determinant()) << "\n-------------\n"
                 << endl;
            */
            return false;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (abs(svd.singularValues()(5)) < 0.0000001)
        {
            // cout << "trajectory invalid 2" << endl;
            cout << abs(svd.singularValues()(5)) << endl;
            return false;
        }
    }
    cout << "trajectory valid" << endl;

    return true;
}

double Controller::norm_angle(double angle)
{
    if (angle > 0)
    {
        return fmod(angle, 2 * M_PI);
    }
    else
    {
        return 2 * M_PI - fmod(-angle, 2 * M_PI);
    }
}

double Controller::calculate_distance(const jointValues &first_vector, const jointValues &second_vector)
{
    jointValues first_norm, second_norm;
    for (int i = 0; i < 6; i++)
    {
        first_norm(i) = norm_angle(second_vector(i));
        second_norm(i) = norm_angle(first_vector(i));
    }
    return (first_norm - second_norm).norm();
}

double Controller::calculate_distance_weighted(const jointValues &first_vector, const jointValues &second_vector)
{
    double distance_val = 0;
    double weight[6] = {1, 1, 1, 10, 10, 1};

    for (int i = 0; i < 6; i++)
    {
        double first = norm_angle(second_vector(i));
        double second = norm_angle(first_vector(i));
        distance_val += weight[i] * fabs(first - second);
    }

    return distance_val;
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