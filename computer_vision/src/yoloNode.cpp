#include "robotVision/robotVision.h"
#include <vector>
#include <queue>
#include <map>
using namespace std;

vector<pair<int, computer_vision::BoundingBox>> blocks_history;
vector<pair<int, computer_vision::BoundingBox>> blocks;
vector<pair<int, computer_vision::BoundingBox>> sil;
bool block_detected = false;
bool init_sil = false;
ros::ServiceClient pc_client;
ros::ServiceServer getPoints_server;
bool checkPointCloud(computer_vision::Points point)
{
    if ((point.x == 0 && point.y == 0 && point.z == 0) || point.z < 0.86 || point.z > 1.0)
    {
        // cout << "Invalid point cloud: " << point << endl;
        return false;
    }
    return true;
}
int call = 0;
const bool testing = false;
int getOrientation(computer_vision::BoundingBox box)
{
    int offset = 10;
    int x = box.xmax - box.xmin;
    int y = box.ymax - box.ymin;
    if (abs(x - y) <= offset)
    {
        return 45;
    }
    else if (x > y)
    {
        return 0;
    }
    else
    {
        return 90;
    }
}

int getOrientationPointCloud(computer_vision::BoundingBox box)
{

    Vector3d block_point_x_bot = Vector3d::Zero();
    Vector3d block_point_y_left = Vector3d::Zero();
    Vector3d block_point_y_right = Vector3d::Zero();

    const int range = 2;

    double min_x = 1000; // Block corner touching bbox
    // Check first point of the block touching bbox on x axis
    // cout << "Getting point cloud on x axis for block: " << box.Class << endl;
    for (int i = box.xmin; i < box.xmax; i++)
    {
        computer_vision::Points block = getPointCloud(i, box.ymax - 2);
        if (block.z > 0.871 && block.x < min_x)
        {
            block_point_x_bot(0) = block.x;
            block_point_x_bot(1) = block.y;
            block_point_x_bot(2) = block.z;
            min_x = block.x;
        }
    }

    // cout << "Getting point cloud on y axis for block: " << box.Class << endl;
    // Check first point of the block touching bbox on y axis
    for (int i = box.ymax; i > box.ymin; i--)
    {
        computer_vision::Points block = getPointCloud(box.xmax - 2, i);
        if (block.z > 0.871)
        {
            block_point_y_right(0) = block.x;
            block_point_y_right(1) = block.y;
            block_point_y_right(2) = block.z;

            break;
        }
    }

    for (int i = box.ymax; i > box.ymin; i--)
    {
        computer_vision::Points block = getPointCloud(box.xmin, i);
        if (block.z > 0.871)
        {
            block_point_y_left(0) = block.x;
            block_point_y_left(1) = block.y;
            block_point_y_left(2) = block.z;

            break;
        }
    }

    double dist_x_right = block_point_y_right(0) - block_point_x_bot(0);
    double dist_y_right = block_point_y_right(1) - block_point_x_bot(1);
    double dist_x_left = block_point_y_left(0) - block_point_x_bot(0);
    double dist_y_left = block_point_y_left(1) - block_point_x_bot(1);

    double angle_right = atan2(dist_x_right, dist_y_right);
    double angle_left = atan2(dist_x_left, dist_y_left);

    // cout << "dist_x_right: " << dist_x_right << " dist_y_right: " << dist_y_right << endl;
    // cout << "dist_x_left: " << dist_x_left << " dist_y_left: " << dist_y_left << endl;

    angle_right = round(angle_right * 180 / M_PI);
    angle_left = round(angle_left * 180 / M_PI);

    double total_dist = abs(block_point_y_left(1) - block_point_y_right(1));
    // cout << "Block: " << box.Class << " total dist: " << total_dist << endl;
    // cout << "dist_x_left: " << dist_x_left << " dist_x_right: " << dist_x_right << endl;
    // cout << "block_point_y_left: " << block_point_y_left << " block_point_y_right: " << block_point_y_right << endl;
    if (total_dist < 0.035 && dist_x_left < 0.015 && dist_x_right < 0.015)
    {
        // cout << "Vertical block" << endl;
        return 90;
    }

    if (dist_x_right >= dist_x_left)
    {
        return angle_right;
    }

    return angle_left - 180;
}

bool checkOnSilhouette(computer_vision::BoundingBox bbox_block)
{
    double offset = 0.2;
    Vector3d sil = blocks_type[bbox_block.Class];
    computer_vision::Points block = getPointCloud(bbox_block);

    if (abs(block.x - sil(0)) <= offset && abs(block.y - sil(1)) <= offset)
        return true;

    return false;
}

bool checkSameBBox(computer_vision::BoundingBox box1, computer_vision::BoundingBox box2)
{
    if (
        abs(box1.xmin - box2.xmin) <= 5 &&
        abs(box1.ymin - box2.ymin) <= 5 &&
        abs(box1.xmax - box2.xmax) <= 5 &&
        abs(box1.ymax - box2.ymax) <= 5)
    {
        return true;
    }
    return false;
}
void robot2DImageCallback(const computer_vision::BoundingBoxes &msg)
{

    for (const auto &box : msg.boxes)
    {
        if (testing)
        {
            int angle = getOrientationPointCloud(box);
            // cout << "BLOCK: " << box.Class << endl;
            // cout << "Angle: " << angle * M_PI / 180 << " Degrees: " << angle << endl;
            // cout << "Position: " << block << endl;
        }

        bool to_insert = true;
        for (int i = 0; i < blocks.size(); i++)
        {
            if (checkOnSilhouette(box) || checkSameBBox(box, blocks[i].second))
            {
                to_insert = false;
                break;
            }
        }
        // for (int i = 0; i < blocks_history.size(); i++)
        // {
        //     if (checkOnSilhouette(box) || checkSameBBox(box, blocks_history[i].second))
        //     {
        //         to_insert = false;
        //         break;
        //     }
        // }
        if (to_insert)
        {
            blocks.push_back(pair<int, computer_vision::BoundingBox>(box.class_n, box));
            block_detected = true;
            to_insert = false;
        }
    }
    return;
}

computer_vision::Points getPointCloud(double x, double y)
{

    // Call service for every type of block
    computer_vision::PointCloud srv;
    srv.request.x = x;
    srv.request.y = y;

    computer_vision::Points res = computer_vision::Points();
    if (pc_client.call(srv))
    {
        res.x = srv.response.wx;
        res.y = srv.response.wy;
        res.z = srv.response.wz;
    }
    else
    {
        ROS_ERROR("Failed to call service /ur5/locosim/pointcloud");
        return computer_vision::Points();
    }
    return res;
}

computer_vision::Points getPointCloud(computer_vision::BoundingBox box)
{
    // get 2 points from the block
    computer_vision::Points top_left = computer_vision::Points();
    computer_vision::Points bottom_right = computer_vision::Points();

    for (int i = box.ymin; i < box.ymax; i++)
    {
        computer_vision::Points block = getPointCloud(box.xmin + 3, i);
        if (block.z > 0.87)
        {
            top_left.x = block.x;
            top_left.y = block.y;
            top_left.z = block.z;
            break;
        }
    }

    for (int i = box.ymax; i > box.ymin; i--)
    {
        computer_vision::Points block = getPointCloud(box.xmax - 3, i);
        if (block.z > 0.87)
        {
            bottom_right.x = block.x;
            bottom_right.y = block.y;
            bottom_right.z = block.z;
            break;
        }
    }

    // middle point
    if (!checkPointCloud(top_left) || !checkPointCloud(bottom_right))
    {
        // cout << "Invalid point cloud for block: " << box.Class << endl;
        // cout << "Top left: " << top_left << " Bottom right: " << bottom_right << endl;
        return computer_vision::Points();
    }
    computer_vision::Points middle = computer_vision::Points();
    middle.x = (top_left.x + bottom_right.x) / 2;
    middle.y = (top_left.y + bottom_right.y) / 2;
    middle.z = (top_left.z + bottom_right.z) / 2;

    return middle;
}

computer_vision::Instruction createInstructions(pair<int, computer_vision::BoundingBox> block)
{
    computer_vision::Instruction instruction;
    instruction.block = getPointCloud(block.second);
    if (!checkPointCloud(instruction.block))
    {
        // cout << "Invalid point cloud for block: " << block.second.Class << endl;
        return computer_vision::Instruction();
    }
    instruction.block.angle = getOrientationPointCloud(block.second);
    // cout << "BLOCK: " << block.second.Class << endl;
    // cout << "Angle: " << instruction.block.angle * M_PI / 180 << " Degrees: " << instruction.block.angle << endl;
    // cout << "Position: " << instruction.block << endl;
    // instruction.block.angle = getOrientation(block.second);
    // instruction.sil = getPointCloud((sil.xmin + sil.xmax) / 2, sil.ymax);
    // ------TESTING------
    instruction.sil = computer_vision::Points();
    Vector3d block_type = blocks_type[block.second.Class];
    instruction.sil.x = block_type(0);
    instruction.sil.y = block_type(1);
    instruction.sil.z = instruction.block.z;
    instruction.sil.angle = 0;
    instruction.type = block.second.Class;
    // ------TESTING------
    return instruction;
}

bool getInstructionsCallback(computer_vision::GetInstructions::Request &req, computer_vision::GetInstructions::Response &res)
{
    for (const auto &block : blocks)
    {
        computer_vision::Instruction instruction = createInstructions(block);
        if (instruction.block.x == 0 && instruction.block.y == 0 && instruction.block.z == 0)
        {
            continue;
        }
        res.instructions.push_back(instruction);
    }

    blocks_history = blocks;
    blocks.clear();

    ROS_INFO("Returning %ld instructions \n", res.instructions.size());

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yoloNode");
    ros::NodeHandle nh;

    ROS_INFO("Computer vision node started");

    ros::Subscriber detect_sub = nh.subscribe("/computer_vision/bounding_box", 1, robot2DImageCallback);

    pc_client = nh.serviceClient<computer_vision::PointCloud>("/ur5/locosim/pointcloud");
    getPoints_server = nh.advertiseService("/computer_vision/Instructions", getInstructionsCallback);
    ROS_INFO("Service /computer_vision/Instructions ready");
    ros::spin();
    return 0;
}