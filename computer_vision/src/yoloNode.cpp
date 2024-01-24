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
// struct Instruction
// {
//     computer_vision::Points block;
//     computer_vision::Points sil;
//     string block_type;
//     int angle;
// };
int call = 0;

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

double getOrientationPointCloud(computer_vision::BoundingBox box)
{
    Vector3d block_point_x = Vector3d::Zero();

    cout << "Calculating angle for block: " << box.Class << endl;

    // Check first point of the block touching bbox on x axis
    for (int i = box.xmin; i < box.xmax; i++)
    {
        computer_vision::Points block = getPointCloud(i, box.ymax - 2);
        if (block.z > 0.871)
        {
            block_point_x(0) = block.x;
            block_point_x(1) = block.y;
            block_point_x(2) = block.z;

            break;
        }
    }

    // Check first point of the block touching bbox on y axis
    Vector3d block_point_y = Vector3d::Zero();
    for (int i = box.ymax; i > box.ymin; i--)
    {
        computer_vision::Points block = getPointCloud(box.xmax - 2, i);
        if (block.z > 0.871)
        {
            block_point_y(0) = block.x;
            block_point_y(1) = block.y;
            block_point_y(2) = block.z;

            break;
        }
    }

    cout << "block_point_x: " << block_point_x << endl;
    cout << "block_point_y: " << block_point_y << endl;

    // Get distance x and y distance from 2 points
    double dist_x = block_point_x(0) - block_point_y(0);
    double dist_y = block_point_x(1) - block_point_y(1);

    cout << "dist_x: " << dist_x << endl;
    cout << "dist_y: " << dist_y << endl;

    // Get angle
    double angle = atan2(dist_x, dist_y) * 180 / M_PI;
    if (angle < 0)
    {
        angle += 360;
    }
    cout << "angle: " << angle << endl;
    cout << "--------------------------" << endl;

    return angle;
}

bool checkOnSilhouette(computer_vision::BoundingBox bbox_block)
{
    int offset = 0.2;
    Vector3d sil = blocks_type[bbox_block.Class];
    computer_vision::Points block = getPointCloud((bbox_block.xmin + bbox_block.xmax) / 2, bbox_block.ymax);

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
    if (!init_sil)
    {
        // for (const auto &box : msg.silhouettes)
        // {
        //     if (box.class_n == 0)
        //     {
        //         sil.push_back(pair<int, computer_vision::BoundingBox>(box.class_n, box));
        //     }
        // }
        init_sil = true;
    }
    for (const auto &box : msg.boxes)
    {
        bool to_insert = true;
        for (int i = 0; i < blocks.size(); i++)
        {
            if (checkOnSilhouette(box) || checkSameBBox(box, blocks[i].second))
            {
                to_insert = false;
                break;
            }
        }
        for (int i = 0; i < blocks_history.size(); i++)
        {
            if (checkOnSilhouette(box) || checkSameBBox(box, blocks_history[i].second))
            {
                to_insert = false;
                break;
            }
        }
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
    if (blocks.empty() || !block_detected)
    {
        return computer_vision::Points();
    }
    // Call service for every type of block
    computer_vision::PointCloud srv;
    srv.request.x = x;
    srv.request.y = y;

    // ROS_INFO("Calling service /ur5/locosim/pointcloud for point cloud at (%f, %f)", srv.request.x, srv.request.y);
    computer_vision::Points res = computer_vision::Points();
    if (pc_client.call(srv))
    {
        res.x = srv.response.wx + 0.011;
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

computer_vision::Instruction createInstructions(pair<int, computer_vision::BoundingBox> block)
{
    computer_vision::Instruction instruction;
    instruction.block = getPointCloud((block.second.xmin + block.second.xmax) / 2, block.second.ymax);
    instruction.block.angle = getOrientationPointCloud(block.second);
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
        res.instructions.push_back(createInstructions(block));
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