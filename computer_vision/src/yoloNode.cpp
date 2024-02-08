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
    Vector3d block_point_x = Vector3d::Zero();
    const int range = 2;
    // cout << "Calculating angle for block: " << box.Class << endl;

    double min_x = 1000; // Block corner touching bbox
    // Check first point of the block touching bbox on x axis
    // cout << "Getting point cloud on x axis for block: " << box.Class << endl;
    for (int i = box.xmin; i < box.xmax; i++)
    {
        computer_vision::Points block = getPointCloud(i, box.ymax - 2);
        if (block.z > 0.871 && block.x < min_x)
        {
            block_point_x(0) = block.x;
            block_point_x(1) = block.y;
            block_point_x(2) = block.z;
            min_x = block.x;
        }
    }

    // cout << "Getting point cloud on y axis for block: " << box.Class << endl;
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

    cout << endl
         << "Block_type: " << box.Class << endl;
    cout << "From left to right: " << endl;
    cout << "block_point_x: " << block_point_x << endl;
    cout << "block_point_y: " << block_point_y << endl;

    // Get distance x and y distance from 2 points
    double dist_x = block_point_y(0) - block_point_x(0);
    double dist_y = block_point_x(1) - block_point_y(1);
    double real_dist = block_point_y(1);
    double angle = 0.0;
    bool vertical = false;
    cout << "dist_x: " << dist_x << endl;
    cout << "dist_y: " << dist_y << endl;
    if (dist_y < 0.018 || (block_point_y(0) == 0 || block_point_x(0) == 0)) // points too close, check other side
    {
        cout << "Points too close, checking other side" << endl;
        min_x = 1000;
        block_point_x = Vector3d::Zero();
        block_point_y = Vector3d::Zero();
        for (int i = box.xmax; i > box.xmin; i--)
        {
            computer_vision::Points block = getPointCloud(i, box.ymax - 2);
            if (block.z > 0.871 && block.x < min_x)
            {
                block_point_x(0) = block.x;
                block_point_x(1) = block.y;
                block_point_x(2) = block.z;
                min_x = block.x;
            }
        }

        for (int i = box.ymax; i > box.ymin; i--)
        {
            computer_vision::Points block = getPointCloud(box.xmin, i);
            if (block.z > 0.871)
            {
                block_point_y(0) = block.x;
                block_point_y(1) = block.y;
                block_point_y(2) = block.z;

                break;
            }
        }
        dist_x = block_point_y(0) - block_point_x(0);
        dist_y = block_point_y(1) - block_point_x(1);
        angle = M_PI - atan2((dist_y), (dist_x));
        real_dist = abs(block_point_y(1) - real_dist);

        cout << "From right to left: " << endl;
        cout << "block_point_x: " << block_point_x << endl;
        cout << "block_point_y: " << block_point_y << endl;

        cout << "dist_x: " << dist_x << endl;
        cout << "dist_y: " << dist_y << endl;
        if (dist_x < 0.01 && real_dist < 0.03 && dist_y > 0.018)
        {
            cout << "Real dist: " << real_dist << endl;
            vertical = true;
        }
    }
    else
    {
        angle = atan2(dist_x, dist_y);
        // if (dist_x < 0.01 && dist_y > 0.018)
        // {
        //     vertical = true;
        // }
    }

    cout << "Angle pre normalization: " << angle * 180 / M_PI << endl;
    // Normalize angle to -180 - 180 degrees
    if (angle < M_PI_2 && angle >= 0)
    {
        angle -= M_PI;
    }
    else if (angle < 0 && angle > -M_PI_2)
    {
        angle += M_PI;
    }

    angle = round(angle * 180 / M_PI);

    if (vertical)
    { // Blocco verticale
        angle = 90;
    }

    cout << "angle: " << angle;
    cout << "--------------------------" << endl;

    return angle;
}

bool checkOnSilhouette(computer_vision::BoundingBox bbox_block)
{
    int offset = 0.2;
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
    // if (blocks.empty() || !block_detected)
    // {
    //     return computer_vision::Points();
    // }
    // Call service for every type of block
    computer_vision::PointCloud srv;
    srv.request.x = x;
    srv.request.y = y;

    // ROS_INFO("Calling service /ur5/locosim/pointcloud for point cloud at (%f, %f)", srv.request.x, srv.request.y);
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

    for (int i = box.xmin; i < box.xmax; i++)
    {
        computer_vision::Points block = getPointCloud(i, box.ymin);
        if (block.z > 0.871)
        {
            top_left.x = block.x;
            top_left.y = block.y;
            top_left.z = block.z;
            break;
        }
    }

    for (int i = box.xmax; i > box.xmin; i--)
    {
        computer_vision::Points block = getPointCloud(i, box.ymax);
        if (block.z > 0.871)
        {
            bottom_right.x = block.x;
            bottom_right.y = block.y;
            bottom_right.z = block.z;
            break;
        }
    }

    // middle point
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
    instruction.block.angle = getOrientationPointCloud(block.second);
    cout << "BLOCK: " << block.second.Class << endl;
    cout << "Angle: " << instruction.block.angle * M_PI / 180 << " Degrees: " << instruction.block.angle << endl;
    cout << "Position: " << instruction.block << endl;
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