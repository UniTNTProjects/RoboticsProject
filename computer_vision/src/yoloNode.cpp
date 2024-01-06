#include "robotVision/robotVision.h"
#include <vector>
#include <map>
using namespace std;

vector<pair<int, computer_vision::BoundingBox>> blocks;
vector<pair<int, computer_vision::BoundingBox>> sil;
bool block_detected = false;
bool init_sil = false;
ros::ServiceClient pc_client;
ros::ServiceServer getPoints_server;
struct Instruction
{
    computer_vision::Points block;
    computer_vision::Points sil;
    string block_type;
    int angle;
};

int getOrientation(computer_vision::BoundingBox box)
{
    int x = box.xmax - box.xmin;
    int y = box.ymax - box.ymin;
    if (x > y)
    {
        return 0;
    }
    else if (x < y)
    {
        return 90;
    }
    else
    {
        return 45;
    }
}

bool checkOnSilhouette(computer_vision::BoundingBox block, computer_vision::BoundingBox sil)
{
    // int offset = 5;
    // if (
    //     block.xmin >= sil.xmin - offset &&
    //     block.xmax <= sil.xmax + offset &&
    //     block.ymin >= sil.ymin - offset &&
    //     block.ymax <= sil.ymax + offset)
    // {
    //     return true;
    // }
    // return false;

    return false;
}

bool checkSameBBox(computer_vision::BoundingBox box1, computer_vision::BoundingBox box2)
{
    if (
        abs(box1.xmin - box2.xmin) <= 2 &&
        abs(box1.ymin - box2.ymin) <= 2 &&
        abs(box1.xmax - box2.xmax) <= 2 &&
        abs(box1.ymax - box2.ymax) <= 2)
    {
        return true;
    }
    return false;
}

computer_vision::BoundingBox getSilhouette(int class_n)
{
    for (const auto &box : sil)
    {
        if (box.first == class_n)
        {
            return box.second;
        }
    }
    return computer_vision::BoundingBox();
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
        if (blocks.size() == 0)
        {
            to_insert = true;
        }
        else
        {
            for (int i = 0; i < blocks.size(); i++)
            {
                if (checkSameBBox(box, blocks[i].second)
                    //|| checkOnSilhouette(box, sil[box.class_n].second)
                )
                {
                    to_insert = false;
                    break;
                }
            }
        }
        if (to_insert)
        {
            if (blocks.size() < 10)
            {
                blocks.push_back(pair<int, computer_vision::BoundingBox>(box.class_n, box));
                block_detected = true;
            }
            else
            {
                blocks.erase(blocks.begin());
                blocks.push_back(pair<int, computer_vision::BoundingBox>(box.class_n, box));
                block_detected = true;
            }
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

Instruction createInstructions()
{
    if (blocks.size() == 0)
    {
        ROS_INFO("No blocks detected");
        Instruction instruction;
        instruction.block = computer_vision::Points();
        instruction.block.x = 0;
        instruction.block.y = 0;
        instruction.block.z = 0;

        instruction.sil = computer_vision::Points();
        instruction.sil.x = 0;
        instruction.sil.y = 0;
        instruction.sil.z = 0;

        instruction.angle = 0;
        instruction.block_type = "none";
        return instruction;
    }
    else
    {
        pair<int, computer_vision::BoundingBox> block = blocks.front();
        computer_vision::BoundingBox sil = getSilhouette(block.first);
        Instruction instruction;
        instruction.block_type = block.second.Class;
        instruction.block = getPointCloud((block.second.xmin + block.second.xmax) / 2, block.second.ymax);
        // instruction.sil = getPointCloud((sil.xmin + sil.xmax) / 2, sil.ymax);
        // ------TESTING------
        instruction.sil = computer_vision::Points();
        instruction.sil.x = 0.85;
        instruction.sil.y = 0.70;
        instruction.sil.z = 0.87;
        // ------TESTING------
        instruction.angle = getOrientation(block.second);
        blocks.erase(blocks.begin());
        return instruction;
    }
}

bool getPointsCallback(computer_vision::GetPoints::Request &req, computer_vision::GetPoints::Response &res)
{
    Instruction instruction = createInstructions();
    res.point.push_back(instruction.block);
    res.point.push_back(instruction.sil);
    res.angle = instruction.angle;
    res.type = instruction.block_type;
    cout << res << endl;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yoloNode");
    ros::NodeHandle nh;

    ROS_INFO("Computer vision node started");

    ros::Subscriber detect_sub = nh.subscribe("/computer_vision/bounding_box", 1, robot2DImageCallback);

    pc_client = nh.serviceClient<computer_vision::PointCloud>("/ur5/locosim/pointcloud");
    getPoints_server = nh.advertiseService("/computer_vision/Points", getPointsCallback);
    ROS_INFO("Service /computer_vision/Points ready");
    ros::spin();
    return 0;
}