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
                if ((abs(box.xmin - blocks[i].second.xmin) <= 2 &&
                     abs(box.ymin - blocks[i].second.ymin) <= 2 &&
                     abs(box.xmax - blocks[i].second.xmax) <= 2 &&
                     abs(box.ymax - blocks[i].second.ymax) <= 2))
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

vector<computer_vision::Points> getAllPoints()
{
    vector<computer_vision::Points> points;
    for (int i = 0; i < blocks.size(); i++)
    {
        pair<int, computer_vision::BoundingBox> block = blocks[i];

        // Bounding box is too large, so we rescale 5px smaller
        double yMax = block.second.ymax - 5;
        double yMin = block.second.ymin + 5;
        double xMax = block.second.xmax - 5;
        double xMin = block.second.xmin + 5;

        double x = (xMax + xMin) / 2;
        double y = yMax;

        cout << "x: " << x << " y: " << y << endl;

        computer_vision::Points point = getPointCloud(x, y);
        points.push_back(point);
    }
    return points;
}

struct Instruction
{
    computer_vision::Points block;
    computer_vision::Points sil;
};

vector<Instruction> createInstructions()
{
    vector<Instruction> retIns = vector<Instruction>();
    if (blocks.size() == 0)
    {
        return retIns;
    }
    for (auto block : blocks)
    {
        Instruction instruction;
        instruction.block = getPointCloud((block.second.xmin + block.second.xmax) / 2, block.second.ymax);
        // instruction.sil = getPointCloud((sil[block.first].second.xmin + sil[block.first].second.xmax) / 2, sil[block.first].second.ymax);
        instruction.sil = computer_vision::Points();
        instruction.sil.x = 0.85;
        instruction.sil.y = 0.70;
        instruction.sil.z = 0.87;
        retIns.push_back(instruction);
    }
    return retIns;
}

bool getPointsCallback(computer_vision::GetPoints::Request &req, computer_vision::GetPoints::Response &res)
{
    vector<Instruction> instructions = createInstructions();
    res.point.push_back(instructions[0].block);
    res.point.push_back(instructions[0].sil);
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

    ros::spin();
    return 0;
}