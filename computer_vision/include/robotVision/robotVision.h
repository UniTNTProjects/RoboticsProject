#ifndef __ROBOTVISION__
#define __ROBOTVISION__

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "computer_vision/BoundingBox.h"
#include "computer_vision/BoundingBoxes.h"
#include "computer_vision/Points.h"
#include "computer_vision/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "computer_vision/GetInstructions.h"
#include "eigen3/Eigen/Dense"
#include <vector>
#include <string>
#include <map>
using namespace std;
using namespace Eigen;
void robot2DImageCallback(const computer_vision::BoundingBoxes &msg);
computer_vision::Points getPointCloud(double x, double y);
computer_vision::Points getPointCloud(computer_vision::BoundingBox);

map<string, Vector3d> blocks_type = {
    {"X1-Y1-Z2", Vector3d(0.8, 0.75, 0.6)},
    {"X1-Y2-Z1", Vector3d(0.7, 0.75, 0.6)},
    {"X1-Y2-Z2", Vector3d(0.6, 0.75, 0.6)},
    {"X1-Y2-Z2-CHAMFER", Vector3d(0.5, 0.75, 0.6)},
    {"X1-Y2-Z2-TWINFILLET", Vector3d(0.4, 0.75, 0.6)},
    {"X1-Y3-Z2", Vector3d(0.3, 0.75, 0.6)},
    {"X1-Y3-Z2-FILLET", Vector3d(0.2, 0.75, 0.6)},
    {"X1-Y4-Z1", Vector3d(0.9, 0.75, 0.6)},
    {"X1-Y4-Z2", Vector3d(0.1, 0.75, 0.6)},
    {"X2-Y2-Z2", Vector3d(0.1, 0.65, 0.6)},
    {"X2-Y2-Z2-FILLET", Vector3d(0.1, 0.55, 0.6)},
};

map<string, Vector3d> blocks_lenght = {
    {"X1-Y1-Z2", Vector3d(0.1, 0.1, 0.2)},
    {"X1-Y2-Z1", Vector3d(0.1, 0.2, 0.1)},
    {"X1-Y2-Z2", Vector3d(0.1, 0.2, 0.2)},
    {"X1-Y2-Z2-CHAMFER", Vector3d(0.1, 0.2, 0.2)},
    {"X1-Y2-Z2-TWINFILLET", Vector3d(0.1, 0.2, 0.2)},
    {"X1-Y3-Z2", Vector3d(0.1, 0.3, 0.2)},
    {"X1-Y3-Z2-FILLET", Vector3d(0.1, 0.3, 0.2)},
    {"X1-Y4-Z1", Vector3d(0.1, 0.4, 0.1)},
    {"X1-Y4-Z2", Vector3d(0.1, 0.4, 0.2)},
    {"X2-Y2-Z2", Vector3d(0.2, 0.2, 0.2)},
    {"X2-Y2-Z2-FILLET", Vector3d(0.2, 0.2, 0.2)},
};

#endif