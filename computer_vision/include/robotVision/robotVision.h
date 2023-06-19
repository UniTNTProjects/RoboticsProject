#ifndef __ROBOTVISION__
#define __ROBOTVISION__

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "computer_vision/BoundingBox.h"
#include "computer_vision/BoundingBoxes.h"
#include "computer_vision/Points.h"
#include "computer_vision/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "computer_vision/GetPoints.h"
void robot2DImageCallback(const computer_vision::BoundingBoxes &msg);

#endif