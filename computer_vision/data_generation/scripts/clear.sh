#!/bin/bash

# This script deletes all files in image,valid,labels directories

rm -f ./dataset/train/images/*
rm -f ./dataset/train/labels/*
rm -f ./dataset/valid/images/*
rm -f ./dataset/valid/labels/*
rm -f ./dataset/test/images/*
rm -f ./dataset/test/labels/*


cp /home/squinkis/ros_ws/src/locosim/RoboticsProject/computer_vision/data_generation/MacroBLockV2Model/train/images/* ./dataset/train/images/
cp /home/squinkis/ros_ws/src/locosim/RoboticsProject/computer_vision/data_generation/MacroBLockV2Model/train/labels/* ./dataset/train/labels/
cp /home/squinkis/ros_ws/src/locosim/RoboticsProject/computer_vision/data_generation/MacroBLockV2Model/valid/images/* ./dataset/valid/images/
cp /home/squinkis/ros_ws/src/locosim/RoboticsProject/computer_vision/data_generation/MacroBLockV2Model/valid/labels/* ./dataset/valid/labels/
cp /home/squinkis/ros_ws/src/locosim/RoboticsProject/computer_vision/data_generation/MacroBLockV2Model/test/images/* ./dataset/test/images/
cp /home/squinkis/ros_ws/src/locosim/RoboticsProject/computer_vision/data_generation/MacroBLockV2Model/test/labels/* ./dataset/test/labels/



