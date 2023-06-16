#!/bin/bash

# This script deletes all files in image,valid,labels directories

rm -f ./dataset/train/images/*
rm -f ./dataset/train/labels/*
rm -f ./dataset/valid/images/*
rm -f ./dataset/valid/labels/*
