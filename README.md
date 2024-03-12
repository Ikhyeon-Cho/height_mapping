# Height Mapping
This repository provides the `height_mapping` [ROS](https://www.ros.org/) package implemented for height mapping (or 2.5D elevation mapping) of complex 3D-shaped outdoor terrains.

## Overview
 The package is designed for the navigation task with ground mobile robots, equipped with a **distance sensor** (e.g. structured light (Kinect, RealSense), laser range finder, stereo camera) and a **3D pose estimator** (e.g. with 3D Visual, LiDAR Odometry / SLAM). The height of surroundings is locally mapped in a robot-centric perspective. 
 
 The packakge is developed on the basis of a [grid_map](https://github.com/ANYbotics/grid_map) C++ library, which is provided by [ANYbotics](https://github.com/ANYbotics). This is a research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


## System Architecture
- To be updated.



## Dependencies

This software is built on ROS1, which needs to be [installed](http://wiki.ros.org) first. Additionally, the following packages are requiared:

- [grid_map](https://github.com/anybotics/grid_map) (2D Grid Map library with multiple data layers)
- [eigen](http://eigen.tuxfamily.org) (Linear Algebra library)
- [pcl](http://pointclouds.org/) (Point cloud library)


## Build

Use the following commands to download and compile the package.

    cd ~/catkin_ws/src
    git clone https://github.com/Ikhyeon-Cho/height_mapping.git
    cd ../
    catkin build

## Run the package
In order to get the height map with your robot, you need to adapt a few parameters, which are specified in the params.yaml file in `height_mapping_ros/launch/config` folder.

To start the height mapping, use command

    roslaunch height_mapping run.launch
