# Height Map

## Overview

This is a [ROS](https://www.ros.org/) package implemented for height mapping (or 2.5D elevation mapping) of 3D terrains for a mobile robot. The package is designed for the navigation tasks with ground robots which are equipped with a 3D pose estimator (e.g. with 3D Visual, LiDAR Odometry / SLAM) and a distance sensor (e.g. structured light (Kinect, RealSense), laser range finder, stereo camera). The height map is locally mapped with a robot-centric perspective. The packakge is developed on the basis of a [grid_map](https://github.com/ANYbotics/grid_map) C++ library, which is from [ANYbotics](https://github.com/ANYbotics).

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.


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
    git clone https://github.com/Ikhyeon-Cho/height_map.git
    cd ../
    catkin build -DCMAKE_BUILD_TYPE=Release

## Run the package
In order to get the height map to run with your robot, you will need to adapt a few parameters. These are specifically the parameter files in `height_map_ros/config` folder.

To start with height mapping, use command

    roslaunch height_map run.launch
