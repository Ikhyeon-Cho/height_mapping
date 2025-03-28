#pragma once

#include <string>
#include <ros/node_handle.h>

struct FrameID {
  std::string robot;
  std::string map;
  std::string sensor;

  static FrameID loadFromConfig(const ros::NodeHandle &nh) {
    FrameID frame;
    frame.robot = nh.param<std::string>("robot_frame", "base_link");
    frame.map = nh.param<std::string>("map_frame", "map");
    frame.sensor = nh.param<std::string>("sensor_frame", "");
    return frame;
  }
};
