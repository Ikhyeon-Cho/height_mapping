#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <height_mapping_core/height_mapping_core.h>
#include <filesystem>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/wall_timer.h>

class DataVisualizationNode {
public:
  DataVisualizationNode();
  ~DataVisualizationNode() = default;

private:
  void getParameters();
  void setupROSInterface();
  void publishNextCloud(const ros::WallTimerEvent& event);
  pcl::PointCloud<Laser>::Ptr readPCDFile(const std::string& filename);

  // ROS members
  ros::NodeHandle nh_;
  ros::NodeHandle nhPriv_{"~"};
  ros::Publisher pubCloud_;
  ros::WallTimer publishTimer_;

  // Parameters
  std::string dataDirectory_;
  double publishRate_;
  std::string frameId_;

  // State
  std::vector<std::string> cloudFiles_;
  size_t currentFileIdx_{0};
}; 