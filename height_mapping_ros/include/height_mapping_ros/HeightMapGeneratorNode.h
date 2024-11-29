#pragma once

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include "height_mapping_ros/CloudTypes.h"
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>

struct pair_hash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& pair) const {
    auto h1 = std::hash<T1>{}(pair.first);
    auto h2 = std::hash<T2>{}(pair.second);
    return h1 ^ (h2 << 1);
  }
};

class HeightMapGeneratorNode {
public:
  HeightMapGeneratorNode();
  ~HeightMapGeneratorNode() = default;

private:
  void getParameters();
  void setupROSInterface();
  void generateHeightMap();
  pcl::PointCloud<Laser>::Ptr readPCDFile(const std::string& filename);
  void publishHeightMap(const ros::WallTimerEvent& event);
  void processPointCloud(const pcl::PointCloud<Laser>::Ptr& cloud);
  pcl::PointCloud<Laser>::Ptr griddedFilterWithMaxHeight(
      const pcl::PointCloud<Laser>::Ptr& cloud, float gridSize);
  pcl::PointCloud<Laser>::Ptr raycastFiltering(
      const pcl::PointCloud<Laser>::Ptr& cloud);
  bool isPointVisible(const Laser& point, 
                     const std::vector<std::vector<float>>& height_map,
                     const std::vector<std::vector<bool>>& visibility_map,
                     float cell_size, float min_x, float min_y);

  // ROS members
  ros::NodeHandle nh_;
  ros::NodeHandle nhPriv_{"~"};
  ros::Publisher pubHeightMap_;
  ros::WallTimer publishTimer_;

  // Grid map
  grid_map::GridMap heightMap_;
  
  // Parameters
  std::string dataDirectory_;
  double publishRate_;
  double mapResolution_;
  double minHeight_;
  double maxHeight_;
  int numScansToAggregate_;
  std::string frameId_;

  // State
  std::vector<std::string> cloudFiles_;
}; 