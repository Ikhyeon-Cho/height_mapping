/*
 * GlobalTerrainMapping.h
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_GLOBAL_TERRAIN_MAPPING_H
#define ROS_GLOBAL_TERRAIN_MAPPING_H

#include <ros/ros.h>

#include <ros_node_utils/Core.h>
#include <ros_transform_utils/TransformHandler.h>
#include <ros_pcl_utils/PointcloudProcessor.h>

#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include "terrain_mapping/ElevationMap.h"
#include "terrain_mapping/ElevationMapRosConverter.h"
#include "terrain_mapping/DescriptorMap.h"

#include <set>

namespace ros
{
class GlobalTerrainMapping
{
public:
  GlobalTerrainMapping();

  void updateMap(const sensor_msgs::PointCloud2ConstPtr& msg);

  void visualize(const ros::TimerEvent& event);

  // Helper function for visualization
private:
  void saveMeasuredIndices(const pcl::PointCloud<pcl::PointXYZI>& pointcloud);

  void visualizeGlobalMap();

  struct Array2iComparator
  {
    bool operator()(const Eigen::Array2i& a, const Eigen::Array2i& b) const
    {
      return (a.x() < b.x()) || ((a.x() == b.x()) && (a.y() < b.y()));
    }
  };

public:
  // ROS Parameters: Node
  roscpp::Parameter<std::string> pointcloud_topic{ "global_terrain_mapping/SubscribedTopic/pointcloud",
                                                   "/velodyne_points" };
  roscpp::Parameter<std::string> elevation_map_topic{ "global_terrain_mapping/PublishingTopic/elevation_map", "map" };
  roscpp::Parameter<std::string> descriptor_map_topic{ "global_terrain_mapping/PublishingTopic/descriptor_map",
                                                       "descriptor" };

  // ROS Parameters : Framd Ids
  roscpp::Parameter<std::string> frameId_robot{ "frameId_robot", "base_link" };
  roscpp::Parameter<std::string> frameId_map{ "frameId_map", "map" };

  // Elevation Map
  roscpp::Subscriber<sensor_msgs::PointCloud2> pointcloud_subscriber{ pointcloud_topic.param(),
                                                                      &GlobalTerrainMapping::updateMap, this };
  roscpp::Publisher<sensor_msgs::PointCloud2> elevation_map_publisher{ elevation_map_topic.param() };
  roscpp::Publisher<grid_map_msgs::GridMap> descriptor_map_publisher{ descriptor_map_topic.param() };

  roscpp::Parameter<double> map_length_x{ "global_terrain_mapping/ElevationMap/map_length_x", 12 };
  roscpp::Parameter<double> map_length_y{ "global_terrain_mapping/ElevationMap/map_length_y", 12 };
  roscpp::Parameter<double> grid_resolution{ "global_terrain_mapping/ElevationMap/grid_resolution", 0.1 };
  roscpp::Parameter<double> min_height_thres{"global_terrain_mapping/ElevationMap/min_height_threshold", -0.7};
  roscpp::Parameter<double> max_height_thres{"global_terrain_mapping/ElevationMap/max_height_threshold", 0.7};
  roscpp::Parameter<double> min_range_thres{"global_terrain_mapping/ElevationMap/min_range_threshold", 0.5};
  roscpp::Parameter<double> max_range_thres{"global_terrain_mapping/ElevationMap/max_range_threshold", 10.0};

  // Map Viauzliation
  roscpp::Parameter<double> map_visualization_duration{ "global_terrain_mapping/Visualization/duration", 0.1 };
  roscpp::Timer map_visualization_timer{ map_visualization_duration, &GlobalTerrainMapping::visualize, this };
  roscpp::Publisher<visualization_msgs::Marker> map_boundary_publisher{ "map_boundary" };
  roscpp::Publisher<nav_msgs::OccupancyGrid> occupancy_map_publisher{ "occu_map" };

private:
  ElevationMap map_{ map_length_x.param(), map_length_y.param(), grid_resolution.param() };
  DescriptorMap descriptor_map_{ map_ };

  TransformHandler transform_handler_;
  PointcloudProcessor<pcl::PointXYZI> pointcloud_processor_;

  std::set<Eigen::Array2i, Array2iComparator> measured_index_set_;
};

}  // namespace ros

#endif