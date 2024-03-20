/*
 * GlobalMapping.h
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef GLOBAL_MAPPING_H
#define GLOBAL_MAPPING_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <height_map_core/HeightMap.h>
#include <height_map_msgs/HeightMapMsgs.h>

class GlobalMapping
{
public:
  GlobalMapping();

  void registerLocalMap(const sensor_msgs::PointCloud2ConstPtr& msg);

  void visualize(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_{ "height_mapping" };
  ros::NodeHandle pnh_{ "~" };

  // Topics
  std::string pointcloud_topic_{ nh_.param<std::string>("heightMapCloudTopic", "/elevation_cloud") };
  std::string globalmap_topic_{ pnh_.param<std::string>("globalMapTopic", "elevation_global") };
  std::string map_region_topic_{ pnh_.param<std::string>("mapRegionTopic", "map_region_global") };

  // Frame Ids
  std::string baselink_frame_{ nh_.param<std::string>("baselinkFrame", "base_link") };
  std::string map_frame_{ nh_.param<std::string>("mapFrame", "map") };

  // Global Map parameters
  double grid_resolution_{ pnh_.param<double>("gridResolution", 0.1) };
  double map_length_x_{ pnh_.param<double>("mapLengthXGlobal", 400) };
  double map_length_y_{ pnh_.param<double>("mapLengthYGlobal", 400) };

  // Duration
  double map_visualization_rate_{ pnh_.param<double>("globalMapVisualizationRate", 1.0) };

  // ROS
  ros::Subscriber sub_pointcloud_{ nh_.subscribe(pointcloud_topic_, 10, &GlobalMapping::registerLocalMap, this) };
  ros::Publisher pub_globalmap_{ pnh_.advertise<sensor_msgs::PointCloud2>(globalmap_topic_, 1) };
  ros::Publisher pub_map_region_{ pnh_.advertise<visualization_msgs::Marker>(map_region_topic_, 1) };
  
  ros::Timer map_visualization_timer_{ nh_.createTimer(map_visualization_rate_, &GlobalMapping::visualize, this) };

private:
  grid_map::HeightMap map_{ map_length_x_, map_length_y_, grid_resolution_ };
};

#endif  // GLOBAL_MAPPING_H