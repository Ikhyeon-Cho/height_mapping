/*
 * HeightMapping.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_H
#define HEIGHT_MAPPING_H

#include <ros/ros.h>

// Utility
#include "ros_utils/TransformHandler.h"
#include "ros_utils/pointcloud.h"

// Msgs
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>

// Height Map
#include <height_map_core/height_map_core.h>
#include <height_map_msgs/HeightMapMsgs.h>
#include <height_map_pcl/cloud_preprocessors.h>
// #include <height_map_core/DescriptorMap.h>

using PointT = pcl::PointXYZI;

class HeightMapping
{
public:
  HeightMapping();

  void measurementUpdate(const sensor_msgs::PointCloud2ConstPtr& msg);

  void updateMapPosition(const ros::TimerEvent& event);

  void visualize(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_{ "~" };
  ros_utils::TransformHandler tf_handler_;

  // Topics
  std::string pointcloud_topic_{ nh_.param<std::string>("pointCloudTopic", "/velodyne_points") };
  std::string heightmap_topic_{ nh_.param<std::string>("heightMapTopic", "elevation_grid") };
  std::string heightmap_cloud_topic_{ nh_.param<std::string>("heightMapCloudTopic", "elevation_cloud") };
  std::string featuremap_topic_{ nh_.param<std::string>("featureMapTopic", "elevation_feature") };

  // Frame Ids
  std::string baselink_frame_{ nh_.param<std::string>("baselinkFrame", "base_link") };
  std::string map_frame_{ nh_.param<std::string>("mapFrame", "map") };

  // Pointcloud Filter
  double height_min_thrsh_{ nh_.param<double>("minHeightThreshold", -0.5) };
  double height_max_thrsh_{ nh_.param<double>("maxHeightThreshold", 1.5) };
  double range_min_thrsh_{ nh_.param<double>("minRangeThreshold", 0.3) };
  double range_max_thrsh_{ nh_.param<double>("maxRangeThreshold", 10.0) };

  // Height Map Parameters
  double grid_resolution_{ nh_.param<double>("gridResolution", 0.1) };
  double map_length_x_{ nh_.param<double>("mapLengthX", 12) };
  double map_length_y_{ nh_.param<double>("mapLengthY", 12) };

  // Height Estimator Parameters
  std::string height_estimator_type_{ nh_.param<std::string>("heightEstimatorType", "KalmanFilter") };

  // Duration
  double pose_update_rate_{ nh_.param<double>("poseUpdateRate", 20) };
  double map_visualization_rate_{ nh_.param<double>("mapVisualizationRate", 10) };

  // ROS
  ros::Subscriber sub_lidar_pointcloud_{ nh_.subscribe(pointcloud_topic_, 10, &HeightMapping::measurementUpdate,
                                                       this) };
  ros::Publisher pub_downsampled_pointcloud_{ nh_.advertise<sensor_msgs::PointCloud2>("cloud_downsampled", 1) };
  ros::Publisher pub_heightmap_{ nh_.advertise<grid_map_msgs::GridMap>(heightmap_topic_, 1) };
  ros::Publisher pub_map_region_{ nh_.advertise<visualization_msgs::Marker>("map_region", 1) };
  ros::Publisher pub_featuremap_{ nh_.advertise<grid_map_msgs::GridMap>(featuremap_topic_, 1) };

  ros::Timer pose_update_timer_{ nh_.createTimer(pose_update_rate_, &HeightMapping::updateMapPosition, this) };
  ros::Timer map_visualization_timer_{ nh_.createTimer(map_visualization_rate_, &HeightMapping::visualize, this) };

private:
  grid_map::HeightMap map_{ map_length_x_, map_length_y_, grid_resolution_ };
  height_map::HeightEstimatorBase::Ptr height_estimator_;

  // Cloud processor
  // height_map::PointCloudProcessor cloud_preprocessor_; // For PointXYZ type
  height_map::IntensityCloudProcessor cloud_preprocessor_;
  // height_map::RGBCloudProcessor cloud_preprocessor_; // For PointXYZRGB type
};

#endif  // HEIGHT_MAPPING_H