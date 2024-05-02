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
#include <height_map_pcl/PCLProcessor.h>

#include <jsk_rviz_plugins/OverlayText.h>

using Laser = pcl::PointXYZI;
using Color = pcl::PointXYZRGB;

class HeightMapping
{
public:
  HeightMapping();

  void updateFromLaserCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

  void updateFromRGBCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

  void updatePosition(const ros::TimerEvent& event);

  void publishHeightmap(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_priv{ "~" };
  utils::TransformHandler tf_tree_;

  // Topics
  std::string lidarcloud_topic_{ nh_priv.param<std::string>("lidarCloudTopic", "/sensor_processor/laser/points") };
  std::string rgbcloud_topic_{ nh_priv.param<std::string>("rgbCloudTopic", "/sensor_processor/color/points") };

  // Frame Ids
  std::string baselink_frame{ nh_priv.param<std::string>("baselinkFrame", "base_link") };
  std::string odom_frame{ nh_priv.param<std::string>("odometryFrame", "odom") };
  std::string map_frame{ nh_priv.param<std::string>("mapFrame", "map") };

  // Pointcloud Preprocessing Parameters
  double height_min_thrsh_{ nh_priv.param<double>("minHeightThreshold", -0.5) };
  double height_max_thrsh_{ nh_priv.param<double>("maxHeightThreshold", 1.5) };
  double range_min_thrsh_{ nh_priv.param<double>("minRangeThreshold", 0.3) };
  double range_max_thrsh_{ nh_priv.param<double>("maxRangeThreshold", 10.0) };
  double depth_min_thrsh_{ nh_priv.param<double>("minDepthThreshold", 1.0) };
  double depth_max_thrsh_{ nh_priv.param<double>("maxDepthThreshold", 5.0) };

  // Height Map Parameters
  double grid_resolution_{ nh_priv.param<double>("gridResolution", 0.1) };
  double map_length_x_{ nh_priv.param<double>("mapLengthX", 12) };
  double map_length_y_{ nh_priv.param<double>("mapLengthY", 12) };
  std::string height_estimator_type_{ nh_priv.param<std::string>("heightEstimatorType", "KalmanFilter") };

  // Timer
  double pose_update_rate_{ nh_priv.param<double>("poseUpdateRate", 20) };
  double heightmap_pub_rate_{ nh_priv.param<double>("mapPublishRate", 10) };

  // Debug Flag
  bool debug_{ nh_priv.param<bool>("debugMode", false) };

  // ROS
  ros::Subscriber sub_lidar_{ nh_priv.subscribe(lidarcloud_topic_, 1, &HeightMapping::updateFromLaserCloud, this) };
  ros::Subscriber sub_rgbd_{ nh_priv.subscribe(rgbcloud_topic_, 1, &HeightMapping::updateFromRGBCloud, this) };
  ros::Publisher pub_laser_downsampled_{ nh_priv.advertise<sensor_msgs::PointCloud2>("laser_downsampled", 1) };
  ros::Publisher pub_rgbd_downsampled_{ nh_priv.advertise<sensor_msgs::PointCloud2>("rgbd_downsampled", 1) };
  ros::Publisher pub_heightmap_{ nh_priv.advertise<grid_map_msgs::GridMap>("gridmap", 1) };
  ros::Publisher pub_laser_processing_time_{ nh_priv.advertise<jsk_rviz_plugins::OverlayText>("laser_processing_time",
                                                                                              1) };
  ros::Publisher pub_rgbd_processing_time_{ nh_priv.advertise<jsk_rviz_plugins::OverlayText>("rgbd_processing_time",
                                                                                             1) };

  ros::Timer robot_pose_update_timer_{ nh_priv.createTimer(pose_update_rate_, &HeightMapping::updatePosition, this,
                                                           false, false) };  // oneshot = false, autostart = false
  ros::Timer pub_heightmap_timer_{ nh_priv.createTimer(heightmap_pub_rate_, &HeightMapping::publishHeightmap, this) };

private:
  grid_map::HeightMap map_{ map_length_x_, map_length_y_, grid_resolution_ };
  height_map::HeightEstimatorBase::Ptr height_estimator_;

  // Cloud processor
  height_map::IntensityCloudProcessor lasercloud_preprocessor_;  // For PointXYZI type
  height_map::RGBCloudProcessor rgbcloud_preprocessor_;          // For PointXYZRGB type
  height_map::pclProcessor test_processor_;

  bool lasercloud_received_{ false };
  bool rgbcloud_received_{ false };
};

#endif  // HEIGHT_MAPPING_H