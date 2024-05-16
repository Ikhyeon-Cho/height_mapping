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
#include <height_map_pcl/pclProcessor.h>

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
  ros::NodeHandle nh_priv_{ "~" };
  utils::TransformHandler tf_;

  // Topics
  std::string lidarcloud_topic_{ nh_priv_.param<std::string>("lidarCloudTopic",
                                                             "/height_mapping/sensor/laser/points") };
  std::string rgbcloud_topic_{ nh_priv_.param<std::string>("rgbCloudTopic",
                                                           "/height_mapping/sensor/color/points") };

  // Frame Ids
  std::string baselink_frame{ nh_priv_.param<std::string>("/frame_id/base_link", "base_link") };
  std::string odom_frame{ nh_priv_.param<std::string>("/frame_id/odom", "odom") };
  std::string map_frame{ nh_priv_.param<std::string>("/frame_id/map", "map") };

  // Height Map Parameters
  double grid_resolution_{ nh_priv_.param<double>("gridResolution", 0.1) };
  double map_length_x_{ nh_priv_.param<double>("mapLengthX", 10) };
  double map_length_y_{ nh_priv_.param<double>("mapLengthY", 10) };
  std::string height_estimator_type_{ nh_priv_.param<std::string>("heightEstimatorType", "StatMean") };
  double height_min_thrsh_{ nh_priv_.param<double>("minHeightThreshold", -0.5) };
  double height_max_thrsh_{ nh_priv_.param<double>("maxHeightThreshold", 1.5) };

  // Timer
  double pose_update_rate_{ nh_priv_.param<double>("poseUpdateRate", 20) };
  double heightmap_pub_rate_{ nh_priv_.param<double>("mapPublishRate", 10) };

  // ROS
  ros::Subscriber sub_lidar_{ nh_priv_.subscribe(lidarcloud_topic_, 1, &HeightMapping::updateFromLaserCloud, this) };
  ros::Subscriber sub_rgbd_{ nh_priv_.subscribe(rgbcloud_topic_, 1, &HeightMapping::updateFromRGBCloud, this) };
  ros::Publisher pub_heightmap_{ nh_priv_.advertise<grid_map_msgs::GridMap>("/height_mapping/map/gridmap", 1) };

  // Debug Flag
  bool debug_{ nh_priv_.param<bool>("debugMode", false) };
  ros::Publisher pub_laser_downsampled_;
  ros::Publisher pub_rgbd_downsampled_;
  ros::Publisher pub_laser_processing_time_;
  ros::Publisher pub_rgbd_processing_time_;

  ros::Timer robot_pose_update_timer_{ nh_priv_.createTimer(pose_update_rate_, &HeightMapping::updatePosition, this,
                                                            false, false) };  // oneshot = false, autostart = false
  ros::Timer pub_heightmap_timer_{ nh_priv_.createTimer(heightmap_pub_rate_, &HeightMapping::publishHeightmap, this) };

private:
  grid_map::HeightMap map_{ map_length_x_, map_length_y_, grid_resolution_ };
  height_map::HeightEstimatorBase::Ptr height_estimator_;

  bool lasercloud_received_{ false };
  bool rgbcloud_received_{ false };
};

#endif  // HEIGHT_MAPPING_H