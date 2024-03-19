/*
 * HeightMapping.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/HeightMapping.h"
#include <chrono>

HeightMapping::HeightMapping()
{
  map_.setFrameId(map_frame_);
}

void HeightMapping::updateHeight(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Convert msg to pcl::PointCloud
  auto cloud_raw = boost::make_shared<pcl::PointCloud<pType>>();
  pcl::fromROSMsg(*msg, *cloud_raw);  // moveFromROSMsg is faster (100 ~ 200 us) than fromROSMsg

  // Transform pointcloud to base_link frame
  const auto& sensor_frame = msg->header.frame_id;
  auto [has_transform_s2b, sensor_to_base] = tf_handler_.getTransform(sensor_frame, baselink_frame_);
  if (!has_transform_s2b)
    return;

  auto cloud_base = ros_utils::pcl::transformPointcloud<pType>(cloud_raw, sensor_to_base);

  // Filter pointcloud by height and range
  auto cloud_filtered =
      ros_utils::pcl::filterPointcloudByField<pType>(cloud_base, "z", height_min_thrsh_, height_max_thrsh_);
  cloud_filtered = ros_utils::pcl::filterPointcloudByRange<pType>(cloud_filtered, range_min_thrsh_, range_max_thrsh_);

  // Transform pointcloud to map frame
  auto [has_transform_b2m, base_to_map] = tf_handler_.getTransform(baselink_frame_, map_frame_);
  if (!has_transform_b2m)
    return;

  auto cloud_map = ros_utils::pcl::transformPointcloud<pType>(cloud_filtered, base_to_map);

  auto cloud_registered = map_.getGridDownsampledCloud(*cloud_map);
  map_.update(*cloud_registered, "KalmanFilter+ConsistencyCheck");

  // For Debug: Publish preprocessed pointcloud
  sensor_msgs::PointCloud2 msg_pc;
  pcl::toROSMsg(*cloud_registered, msg_pc);
  pub_registered_pointcloud_.publish(msg_pc);
}

void HeightMapping::updatePosition(const ros::TimerEvent& event)
{
  auto [has_transform_b2m, base_to_map] = tf_handler_.getTransform(baselink_frame_, map_frame_);
  if (!has_transform_b2m)
    return;

  map_.move(grid_map::Position(base_to_map.transform.translation.x, base_to_map.transform.translation.y));
}

void HeightMapping::visualize(const ros::TimerEvent& event)
{
  // Grid Map Visualization
  grid_map_msgs::GridMap msg_gridmap;
  // grid_map::GridMapRosConverter::toMessage(map_, map_.getBasicLayers(), msg_gridmap);
  grid_map::GridMapRosConverter::toMessage(map_, msg_gridmap);
  pub_heightmap_.publish(msg_gridmap);

  // Feature Map Visualization
  // grid_map_msgs::GridMap msg_featuremap;
  // grid_map::GridMapRosConverter::toMessage(descriptor_map_, msg_featuremap);
  // pub_featuremap_.publish(msg_featuremap);
}