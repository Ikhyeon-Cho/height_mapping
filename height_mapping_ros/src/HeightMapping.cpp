/*
 * HeightMapping.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/HeightMapping.h"
#include <chrono>

HeightMapping::HeightMapping()
{
  // Set frame id
  map_.setFrameId(map_frame_);

  // Set height estimator
  if (height_estimator_type_ == "KalmanFilter")
  {
    height_estimator_ = std::make_unique<height_map::KalmanEstimator>();
  }
  else if (height_estimator_type_ == "MovingAverage")
  {
    height_estimator_ = std::make_unique<height_map::MovingAverageEstimator>();
  }
  else if (height_estimator_type_ == "SimpleMean")
  {
    height_estimator_ = std::make_unique<height_map::SimpleMeanEstimator>();
  }
  else
  {
    ROS_WARN("[HeightMapping] Invalid height estimator type. Set Default: KalmanFilter");
    height_estimator_ = std::make_unique<height_map::KalmanEstimator>();
  }
}

void HeightMapping::measurementUpdate(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Convert msg to pcl::PointCloud
  auto cloud_raw = boost::make_shared<pcl::PointCloud<PointT>>();
  pcl::fromROSMsg(*msg, *cloud_raw);  // moveFromROSMsg is faster (100 ~ 200 us) than fromROSMsg

  // Get Transform from sensor to base_link
  const auto& sensor_frame = msg->header.frame_id;
  auto [get_transform_s2b, sensor_to_base] = tf_handler_.getTransform(sensor_frame, baselink_frame_);
  if (!get_transform_s2b)
    return;

  // Transform pointcloud to base_link frame
  auto cloud_base = ros_utils::pcl::transformPointcloud<PointT>(cloud_raw, sensor_to_base);

  // Filter pointcloud by height and range
  auto cloud_filtered =
      ros_utils::pcl::filterPointcloudByField<PointT>(cloud_base, "z", height_min_thrsh_, height_max_thrsh_);
  cloud_filtered = ros_utils::pcl::filterPointcloudByRange<PointT>(cloud_filtered, range_min_thrsh_, range_max_thrsh_);

  // Get Transform from base_link to map
  auto [get_transform_b2m, base_to_map] = tf_handler_.getTransform(baselink_frame_, map_frame_);
  if (!get_transform_b2m)
    return;

  // Transform pointcloud to map frame
  auto cloud_map = ros_utils::pcl::transformPointcloud<PointT>(cloud_filtered, base_to_map);

  auto start = std::chrono::high_resolution_clock::now();

  // Downsampling of pointcloud by grid -> each grid cell has only one point (with max height)
  auto [get_cloud_downsampled, cloud_downsampled] = cloud_preprocessor_.gridDownsampling(*cloud_map, map_);
  if (!get_cloud_downsampled)
    return;

  auto [get_cloud_consistent, cloud_consistent] = cloud_preprocessor_.removeInconsistentPoints(*cloud_downsampled, map_);

  // Estimate height of each grid cell
  height_estimator_->estimate(map_, *cloud_consistent);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "Height mapping takes: " << duration.count() << " microseconds" << std::endl;

  // For Debug: Publish preprocessed pointcloud
  sensor_msgs::PointCloud2 msg_pc;
  pcl::toROSMsg(*cloud_consistent, msg_pc);
  pub_downsampled_pointcloud_.publish(msg_pc);
}

void HeightMapping::updateMapPosition(const ros::TimerEvent& event)
{
  // Get Transform from base_link to map (localization pose)
  auto [get_transform_b2m, base_to_map] = tf_handler_.getTransform(baselink_frame_, map_frame_);
  if (!get_transform_b2m)
    return;

  // Update map position
  map_.move(grid_map::Position(base_to_map.transform.translation.x, base_to_map.transform.translation.y));
}

void HeightMapping::visualize(const ros::TimerEvent& event)
{
  // Height Map Visualization
  grid_map_msgs::GridMap msg_heightmap;
  grid_map::GridMapRosConverter::toMessage(map_, msg_heightmap);
  pub_heightmap_.publish(msg_heightmap);

  // Feature Map Visualization
  // grid_map_msgs::GridMap msg_featuremap;
  // grid_map::GridMapRosConverter::toMessage(descriptor_map_, msg_featuremap);
  // pub_featuremap_.publish(msg_featuremap);
}
