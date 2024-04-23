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
  // Set heightmap frame_id
  map_.setFrameId(map_frame);

  // Set height_estimator
  if (height_estimator_type_ == "KalmanFilter")
  {
    height_estimator_ = std::make_unique<height_map::KalmanEstimator>();
  }
  else if (height_estimator_type_ == "MovingAverage")
  {
    height_estimator_ = std::make_unique<height_map::MovingAverageEstimator>();
  }
  else if (height_estimator_type_ == "StatMean")
  {
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  }
  else
  {
    ROS_WARN("[HeightMapping] Invalid height estimator type. Set Default: StatMean");
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  }
}

void HeightMapping::updateFromCloudLaser(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (lasercloud_received_ == false)
  {
    robot_pose_update_timer_.start();

    ROS_INFO("[HeightMapping] Laser Cloud Received! Use laser cloud for height mapping...");
    lasercloud_received_ = true;
  }

  auto start = std::chrono::high_resolution_clock::now();

  // Convert msg to pcl::PointCloud<pcl::PointXYZI>
  auto lasercloud_raw = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::fromROSMsg(*msg, *lasercloud_raw);  // !Note: moveFromROSMsg is faster (100 ~ 200 us) than fromROSMsg

  // Transform pointcloud to base_link is needed for range-based filtering
  auto sensor_frame = msg->header.frame_id;
  auto [get_transform_s2b, sensor_to_baselink] = tf_tree_.getTransform(sensor_frame, baselink_frame);
  if (!get_transform_s2b)
    return;
  auto lasercloud_baselink = utils::pcl::transformPointcloud<Laser>(lasercloud_raw, sensor_to_baselink);

  // Filter pointcloud by range (in x-y) and height (in z)
  auto lasercloud_filtered =
      utils::pcl::filterPointcloudByRange2D<Laser>(lasercloud_baselink, range_min_thrsh_, range_max_thrsh_);
  lasercloud_filtered =
      utils::pcl::filterPointcloudByField<Laser>(lasercloud_filtered, "z", height_min_thrsh_, height_max_thrsh_);

  // Register laser cloud to the map
  auto [get_transform_b2m, base_to_map] = tf_tree_.getTransform(baselink_frame, map_frame);
  if (!get_transform_b2m)
    return;
  auto lasercloud_to_be_registered = utils::pcl::transformPointcloud<Laser>(lasercloud_filtered, base_to_map);

  // Cloud Downsampling by grid resolution-> each grid cell has only one point, with max height
  auto [get_cloud_downsampled, lasercloud_downsampled] =
      lasercloud_preprocessor_.gridDownsampling(*lasercloud_to_be_registered, map_);
  if (!get_cloud_downsampled)
    return;

  auto [get_cloud_consistent, lasercloud_consistent] =
      lasercloud_preprocessor_.removeInconsistentPoints(*lasercloud_downsampled, map_);
  if (!get_cloud_consistent)
    return;

  // Estimate height of each grid cell
  height_estimator_->estimate(map_, *lasercloud_consistent);

  // For Debug:
  // Publish preprocessed pointcloud
  // Publish processing time
  if (debug_)
  {
    sensor_msgs::PointCloud2 msg_pc;
    pcl::toROSMsg(*lasercloud_consistent, msg_pc);
    pub_laser_downsampled_.publish(msg_pc);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    jsk_rviz_plugins::OverlayText overlay_text;
    overlay_text.text = "Laser Processing time: " + std::to_string(duration.count()) + " ms";
    pub_laser_processing_time_.publish(overlay_text);
  }
}

void HeightMapping::updateFromCloudRGB(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  robot_pose_update_timer_.start();

  auto start = std::chrono::high_resolution_clock::now();

  // Convert msg to pcl::PointCloud<pcl::PointXYZRGB>
  auto rgbcloud_raw = boost::make_shared<pcl::PointCloud<Color>>();
  pcl::fromROSMsg(*msg, *rgbcloud_raw);

  auto rgbcloud_downsampled = utils::pcl::filterPointcloudByVoxel<Color>(rgbcloud_raw, grid_resolution_ * 0.5);

  // Transform pointcloud to base_link is needed for range-based filtering
  auto sensor_frame = msg->header.frame_id;
  auto [get_transform_s2b, sensor_to_baselink] = tf_tree_.getTransform(sensor_frame, baselink_frame);
  if (!get_transform_s2b)
    return;
  auto rgbcloud_baselink = utils::pcl::transformPointcloud<Color>(rgbcloud_downsampled, sensor_to_baselink);

  // Filter pointcloud by range (in x-y) and depth (in z)
  auto rgbcloud_filtered =
      utils::pcl::filterPointcloudByRange2D<Color>(rgbcloud_baselink, depth_min_thrsh_, depth_max_thrsh_);
  rgbcloud_filtered =
      utils::pcl::filterPointcloudByField<Color>(rgbcloud_filtered, "z", height_min_thrsh_, height_max_thrsh_);

  // Register rgb cloud to the map
  auto [get_transform_b2m, base_to_map] = tf_tree_.getTransform(baselink_frame, map_frame);
  if (!get_transform_b2m)
    return;
  auto rgbcloud_registered = utils::pcl::transformPointcloud<Color>(rgbcloud_filtered, base_to_map);

  // Estimate height of each grid cell
  height_estimator_->estimate(map_, *rgbcloud_registered);

  // For Debug:
  // Publish preprocessed pointcloud
  // Publish processing time
  if (debug_)
  {
    sensor_msgs::PointCloud2 msg_pc;
    pcl::toROSMsg(*rgbcloud_registered, msg_pc);
    pub_rgbd_downsampled_.publish(msg_pc);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    jsk_rviz_plugins::OverlayText overlay_text;
    overlay_text.text = "RGBD Processing time: " + std::to_string(duration.count()) + " ms";
    pub_rgbd_processing_time_.publish(overlay_text);
  }
}

void HeightMapping::updatePosition(const ros::TimerEvent& event)
{
  // Get Transform from base_link to map (localization pose)
  auto [get_transform_b2m, base_to_map] = tf_tree_.getTransform(baselink_frame, map_frame);
  if (!get_transform_b2m)
    return;
  // Update map position
  map_.move(grid_map::Position(base_to_map.transform.translation.x, base_to_map.transform.translation.y));
}

void HeightMapping::publishGridmap(const ros::TimerEvent& event)
{
  // Height Map Visualization
  grid_map_msgs::GridMap msg_heightmap;
  grid_map::GridMapRosConverter::toMessage(map_, msg_heightmap);
  pub_heightmap_.publish(msg_heightmap);
}
