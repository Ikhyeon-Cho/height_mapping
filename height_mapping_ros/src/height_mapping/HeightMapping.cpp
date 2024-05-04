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
  map_.setFrameId(map_frame);

  // Set height_estimator:
  // - Kalman Filter
  // - Moving Average
  // - StatMean (by default)
  if (height_estimator_type_ == "KalmanFilter")
  {
    std::cout << "\033[1;33m[HeightMapping]: Height estimator type ---> KalmanFilter \033[0m\n";
    height_estimator_ = std::make_unique<height_map::KalmanEstimator>();
  }
  else if (height_estimator_type_ == "MovingAverage")
  {
    std::cout << "\033[1;33m[HeightMapping]: Height estimator type ---> MovingAverage \033[0m\n";
    height_estimator_ = std::make_unique<height_map::MovingAverageEstimator>();
  }
  else if (height_estimator_type_ == "StatMean")
  {
    std::cout << "\033[1;33m[HeightMapping]: Height estimator type ---> StatisticalMeanEstimator \033[0m\n";
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  }
  else
  {
    ROS_WARN("[HeightMapping] Invalid height estimator type. Set Default: StatMeanEstimator");
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  }
}

void HeightMapping::updateFromLaserCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (!lasercloud_received_)
  {
    lasercloud_received_ = true;
    std::cout << "\033[32m[HeightMapping]: Laser Cloud Received! Use LiDAR pointcloud for height mapping... \033[0m\n";
    robot_pose_update_timer_.start();
  }

  // Timer for benchmarking:
  auto start_begin = std::chrono::high_resolution_clock::now();
  auto start = std::chrono::high_resolution_clock::now();

  // Msg conversion
  auto lasercloud_raw = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::fromROSMsg(*msg, *lasercloud_raw);  // !Note: moveFromROSMsg is faster (100 ~ 200 us) than fromROSMsg

  auto end = std::chrono::high_resolution_clock::now();
  auto duration_conversion = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  start = end;

  // Transform pointcloud to base_link
  // Required for height filtering: If already in base_link, then will return the same pointcloud
  const auto& sensor_frame = msg->header.frame_id;
  auto [get_transform_s2b, sensor_to_baselink] = tf_.getTransform(sensor_frame, baselink_frame);
  if (!get_transform_s2b)
    return;
  auto lasercloud_baselink = utils::pcl::transformPointcloud<Laser>(lasercloud_raw, sensor_to_baselink);

  end = std::chrono::high_resolution_clock::now();
  auto duration_transform = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  start = end;

  // Filter pointcloud by height (z-axis)
  auto lasercloud_filtered =
      utils::pcl::filterPointcloudByField<Laser>(lasercloud_baselink, "z", height_min_thrsh_, height_max_thrsh_);

  end = std::chrono::high_resolution_clock::now();
  auto duration_filter = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  start = end;

  // Register laser cloud to the map
  auto [get_transform_b2m, base_to_map] = tf_.getTransform(baselink_frame, map_frame);
  if (!get_transform_b2m)
  {
    std::cout << "\033[33m[HeightMapping]: Failed to register the cloud. Skip this frame... \033[0m\n";
    return;
  }
  auto lasercloud_registered = utils::pcl::transformPointcloud<Laser>(lasercloud_filtered, base_to_map);

  end = std::chrono::high_resolution_clock::now();
  auto duration_registration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  start = end;

  // Downsampling by height map resolution-> each grid cell has only one point, with max height
  auto lasercloud_registered_downsampled =
      height_map::pclProcessor::gridDownsampling<Laser>(lasercloud_registered, grid_resolution_);

  end = std::chrono::high_resolution_clock::now();
  auto duration_downsampling = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  start = end;

  // Estimate height of each grid cell
  height_estimator_->estimate(map_, *lasercloud_registered_downsampled);

  end = std::chrono::high_resolution_clock::now();
  auto duration_estimation = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  // For Debug:
  // - Publish preprocessed pointcloud
  // - Publish processing time
  if (debug_)
  {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_begin);

    // Publish preprocessed pointcloud
    sensor_msgs::PointCloud2 msg_pc;
    pcl::toROSMsg(*lasercloud_registered_downsampled, msg_pc);
    pub_laser_downsampled_.publish(msg_pc);

    // Publish processing time
    jsk_rviz_plugins::OverlayText overlay_text;
    overlay_text.height = 300;
    overlay_text.width = 300;
    overlay_text.text = "Laser Processing time: " + std::to_string(duration.count()) + " us" +
                        "\nConversion: " + std::to_string(duration_conversion.count()) + " us" +
                        "\nTransform: " + std::to_string(duration_transform.count()) + " us" +
                        "\nFilter: " + std::to_string(duration_filter.count()) + " us" +
                        "\nRegistration: " + std::to_string(duration_registration.count()) + " us" +
                        "\nDownsampling: " + std::to_string(duration_downsampling.count()) + " us" +
                        "\nEstimation: " + std::to_string(duration_estimation.count()) + " us";
    pub_laser_processing_time_.publish(overlay_text);
  }
}

void HeightMapping::updateFromRGBCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (!rgbcloud_received_)
  {
    rgbcloud_received_ = true;
    std::cout << "\033[32m[HeightMapping]: RGB Cloud Received! Use RGB-D pointcloud for height mapping... \033[0m\n";
    robot_pose_update_timer_.start();
  }

  // Timer for benchmarking:
  auto start_begin = std::chrono::high_resolution_clock::now();
  auto start = std::chrono::high_resolution_clock::now();

  // Msg conversion
  auto rgbcloud_raw = boost::make_shared<pcl::PointCloud<Color>>();
  pcl::fromROSMsg(*msg, *rgbcloud_raw);

  auto end = std::chrono::high_resolution_clock::now();
  auto duration_conversion = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  start = end;

  // Transform pointcloud to base_link
  const auto& sensor_frame = msg->header.frame_id;
  auto [get_transform_s2b, sensor_to_baselink] = tf_.getTransform(sensor_frame, baselink_frame);
  if (!get_transform_s2b)
    return;
  auto rgbcloud_baselink = utils::pcl::transformPointcloud<Color>(rgbcloud_raw, sensor_to_baselink);

  end = std::chrono::high_resolution_clock::now();
  auto duration_transform = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  start = end;

  // Filter pointcloud by height (z-axis)
  auto rgbcloud_filtered =
      utils::pcl::filterPointcloudByField<Color>(rgbcloud_baselink, "z", height_min_thrsh_, height_max_thrsh_);

  end = std::chrono::high_resolution_clock::now();
  auto duration_filter = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  start = end;

  // Register rgb cloud to the map
  auto [get_transform_b2m, base_to_map] = tf_.getTransform(baselink_frame, map_frame);
  if (!get_transform_b2m)
  {
    std::cout << "\033[33m[HeightMapping]: Failed to register the cloud. Skip this frame... \033[0m\n";
    return;
  }
  auto rgbcloud_registered = utils::pcl::transformPointcloud<Color>(rgbcloud_filtered, base_to_map);

  end = std::chrono::high_resolution_clock::now();
  auto duration_registration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  start = end;

  // Downsampling by height map resolution-> each grid cell has only one point, with max height
  auto rgbcloud_registered_downsampled =
      height_map::pclProcessor::gridDownsampling<Color>(rgbcloud_registered, grid_resolution_);

  end = std::chrono::high_resolution_clock::now();
  auto duration_downsampling = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  start = end;

  // Estimate height of each grid cell
  height_estimator_->estimate(map_, *rgbcloud_registered_downsampled);

  end = std::chrono::high_resolution_clock::now();
  auto duration_estimation = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

  // For Debug:
  // - Publish preprocessed pointcloud
  // - Publish processing time
  if (debug_)
  {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_begin);

    // Publish preprocessed pointcloud
    sensor_msgs::PointCloud2 msg_pc;
    pcl::toROSMsg(*rgbcloud_registered_downsampled, msg_pc);
    pub_rgbd_downsampled_.publish(msg_pc);

    // Publish processing time
    jsk_rviz_plugins::OverlayText overlay_text;
    overlay_text.height = 300;
    overlay_text.width = 300;
    overlay_text.text = "RGBD Processing time: " + std::to_string(duration.count()) + " ms" +
                        "\nConversion: " + std::to_string(duration_conversion.count()) + " us" +
                        "\nTransform: " + std::to_string(duration_transform.count()) + " us" +
                        "\nFilter: " + std::to_string(duration_filter.count()) + " us" +
                        "\nRegistration: " + std::to_string(duration_registration.count()) + " us" +
                        "\nDownsampling: " + std::to_string(duration_downsampling.count()) + " us" +
                        "\nEstimation: " + std::to_string(duration_estimation.count()) + " us";
    pub_rgbd_processing_time_.publish(overlay_text);
  }
}

void HeightMapping::updatePosition(const ros::TimerEvent& event)
{
  // Get Transform from base_link to map (tf provided by 3d pose estimator. ex: VINS, LOAM, etc.)
  auto [get_transform_b2m, base_to_map] = tf_.getTransform(baselink_frame, map_frame);
  if (!get_transform_b2m)
    return;
  // Update map position
  map_.move(grid_map::Position(base_to_map.transform.translation.x, base_to_map.transform.translation.y));
}

void HeightMapping::publishHeightmap(const ros::TimerEvent& event)
{
  grid_map_msgs::GridMap msg_heightmap;
  grid_map::GridMapRosConverter::toMessage(map_, msg_heightmap);
  pub_heightmap_.publish(msg_heightmap);
}
