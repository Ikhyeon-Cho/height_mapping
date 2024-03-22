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

  // Downsample pointcloud -> each grid should have only one point (max height)
  auto [success_downsampling, cloud_registered] = HeightMapping::getGridDownsampledCloud(*cloud_map, map_);

  // map_.update(*cloud_registered, "KalmanFilter+ConsistencyCheck");
  map_.update(*cloud_registered);
  // map_.update(*cloud_registered);

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
  // Height Map Visualization
  grid_map_msgs::GridMap msg_heightmap;
  grid_map::GridMapRosConverter::toMessage(map_, msg_heightmap);
  pub_heightmap_.publish(msg_heightmap);

  // Feature Map Visualization
  // grid_map_msgs::GridMap msg_featuremap;
  // grid_map::GridMapRosConverter::toMessage(descriptor_map_, msg_featuremap);
  // pub_featuremap_.publish(msg_featuremap);
}

std::pair<bool, pcl::PointCloud<pcl::PointXYZI>::Ptr>
HeightMapping::getGridDownsampledCloud(const pcl::PointCloud<pcl::PointXYZI>& pointcloud, grid_map::HeightMap& map)
{
  if (pointcloud.empty())
  {
    ROS_WARN("[ HeightMapping] Pointcloud is empty. Cannot downsample the point cloud");
    return { false, nullptr };
  }

  // The layer for grid-based downsampling of pointcloud
  if (!map.exists("downsampled_cloud") && !map.exists("downsampled_cloud_intensity"))
  {
    map.add("downsampled_cloud");
    map.add("downsampled_cloud_intensity");
  }
  else
  {
    map.clear("downsampled_cloud");
    map.clear("downsampled_cloud_intensity");
  }

  auto& data_downsampled_cloud = map["downsampled_cloud"];
  auto& data_downsampled_cloud_intensity = map["downsampled_cloud_intensity"];

  // Create a set to keep track of unique grid indices.
  std::vector<grid_map::Index> measured_index_list;
  grid_map::Index index;
  for (const auto& point : pointcloud)
  {
    // Check whether point is inside the map
    if (!map.getIndex(grid_map::Position(point.x, point.y), index))
      continue;

    // First grid height measuerment
    if (map.isEmptyAt("downsampled_cloud", index))
    {
      data_downsampled_cloud(index(0), index(1)) = point.z;
      data_downsampled_cloud_intensity(index(0), index(1)) = point.intensity;
      measured_index_list.push_back(index);
    }
    else if (point.z > data_downsampled_cloud(index(0), index(1)))
    {
      data_downsampled_cloud(index(0), index(1)) = point.z;
      data_downsampled_cloud_intensity(index(0), index(1)) = point.intensity;
    }
  }  // pointcloud loop ends

  // Get downsampled_cloud >> from multiple points per grid, get max height point
  auto downsampled_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  downsampled_cloud->header = pointcloud.header;
  downsampled_cloud->reserve(measured_index_list.size());

  // declare outside of for loop : for faster speed
  // Eigen constructors are pretty slow
  grid_map::Position3 grid_position3D;
  pcl::PointXYZI point;
  for (const auto& index : measured_index_list)
  {
    map.getPosition3("downsampled_cloud", index, grid_position3D);

    point.x = grid_position3D.x();
    point.y = grid_position3D.y();
    point.z = grid_position3D.z();
    point.intensity = data_downsampled_cloud_intensity(index(0), index(1));

    downsampled_cloud->push_back(point);
  }

  return { true, downsampled_cloud };
}