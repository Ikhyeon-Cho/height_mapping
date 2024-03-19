/*
 * HeightMapVisualization.cpp
 *
 *  Created on: Feb 12, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/HeightMapVisualization.h"
#include <chrono>

void HeightMapVisualization::gridMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  auto start = std::chrono::high_resolution_clock::now();

  grid_map::GridMapRosConverter::fromMessage(*msg, map_);

  visualizePointCloud(map_, pub_elevationcloud_);

  visualizeMapBoundary(map_, pub_boundary_);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

  std::cout << "Time taken for visualization: " << duration.count() << " milliseconds" << std::endl;
}

void HeightMapVisualization::featureMapCallback(const grid_map_msgs::GridMapConstPtr& msg)
{
  grid_map::GridMapRosConverter::fromMessage(*msg, feature_map_);

  visualizePointCloud(feature_map_, pub_featurecloud_);
}

void HeightMapVisualization::visualizePointCloud(const grid_map::GridMap& map, ros::Publisher& pub_cloud)
{
  // Publish pointcloud
  if (map.exists("elevation"))
  {
    sensor_msgs::PointCloud2 msg_cloud;
    grid_map::GridMapRosConverter::toPointCloud(map, "elevation", msg_cloud);
    pub_cloud.publish(msg_cloud);
  }
}

void HeightMapVisualization::visualizeMapBoundary(const grid_map::GridMap& map, ros::Publisher& pub_boundary)
{
  visualization_msgs::Marker msg_boundary;
  HeightMapMsgs::toMapBoundary(map, msg_boundary);
  pub_boundary.publish(msg_boundary);
}