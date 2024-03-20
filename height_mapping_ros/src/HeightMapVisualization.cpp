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
  grid_map::GridMapRosConverter::fromMessage(*msg, map_);

  visualizePointCloud(map_, pub_elevationcloud_);  // visualize elevation map

  visualizeMapRegion();
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

void HeightMapVisualization::visualizeMapRegion()
{
  visualization_msgs::Marker msg_map_region;
  HeightMapMsgs::toMapRegion(map_, msg_map_region);
  pub_map_region_.publish(msg_map_region);
}