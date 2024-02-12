/*
 * HeightMapVisualization.h
 *
 *  Created on: Feb 12, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_VISUALIZATION_H
#define HEIGHT_MAP_VISUALIZATION_H

#include <ros/ros.h>

// Msgs
#include <height_map_msgs/HeightMapMsgs.h>
#include <sensor_msgs/PointCloud2.h>

class HeightMapVisualization
{
public:
  HeightMapVisualization() = default;

  void gridMapCallback(const grid_map_msgs::GridMapConstPtr& msg);

  void featureMapCallback(const grid_map_msgs::GridMapConstPtr& msg);

  void visualizePointCloud(const grid_map::GridMap& map, ros::Publisher& pub_cloud);

  void visualizeMapBoundary(const grid_map::GridMap& map, ros::Publisher& pub_boundary);

private:
  ros::NodeHandle nh_{ "height_mapping" };

  std::string heightmap_topic_{ nh_.param<std::string>("heightMapTopic", "gridmap") };
  ros::Subscriber sub_heightmap_{ nh_.subscribe(heightmap_topic_, 10, &HeightMapVisualization::gridMapCallback, this) };

  std::string featuremap_topic_{ nh_.param<std::string>("featureMapTopic", "featuremap") };
  ros::Subscriber sub_featuremap_{ nh_.subscribe(featuremap_topic_, 10, &HeightMapVisualization::featureMapCallback,
                                                 this) };

  // Publishers
  ros::Publisher pub_elevationcloud_{ nh_.advertise<sensor_msgs::PointCloud2>("elevation_cloud", 1) };
  ros::Publisher pub_featurecloud_{ nh_.advertise<sensor_msgs::PointCloud2>("feature_cloud", 1) };
  ros::Publisher pub_boundary_{ nh_.advertise<visualization_msgs::Marker>("map_boundary", 1) };

private:
  grid_map::GridMap map_;
  grid_map::GridMap feature_map_;
};

#endif  // HEIGHT_MAP_VISUALIZATION_H