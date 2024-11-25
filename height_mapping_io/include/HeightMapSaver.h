/*
 * HeightMapSaver.h
 *
 *  Created on: Mar 22, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_SAVER_H
#define HEIGHT_MAP_SAVER_H

#include <ros/ros.h>

#include <height_mapping_core/map/HeightMap.h>
#include <height_mapping_msgs/HeightMapMsgs.h>

#include <opencv2/opencv.hpp>

class HeightMapSaver {
public:
  HeightMapSaver();

  void visualizeHeightMap(const ros::WallTimerEvent &event);

private:
  ros::NodeHandle nh_{"height_mapping"};
  ros::NodeHandle pnh_{"~"};

  // Topics
  std::string heightmap_topic_{
      pnh_.param<std::string>("heightMapTopic", "elevation_grid")};

  // Frame Ids
  std::string map_frame_{nh_.param<std::string>("mapFrame", "map")};

  // Image parameters
  std::string image_dir_{
      pnh_.param<std::string>("imageDir", "/home/isr/Downloads")};
  std::string image_path_{
      image_dir_ + pnh_.param<std::string>("imageFile", "elevation.png")};

  // Node parameters
  double map_visualization_rate_{
      pnh_.param<double>("mapVisualizationRate", 1.0)};

  // ROS
  ros::Publisher pub_heightmap_{
      pnh_.advertise<grid_map_msgs::GridMap>(heightmap_topic_, 1)};
  ros::Publisher pub_heightcloud_{
      pnh_.advertise<sensor_msgs::PointCloud2>("elevation_cloud", 1)};
  ros::Publisher pub_map_region_{
      pnh_.advertise<visualization_msgs::Marker>("map_region", 1)};
  ros::WallTimer visualization_timer_{nh_.createWallTimer(
      ros::WallDuration(1.0 / map_visualization_rate_),
      &HeightMapSaver::visualizeHeightMap, this, false, false)};

  // Height Map
  grid_map::HeightMap map_{0, 0, 0.1};
};

#endif // HEIGHT_MAP_SAVER_H