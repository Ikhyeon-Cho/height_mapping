/*
 * HeightMapLoader.h
 *
 *  Created on: Mar 21, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_LOADER_H
#define HEIGHT_MAP_LOADER_H

#include <ros/ros.h>

#include <height_mapping_core/map/HeightMap.h>
#include <height_mapping_msgs/HeightMapConverter.h>
#include <height_mapping_msgs/HeightMapMsgs.h>

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

class HeightMapLoader {
public:
  HeightMapLoader();

  // bool readFromPCD(const std::string& file_path);

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
  std::string image_path_{
      pnh_.param<std::string>("imagePath", "/home/isr/Downloads")};
  std::string map_name_{pnh_.param<std::string>("map", "map_name")};

  // Map parameters
  double grid_resolution_{pnh_.param<double>("gridResolution", 0.1)};

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
      &HeightMapLoader::visualizeHeightMap, this, false, false)};

  // Height Map
  grid_map::HeightMap map_{0, 0, grid_resolution_};
};

#endif // HEIGHT_MAP_LOADER_H