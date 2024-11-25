/*
 * GlobalMappingNode.h
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "height_mapping_ros/GlobalMapping.h"

class GlobalMappingNode {
public:
  GlobalMappingNode();
  ~GlobalMappingNode() = default;

private:
  void getNodeParameters();
  void getFrameIDs();
  void setTimers();
  void setupROSInterface();
  GlobalMapping::Parameters getGlobalMappingParameters();

  void localMapCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void publishMap(const ros::TimerEvent &event);
  bool clearMapCallback(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res);
  bool saveMapCallback(height_map_msgs::SaveLayerToImage::Request &req,
                       height_map_msgs::SaveLayerToImage::Response &res);

  void
  toPointCloud2(const grid_map::HeightMap &map,
                const std::vector<std::string> &layers,
                const std::unordered_set<grid_map::Index> &measured_indices,
                sensor_msgs::PointCloud2 &cloud);

  // ROS members
  ros::NodeHandle nh_;                      // "/height_mapping/"
  ros::NodeHandle nhPriv_{"~"};             // "/height_mapping/global_mapping"
  ros::NodeHandle nhMap_{nh_, "globalmap"}; // "/height_mapping/globalmap/"
  ros::NodeHandle nhFrameID_{nh_, "frame_id"}; // "/height_mapping/frame_id/"

  // Subscribers
  ros::Subscriber subLocalMap_;

  // Publishers
  ros::Publisher pubGlobalMap_;
  ros::Publisher pubMapRegion_;

  // Services
  ros::ServiceServer srvClearMap_;
  ros::ServiceServer srvSaveMap_;

  // Timers
  ros::Timer mapPublishTimer_;

  // Frame IDs
  std::string mapFrame_;
  std::string baselinkFrame_;

  // Parameters
  bool debugMode_{false};
  double mapPublishRate_{10.0};

  // Core mapping object
  std::unique_ptr<GlobalMapping> globalMapping_;

  // State variables
  bool localMapReceived_{false};
};
