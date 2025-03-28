/*
 * gt_generation_node.h
 *
 *  Created on: Mar 25, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include "common/ros/common.h"
#include "data_generation/core/core.h"
#include "data_generation/types/velodyne_point.h"

namespace data_generation_ros {

using VelodynePoint = data_generation_types::VelodynePoint;

class GTGenerationNode {

public:
  struct Config {
    std::string inputscan_topic;
    std::string reference_map_path;
    // Dataset params: scan, map
    double scan_z_min;
    double scan_z_max;
    double scan_range_max;
    double map_size_x;
    double map_size_y;
    double data_collection_period;
    std::string dataset_output_path;

    bool debug_mode;
  } cfg_;

  GTGenerationNode();
  ~GTGenerationNode() = default;

private:
  // Init functions
  void loadConfig(const ros::NodeHandle &nh);
  void initializeTimers();
  void initializePubSubs();
  void initializeServices();
  bool loadGlobalMap(const std::string &path);

  // Callback functions
  // - Height map processing
  void lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg);
  pcl::PointCloud<Laser>::Ptr
  extractGroundPoints(const pcl::PointCloud<VelodynePoint>::Ptr &cloud);
  pcl::PointCloud<Laser>::Ptr
  extractNonGroundPoints(const pcl::PointCloud<VelodynePoint>::Ptr &cloud);
  HeightMap::Ptr getLatestLocalHeightMapFromScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                                                 const grid_map::Position &position);
  HeightMap::Ptr getLocalHeightMap(const grid_map::Position &position);
  void updateHeightMap(const HeightMap::Ptr &map,
                       const pcl::PointCloud<Laser>::Ptr &cloud);
  void clampHeightMap(const HeightMap::Ptr &map, const float max_height);
  // - Timer callbacks
  void publishTimerCallback(const ros::TimerEvent &event);
  void debugTimerCallback(const ros::TimerEvent &event);

  // Data collection
  bool startCollection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool stopCollection(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  void collectionTimerCallback(const ros::TimerEvent &event);

  // Helper functions
  bool extractLocalMap(const grid_map::Position &robot_position,
                       grid_map::GridMap &local_map);
  void publishScan(const pcl::PointCloud<Laser>::Ptr &cloud);
  void publishLocalMap(const grid_map::GridMap &local_map);
  void publishDebugMap(const grid_map::GridMap &debug_map);

  ros::NodeHandle nh_;

  // Subscribers & Publishers
  ros::Subscriber sub_scan_;
  ros::Publisher pub_reference_map_;
  ros::Publisher pub_map_;
  ros::Publisher pub_scan_;

  // Services
  ros::ServiceServer srv_start_collection_;
  ros::ServiceServer srv_stop_collection_;

  // Timers
  ros::Timer collection_timer_;
  ros::Timer publish_timer_;

  // Core objects
  std::unique_ptr<height_mapping::HeightMapper> height_mapper_;
  HeightMapRaycaster raycaster_;
  TransformOps tf_;

  // Data
  HeightMap global_map_;
  pcl::PointCloud<VelodynePoint>::Ptr input_scan_{
      boost::make_shared<pcl::PointCloud<VelodynePoint>>()};
  HeightMap::Ptr target_map_;

  // State variables
  bool lidarscan_received_{false};
  bool is_collecting_{false};
  int sample_count_{0};

  // Debug
  ros::Publisher debug_pub_map_;
  ros::Timer debug_pub_timer_;
};

} // namespace data_generation_ros
