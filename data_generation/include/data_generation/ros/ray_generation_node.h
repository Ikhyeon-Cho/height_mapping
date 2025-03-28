/*
 * ray_generation_node.h
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
#include "data_generation/types/elevation_point.h"

namespace data_generation_ros {

class RayGeneration {
public:
  struct Config {
    std::string input_scan_topic;
    std::string reference_map_path;
    double data_collection_period;
    // Dataset params
    std::string dataset_output_path;
    double scan_range;
    double height_map_size_x;
    double height_map_size_y;
  } cfg_;

  RayGeneration();
  ~RayGeneration() = default;

private:
  // Init functions
  void loadConfig(const ros::NodeHandle &nh);
  void initializeTimers();
  void initializePubSubs();
  void initializeServices();
  bool loadGlobalMap(const std::string &path);

  // Callback functions
  void lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg);
  pcl::PointCloud<Laser>::Ptr
  processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                   const geometry_msgs::TransformStamped &lidar2base,
                   const geometry_msgs::TransformStamped &base2map);
  bool startCollectionCallback(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res);
  bool stopCollectionCallback(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res);
  void collectionTimerCallback(const ros::TimerEvent &event);
  void publishTimerCallback(const ros::TimerEvent &event);

  // Core functions
  bool extractLocalMap(const geometry_msgs::TransformStamped &robot_pose,
                       grid_map::GridMap &local_map);

  // Helper functions
  void publishScan(const pcl::PointCloud<Laser>::Ptr &cloud);
  void publishLocalMap(const grid_map::GridMap &local_map);

  ros::NodeHandle nh_;

  // Subscribers & Publishers
  ros::Subscriber sub_lidarscan_;
  ros::Publisher pub_ref_map_;
  ros::Publisher pub_local_map_;
  ros::Publisher pub_scan_;

  // Services
  ros::ServiceServer srv_start_collection_;
  ros::ServiceServer srv_stop_collection_;

  // Timers
  ros::Timer collection_timer_;
  ros::Timer publish_timer_;

  // Core objects
  std::unique_ptr<height_mapping::HeightMapper> height_mapper_;
  TransformOps tf_;

  // State variables
  bool lidarscan_received_{false};
  bool is_collecting_{false};
  int sample_count_{0};
  pcl::PointCloud<Laser>::Ptr current_scan_{boost::make_shared<pcl::PointCloud<Laser>>()};
};

} // namespace data_generation_ros
