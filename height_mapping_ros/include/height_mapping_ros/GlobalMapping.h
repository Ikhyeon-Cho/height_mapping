/*
 * GlobalMapping.h
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef GLOBAL_MAPPING_H
#define GLOBAL_MAPPING_H

#include <ros/ros.h>

#include <std_srvs/Empty.h>

#include <sensor_msgs/PointCloud2.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <height_map_core/height_map_core.h>
#include <height_map_msgs/HeightMapMsgs.h>
#include <height_map_msgs/HeightMapConverter.h>

#include <unordered_set>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <jsk_rviz_plugins/OverlayText.h>

// using PointT = pcl::PointXYZRGB;

template <typename PointT>
class GlobalMapping
{
public:
  GlobalMapping();

  /// @brief Update global map from local map
  /// @param msg local map pointcloud
  void updateFromLocalMap(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::fromROSMsg(*msg, *heightmap_cloud_);
    addMeasuredGridIndices(globalmap_, *heightmap_cloud_);

    height_estimator_->estimate(globalmap_, *heightmap_cloud_);
  }

  void visualize(const ros::TimerEvent& event);
  bool clearMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  bool saveLayerToImage(height_map_msgs::SaveLayerToImage::Request& request,
                        height_map_msgs::SaveLayerToImage::Response& response);
  bool saveMapToImage(const std::string& layer, const std::string& file_path);

  // bool saveAsPcd(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  struct IndexHash
  {
    std::size_t operator()(const grid_map::Index& index) const
    {
      return std::hash<int>()(index.x()) ^ std::hash<int>()(index.y());
    }
  };
  struct IndexEqual
  {
    bool operator()(const grid_map::Index& a, const grid_map::Index& b) const
    {
      return a.x() == b.x() && a.y() == b.y();
    }
  };

  // Helper functions
  void addMeasuredGridIndices(const grid_map::HeightMap& map, const pcl::PointCloud<PointT>& cloud);
  void toPointCloud2(const grid_map::HeightMap& map, const std::vector<std::string>& layers,
                     const std::unordered_set<grid_map::Index, IndexHash, IndexEqual>& measured_indices,
                     sensor_msgs::PointCloud2& cloud);

private:
  ros::NodeHandle nh_{ "height_mapping" };
  ros::NodeHandle nh_priv_{ "~" };

  // Frame Ids
  std::string baselink_frame_{ nh_priv_.param<std::string>("/frame_id/base_link", "base_link") };
  std::string map_frame_{ nh_priv_.param<std::string>("/frame_id/map", "map") };

  // Global Map parameters
  double grid_resolution_{ nh_priv_.param<double>("gridResolution", 0.1) };
  double map_length_x_{ nh_priv_.param<double>("mapLengthXGlobal", 400) };
  double map_length_y_{ nh_priv_.param<double>("mapLengthYGlobal", 400) };
  std::string height_estimator_type_{ nh_.param<std::string>("heightEstimatorType", "StatMean") };

  // Map saver parameters
  std::string home_dir_{ std::getenv("HOME") };
  std::string map_save_directory_{ nh_priv_.param<std::string>("mapSaveDir", home_dir_ + "/Downloads") };

  // Duration
  double map_visualization_rate_{ nh_priv_.param<double>("mapPublishRate", 10.0) };

  // ROS
  ros::Subscriber sub_pointcloud_{ nh_priv_.subscribe("/height_mapping/map/pointcloud", 10,
                                                      &GlobalMapping::updateFromLocalMap, this) };
  ros::Publisher pub_globalmap_{ nh_priv_.advertise<sensor_msgs::PointCloud2>("/height_mapping/globalmap/pointcloud",
                                                                              1) };
  ros::Publisher pub_map_region_{ nh_priv_.advertise<visualization_msgs::Marker>("/height_mapping/globalmap/region",
                                                                                 1) };

  ros::Timer map_visualization_timer_{ nh_priv_.createTimer(map_visualization_rate_, &GlobalMapping::visualize, this) };

  ros::ServiceServer clear_map_{ nh_priv_.advertiseService("/height_mapping/global/clear_map", &GlobalMapping::clearMap,
                                                           this) };
  ros::ServiceServer srv_image_saver_{ nh_priv_.advertiseService("/height_mapping/global/save_to_image",
                                                                 &GlobalMapping::saveLayerToImage, this) };

  // Debug Flag
  bool debug_{ nh_priv_.param<bool>("debugMode", false) };
  ros::Publisher pub_processing_time_;

private:
  // Global Map
  grid_map::HeightMap globalmap_{ map_length_x_, map_length_y_, grid_resolution_ };
  height_map::HeightEstimatorBase::Ptr height_estimator_;
  std::unordered_set<grid_map::Index, IndexHash, IndexEqual> measured_indices_;

  // Height map cloud
  typename pcl::PointCloud<PointT>::Ptr heightmap_cloud_{ boost::make_shared<pcl::PointCloud<PointT>>() };
};

#endif  // GLOBAL_MAPPING_H