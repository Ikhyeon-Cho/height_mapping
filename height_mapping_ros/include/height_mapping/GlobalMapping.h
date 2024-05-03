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

using PointT = pcl::PointXYZRGB;

class GlobalMapping
{
public:
  GlobalMapping();

  void updateFromLocalMap(const sensor_msgs::PointCloud2ConstPtr& msg);

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

private:
  // Helper functions
  void addMeasuredGridIndices(const grid_map::HeightMap& map, const pcl::PointCloud<PointT>& cloud);
  void toPointCloud2(const grid_map::HeightMap& map, const std::vector<std::string>& layers,
                     const std::unordered_set<grid_map::Index, IndexHash, IndexEqual>& measured_indices,
                     sensor_msgs::PointCloud2& cloud);
  ros::NodeHandle nh_{ "height_mapping" };
  ros::NodeHandle pnh_{ "~" };

  // Topics
  std::string globalmap_topic_{ pnh_.param<std::string>("globalMapTopic", "elevation_global") };
  std::string map_region_topic_{ pnh_.param<std::string>("mapRegionTopic", "map_region_global") };
  // Debug Flag
  bool debug_{ nh_.param<bool>("publishDebugTopics", false) };

  // Frame Ids
  std::string baselink_frame_{ nh_.param<std::string>("baselinkFrame", "base_link") };
  std::string map_frame_{ nh_.param<std::string>("mapFrame", "map") };

  // Global Map parameters
  double grid_resolution_{ pnh_.param<double>("gridResolution", 0.1) };
  double map_length_x_{ pnh_.param<double>("mapLengthXGlobal", 400) };
  double map_length_y_{ pnh_.param<double>("mapLengthYGlobal", 400) };

  // Height Estimator Parameters
  std::string height_estimator_type_{ pnh_.param<std::string>("heightEstimatorType", "StatMean") };

  // Map saver parameters
  std::string file_save_path_{ pnh_.param<std::string>("mapSaveDir", "/home/isr/Downloads") };

  // Duration
  double map_visualization_rate_{ pnh_.param<double>("globalMapVisualizationRate", 10.0) };

  // ROS
  ros::Subscriber sub_pointcloud_{ nh_.subscribe("pointcloud", 10, &GlobalMapping::updateFromLocalMap, this) };
  ros::Publisher pub_globalmap_{ nh_.advertise<sensor_msgs::PointCloud2>("global/pointcloud", 1) };
  ros::Publisher pub_map_region_{ nh_.advertise<visualization_msgs::Marker>("global/map_region", 1) };

  ros::Timer map_visualization_timer_{ nh_.createTimer(map_visualization_rate_, &GlobalMapping::visualize, this) };

  ros::ServiceServer clear_map_{ nh_.advertiseService("clear_map", &GlobalMapping::clearMap, this) };
  ros::ServiceServer srv_image_saver_{ nh_.advertiseService("save_to_image", &GlobalMapping::saveLayerToImage, this) };

private:
  // Global Map
  grid_map::HeightMap globalmap_{ map_length_x_, map_length_y_, grid_resolution_ };
  height_map::HeightEstimatorBase::Ptr height_estimator_;

  // Height map cloud
  pcl::PointCloud<PointT>::Ptr heightmap_cloud_;
  // std::vector<grid_map::Index> measured_indices_;
  std::unordered_set<grid_map::Index, IndexHash, IndexEqual> measured_indices_;
};

#endif  // GLOBAL_MAPPING_H