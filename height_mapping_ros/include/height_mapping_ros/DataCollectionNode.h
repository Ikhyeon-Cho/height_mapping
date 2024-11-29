#pragma once

#include <height_mapping_core/height_mapping_core.h>
#include <height_mapping_io/height_mapping_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include "height_mapping_ros/CloudTypes.h"
#include "height_mapping_ros/HeightMapping.h"
#include "utils/TransformHandler.h"
#include "utils/pointcloud.h"

class DataCollectionNode {
public:
  DataCollectionNode();
  ~DataCollectionNode() = default;

private:
  void getNodeParameters();
  void getFrameIDs();
  void getDataCollectionParameters();
  void setupROSInterface();
  HeightMapping::Parameters getHeightMappingParameters();

  void laserCallback(const sensor_msgs::PointCloud2Ptr &msg);
  bool startMapSaverCallback(std_srvs::Empty::Request &req,
                             std_srvs::Empty::Response &res);

  std::vector<geometry_msgs::TransformStamped>
  readPosesFromFile(const std::string &pose_file);

  // ROS members
  ros::NodeHandle nh_;
  ros::NodeHandle nhPriv_{"~"};
  ros::NodeHandle nhFrameID_{nh_, "frame_id"};
  ros::NodeHandle nhDataCollection_{nh_, "data_collection"};
  ros::NodeHandle nhMap_{nh_, "map"};
  // Frame IDs
  std::string mapFrame_;
  std::string baselinkFrame_;

  // Subscribers
  ros::Subscriber subLidarScan_;

  ros::Publisher pubHeightMap_;
  ros::Publisher pubScan_;

  // Service server
  ros::ServiceServer startCollectionServer_;

  // TF
  utils::TransformHandler tf_;

  // Height map
  std::unique_ptr<HeightMapping> heightMapping_;
  HeightMapReader mapReader_;

  // Parameters
  std::string subLidarTopic_;
  std::string dataCollectionPath_{"/home/ikhyeon/ros/dev_ws/src/height_mapping/"
                                  "height_mapping_ros/maps/"};
  unsigned int scanCount_{0};
  bool removeRemoterPoints_{true};
};
