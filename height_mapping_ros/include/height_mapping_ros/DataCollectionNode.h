#pragma once

#include <height_mapping_io/height_mapping_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "height_mapping_ros/CloudTypes.h"
#include "utils/TransformHandler.h"

class DataCollectionNode {
public:
  DataCollectionNode();
  ~DataCollectionNode() = default;

private:
  void getNodeParameters();
  void getFrameIDs();
  void getDataCollectionParameters();
  void setupROSInterface();

  void loadGlobalMap();
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void updateRobotPose(const ros::TimerEvent &event);

  // ROS members
  ros::NodeHandle nh_;
  ros::NodeHandle nhPriv_{"~"};
  ros::NodeHandle nhFrameID_{nh_, "frame_id"};
  ros::NodeHandle nhDataCollection_{nh_, "data_collection"};

  // Frame IDs
  std::string mapFrame_;
  std::string baselinkFrame_;

  // Subscribers
  ros::Subscriber subLidarScan_;

  // Publishers
  ros::Publisher pubLocalDenseMap_;

  // Timer
  ros::Timer poseUpdateTimer_;

  // Main variables
  utils::TransformHandler tf_;
  grid_map::GridMap map_;
  HeightMapReader mapReader_;

  // Parameters
  std::string subCloudTopic_;
  double poseUpdateRate_;
  std::string globalMapPath_;
  std::string dataCollectionPath_;

  // State tracking
  bool cloudReceived_{false};
  struct PoseData {
    double x;
    double y;
    double yaw;
    ros::Time timestamp;
  };
  std::vector<PoseData> poseHistory_;
};
