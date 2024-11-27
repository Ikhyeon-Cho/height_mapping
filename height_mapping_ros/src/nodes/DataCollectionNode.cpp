#include "height_mapping_ros/DataCollectionNode.h"
#include <fstream>
#include <tf2/utils.h>

DataCollectionNode::DataCollectionNode() {

  getNodeParameters();
  getFrameIDs();
  setupROSInterface();
  getDataCollectionParameters();

  loadGlobalMap();

  std::cout
      << "\033[1;32m[HeightMapping::DataCollection]: Data collection node "
         "initialized. Waiting for LiDAR scan input... \033[0m\n";
}

void DataCollectionNode::getNodeParameters() {

  poseUpdateRate_ = nhPriv_.param<double>("poseUpdateRate", 10.0); // Hz
  globalMapPath_ = nhDataCollection_.param<std::string>(
      "globalMapPath", "/home/ikhyeon/ros/dev_ws/src/height_mapping/"
                       "height_mapping_ros/maps/globalmap.bag");
  dataCollectionPath_ = nhDataCollection_.param<std::string>(
      "dataCollectionPath",
      "/home/ikhyeon/ros/dev_ws/src/height_mapping/height_mapping_ros/maps/");
}

void DataCollectionNode::getFrameIDs() {
  mapFrame_ = nhFrameID_.param<std::string>("map", "map");
  baselinkFrame_ = nhFrameID_.param<std::string>("base_link", "base_link");
}

void DataCollectionNode::getDataCollectionParameters() {

  globalMapPath_ = nhDataCollection_.param<std::string>(
      "globalMapPath", "/home/ikhyeon/ros/dev_ws/src/height_mapping/"
                       "height_mapping_ros/maps/globalmap.bag");
  dataCollectionPath_ = nhDataCollection_.param<std::string>(
      "dataCollectionPath",
      "/home/ikhyeon/ros/dev_ws/src/height_mapping/height_mapping_ros/maps/");
}

void DataCollectionNode::setupROSInterface() {
  subCloudTopic_ = nhDataCollection_.param<std::string>("lidarCloudTopic",
                                                        "/velodyne/points");

  // Subscriber
  subLidarScan_ = nh_.subscribe(subCloudTopic_, 1,
                                &DataCollectionNode::pointCloudCallback, this);

  // Publisher
  pubLocalDenseMap_ = nh_.advertise<grid_map_msgs::GridMap>(
      "/height_mapping/data_collection/dense_map", 1);

  // Timer
  poseUpdateTimer_ =
      nhPriv_.createTimer(ros::Duration(1.0 / poseUpdateRate_),
                          &DataCollectionNode::updateRobotPose, this, false,
                          false); // Needs manual start
}

void DataCollectionNode::loadGlobalMap() {
  // Load global map
  map_ = mapReader_.openBagFile(globalMapPath_,
                                "/height_mapping/globalmap/gridmap");
  // Start pose update timer
  poseUpdateTimer_.start();

  std::cout << "\033[1;32m[HeightMapping::DataCollection]: Global map loaded! "
               "Starting local map generation...\033[0m\n";
}

void DataCollectionNode::pointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg) {

  // Get current robot pose
  auto [success, transform] = tf_.getTransform(
      baselinkFrame_, mapFrame_, msg->header.stamp, ros::Duration(0.1));
  if (!success)
    return;
}

void DataCollectionNode::updateRobotPose(const ros::TimerEvent &event) {
  auto [success, transform] = tf_.getTransform(
      baselinkFrame_, mapFrame_, ros::Time(0), ros::Duration(0.1));
  if (!success)
    return;

  // Get Submap from current robot position
  grid_map::Length length(10.0, 10.0);
  grid_map::Position position(transform.transform.translation.x,
                              transform.transform.translation.y);
  bool isSuccess;
  auto submap = map_.getSubmap(position, length, isSuccess);
  if (!isSuccess) {
    std::cerr << "\033[1;31m[HeightMapping::DataCollection]: Failed to get "
                 "submap from global map! \033[0m\n";
    return;
  }

  // Publish submap
  grid_map_msgs::GridMap localDenseMapMsg;
  grid_map::GridMapRosConverter::toMessage(submap, localDenseMapMsg);
  pubLocalDenseMap_.publish(localDenseMapMsg);

  // Extract pose information
  PoseData pose;
  pose.x = transform.transform.translation.x;
  pose.y = transform.transform.translation.y;
  pose.yaw = tf2::getYaw(transform.transform.rotation);
  pose.timestamp = transform.header.stamp;

  poseHistory_.push_back(pose);
}
