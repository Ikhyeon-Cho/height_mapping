#include "height_mapping_ros/DataCollectionNode.h"
#include <fstream>
#include <tf2/utils.h>

DataCollectionNode::DataCollectionNode() {

  getNodeParameters();
  getFrameIDs();
  setupROSInterface();
  getDataCollectionParameters();

  loadGlobalMap();

  std::cout << "\033[1;32m[HeightMapping::DataCollection]: Global map loaded! "
               "Starting data collection node...\033[0m\n";
  std::cout
      << "\033[1;32m[HeightMapping::DataCollection]: Data collection node "
         "initialized. Waiting for LiDAR scan input... \033[0m\n";
}

void DataCollectionNode::getNodeParameters() {
  poseUpdateRate_ = nhPriv_.param<double>("poseUpdateRate", 10.0); // Hz
}

void DataCollectionNode::getFrameIDs() {
  mapFrame_ = nhFrameID_.param<std::string>("map", "map");
  baselinkFrame_ = nhFrameID_.param<std::string>("base_link", "base_link");
}

void DataCollectionNode::getDataCollectionParameters() {

  nhDataCollection_.getParam("globalMapPath", globalMapPath_);
  nhDataCollection_.getParam("dataCollectionPath", dataCollectionPath_);
}

void DataCollectionNode::setupROSInterface() {
  subCloudTopic_ = nhDataCollection_.param<std::string>("lidarCloudTopic",
                                                        "/velodyne/points");
  // Subscriber
  subLidarScan_ = nh_.subscribe(subCloudTopic_, 1,
                                &DataCollectionNode::laserCallback, this);

  // Publisher
  pubLocalDenseMap_ = nh_.advertise<grid_map_msgs::GridMap>(
      "/height_mapping/data_collection/map", 1);
  pubScan_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/height_mapping/data_collection/laserscan", 1);

  // Timer
  poseUpdateTimer_ =
      nhPriv_.createTimer(ros::Duration(1.0 / poseUpdateRate_),
                          &DataCollectionNode::updateRobotPose, this);
}

void DataCollectionNode::loadGlobalMap() {
  map_ = mapReader_.openBagFile(globalMapPath_,
                                "/height_mapping/globalmap/gridmap");
}

void DataCollectionNode::laserCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  // Get Transform matrix
  const auto &laserFrame = msg->header.frame_id;
  auto [get1, laser2Map] = tf_.getTransform(laserFrame, mapFrame_);
  auto [get2, baselink2Map] = tf_.getTransform(baselinkFrame_, mapFrame_);
  if (!get1 || !get2)
    return;

  // Transform pointcloud to map frame
  auto cloud = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *cloud);
  auto transformedCloud =
      utils::pcl::transformPointcloud<Laser>(cloud, laser2Map);

  // Get robot-centric heightmap
  grid_map::Length length(15.0, 15.0);
  grid_map::Position position(baselink2Map.transform.translation.x,
                              baselink2Map.transform.translation.y);
  bool isSuccess;
  auto heightmap = map_.getSubmap(position, length, isSuccess);
  if (!isSuccess) {
    std::cerr << "\033[1;31m[HeightMapping::DataCollection]: Failed to get "
                 "submap from global map! \033[0m\n";
    return;
  }

  // subtract offset: offset is heightmap origin
  auto offset2D = heightmap.getPosition();
  for (auto &point : transformedCloud->points) {
    point.x -= offset2D.x();
    point.y -= offset2D.y();
    point.z -= baselink2Map.transform.translation.z;
  }
  heightmap.setPosition(grid_map::Position(0.0, 0.0));
  // matrix operation: subtract height offset
  heightmap["elevation"].array() -= baselink2Map.transform.translation.z;

  transformedCloud->header.frame_id = baselinkFrame_;
  heightmap.setFrameId(baselinkFrame_);

  // Publish pointcloud
  sensor_msgs::PointCloud2 cloudMsg;
  pcl::toROSMsg(*transformedCloud, cloudMsg);
  pubScan_.publish(cloudMsg);

  // Publish heightmap
  grid_map_msgs::GridMap heightmapMsg;
  grid_map::GridMapRosConverter::toMessage(heightmap, heightmapMsg);
  pubLocalDenseMap_.publish(heightmapMsg);
}

void DataCollectionNode::updateRobotPose(const ros::TimerEvent &event) {
  // auto [success, transform] = tf_.getTransform(
  //     baselinkFrame_, mapFrame_, ros::Time(0), ros::Duration(0.1));
  // if (!success)
  //   return;

  // // Get Submap from current robot position
  // grid_map::Length length(10.0, 10.0);
  // grid_map::Position position(transform.transform.translation.x,
  //                             transform.transform.translation.y);
  // bool isSuccess;
  // auto submap = map_.getSubmap(position, length, isSuccess);
  // if (!isSuccess) {
  //   std::cerr << "\033[1;31m[HeightMapping::DataCollection]: Failed to get "
  //                "submap from global map! \033[0m\n";
  //   return;
  // }

  // // Publish submap
  // grid_map_msgs::GridMap localDenseMapMsg;
  // grid_map::GridMapRosConverter::toMessage(submap, localDenseMapMsg);
  // pubLocalDenseMap_.publish(localDenseMapMsg);

  // // Extract pose information
  // PoseData pose;
  // pose.x = transform.transform.translation.x;
  // pose.y = transform.transform.translation.y;
  // pose.yaw = tf2::getYaw(transform.transform.rotation);
  // pose.timestamp = transform.header.stamp;

  // poseHistory_.push_back(pose);
}
