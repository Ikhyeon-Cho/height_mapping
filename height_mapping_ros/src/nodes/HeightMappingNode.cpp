/*
 * HeightMappingNode.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/HeightMappingNode.h"

HeightMappingNode::HeightMappingNode() {

  getNodeParameters();

  getFrameIDs();

  setTimers();

  setupROSInterface();

  heightMapping_ =
      std::make_unique<HeightMapping>(getHeightMappingParameters());

  std::cout << "\033[1;33m[HeightMapping::HeightMapping]: "
               "Height mapping node initialized. Waiting for scan inputs... "
               "\033[0m\n";
}

void HeightMappingNode::getNodeParameters() {

  robotPoseUpdateRate_ =
      nhPriv_.param<double>("robotPoseUpdateRate", 20.0);          // [Hz]
  mapPublishRate_ = nhPriv_.param<double>("mapPublishRate", 10.0); // [Hz]
  debugMode_ = nhPriv_.param<bool>("debugMode", false);
  useLidarCallback_ = nhPriv_.param<bool>("useLidarCallback", true);
}

void HeightMappingNode::getFrameIDs() {

  baselinkFrame_ = nhFrameID_.param<std::string>("base_link", "base_link");
  mapFrame_ = nhFrameID_.param<std::string>("map", "map");
}

void HeightMappingNode::setTimers() {

  robotPoseUpdateTimer_ = nhPriv_.createTimer(
      ros::Duration(1.0 / robotPoseUpdateRate_),
      &HeightMappingNode::updateRobotPose, this, false, false);

  mapPublishTimer_ =
      nhPriv_.createTimer(ros::Duration(1.0 / mapPublishRate_),
                          &HeightMappingNode::publishMap, this, false, false);
}

void HeightMappingNode::setupROSInterface() {
  // Topics
  subLaserTopic_ =
      nhMap_.param<std::string>("lidarCloudTopic", "/points_laser");
  subRGBTopic_ = nhMap_.param<std::string>("rgbCloudTopic", "/points_rgb");

  // Subscribers
  if (useLidarCallback_) {
    subLaserCloud_ = nh_.subscribe(
        subLaserTopic_, 1, &HeightMappingNode::laserCloudCallback, this);
  }

  subRGBCloud_ = nh_.subscribe(subRGBTopic_, 1,
                               &HeightMappingNode::rgbCloudCallback, this);

  // Publishers
  pubHeightMap_ =
      nh_.advertise<grid_map_msgs::GridMap>("/height_mapping/map/gridmap", 1);

  pubLaserProcessed_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/height_mapping/mapping/lasercloud", 1);
  pubRGBProcessed_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/height_mapping/mapping/rgbdcloud", 1);

  // For debug
  if (debugMode_) {
    pubDebugLaserCloud_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/height_mapping/debug/lasercloud", 1);
    pubDebugRGBCloud_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/height_mapping/debug/rgbdcloud", 1);
  }
}

HeightMapping::Parameters HeightMappingNode::getHeightMappingParameters() {

  HeightMapping::Parameters params;
  params.mapFrame = mapFrame_;
  params.mapLengthX = nhMap_.param<double>("mapLengthX", 10.0);
  params.mapLengthY = nhMap_.param<double>("mapLengthY", 10.0);
  params.gridResolution = nhMap_.param<double>("gridResolution", 0.1);
  params.heightEstimatorType =
      nhMap_.param<std::string>("heightEstimatorType", "StatMean");
  params.minHeight = nhMap_.param<double>("minHeightThreshold", 0.0);
  params.maxHeight = nhMap_.param<double>("maxHeightThreshold", 1.0);

  return params;
}

void HeightMappingNode::laserCloudCallback(
    const sensor_msgs::PointCloud2Ptr &msg) {

  // First time receiving laser cloud -> start pose update timer
  if (!laserReceived_) {
    laserReceived_ = true;
    robotPoseUpdateTimer_.start();
    mapPublishTimer_.start();
    std::cout
        << "\033[1;33m[HeightMapping::HeightMapping]: Laser cloud Received! "
        << "Use LiDAR sensor for height mapping... \033[0m\n";
  }

  // Get Transform matrix
  const auto &laserFrame = msg->header.frame_id;
  auto [get1, laser2Baselink] = tf_.getTransform(laserFrame, baselinkFrame_);
  auto [get2, baselink2Map] = tf_.getTransform(baselinkFrame_, mapFrame_);
  if (!get1 || !get2)
    return;

  // Prepare pointcloud
  auto inputCloud = boost::make_shared<pcl::PointCloud<Laser>>();
  auto processedCloud = boost::make_shared<pcl::PointCloud<Laser>>();

  // Preprocess pointcloud
  pcl::moveFromROSMsg(*msg, *inputCloud);
  auto baselinkCloud =
      utils::pcl::transformPointcloud<Laser>(inputCloud, laser2Baselink);
  heightMapping_->fastHeightFilter<Laser>(baselinkCloud, processedCloud);
  if (removeRemoterPoints_) {
    processedCloud = utils::pcl::filterPointcloudByAngle<Laser>(processedCloud,
                                                                -135.0, 135.0);
  }

  // Mapping
  auto transform = utils::tf::toAffine3d(baselink2Map.transform);
  auto mappedLaserCloud =
      heightMapping_->mapping<Laser>(processedCloud, transform);

  // Publish pointcloud used for mapping
  sensor_msgs::PointCloud2 cloudMsg;
  pcl::toROSMsg(*mappedLaserCloud, cloudMsg);
  pubLaserProcessed_.publish(cloudMsg);

  // Debug: publish pointcloud that you want to see
  if (debugMode_) {
    sensor_msgs::PointCloud2 debugMsg;
    pcl::toROSMsg(*processedCloud, debugMsg);
    pubDebugLaserCloud_.publish(debugMsg);
  }
}

void HeightMappingNode::rgbCloudCallback(
    const sensor_msgs::PointCloud2Ptr &msg) {

  // First time receiving RGB-D cloud -> start pose update timer
  if (!rgbReceived_) {
    rgbReceived_ = true;
    robotPoseUpdateTimer_.start();
    mapPublishTimer_.start();
    std::cout
        << "\033[1;33m[HeightMapping::HeightMapping]: Colored cloud Received! "
        << "Use RGB-D sensors for height mapping... \033[0m\n";
  }

  // Get Transform matrix
  const auto &cameraFrame = msg->header.frame_id;
  auto [get1, camera2Baselink] = tf_.getTransform(cameraFrame, baselinkFrame_);
  auto [get2, baselink2Map] = tf_.getTransform(baselinkFrame_, mapFrame_);
  if (!get1 || !get2)
    return;

  // Prepare pointcloud
  auto inputCloud = boost::make_shared<pcl::PointCloud<Color>>();
  auto processedCloud = boost::make_shared<pcl::PointCloud<Color>>();

  // Preprocess pointcloud
  pcl::moveFromROSMsg(*msg, *inputCloud);
  auto baselinkCloud =
      utils::pcl::transformPointcloud<Color>(inputCloud, camera2Baselink);
  heightMapping_->fastHeightFilter<Color>(baselinkCloud, processedCloud);
  if (removeRemoterPoints_) {
    processedCloud = utils::pcl::filterPointcloudByAngle<Color>(processedCloud,
                                                                -135.0, 135.0);
  }

  // Mapping
  auto transform = utils::tf::toAffine3d(baselink2Map.transform);
  auto mappedRGBCloud =
      heightMapping_->mapping<Color>(processedCloud, transform);

  // Publish pointcloud used for mapping
  sensor_msgs::PointCloud2 cloudMsg;
  pcl::toROSMsg(*mappedRGBCloud, cloudMsg);
  pubRGBProcessed_.publish(cloudMsg);

  // Debug: publish pointcloud that you want to see
  if (debugMode_) {
    sensor_msgs::PointCloud2 debugMsg;
    pcl::toROSMsg(*processedCloud, debugMsg);
    pubDebugRGBCloud_.publish(debugMsg);
  }
}

void HeightMappingNode::updateRobotPose(const ros::TimerEvent &event) {

  auto [success, baselink2Map] = tf_.getTransform(baselinkFrame_, mapFrame_);
  if (!success)
    return;

  // Update map origin
  auto robotPosition = grid_map::Position(baselink2Map.transform.translation.x,
                                          baselink2Map.transform.translation.y);
  heightMapping_->updateMapOrigin(robotPosition);
}

void HeightMappingNode::publishMap(const ros::TimerEvent &event) {

  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(heightMapping_->getHeightMap(), msg);
  pubHeightMap_.publish(msg);
}
