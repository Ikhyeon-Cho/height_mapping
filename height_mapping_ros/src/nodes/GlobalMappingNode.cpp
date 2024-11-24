/*
 * GlobalMappingNode.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/GlobalMappingNode.h"

GlobalMappingNode::GlobalMappingNode() {

  getNodeParameters();

  getFrameIDs();

  setTimers();

  setupROSInterface();

  globalMapping_ =
      std::make_unique<GlobalMapping>(getGlobalMappingParameters());
}

void GlobalMappingNode::getNodeParameters() {
  mapPublishRate_ = nhPriv_.param<double>("mapPublishRate", 1.0);
}

void GlobalMappingNode::getFrameIDs() {
  baselinkFrame_ = nhFrameID_.param<std::string>("base_link", "base_link");
  mapFrame_ = nhFrameID_.param<std::string>("map", "map");
}

void GlobalMappingNode::setTimers() {
  mapPublishTimer_ =
      nhPriv_.createTimer(ros::Duration(1.0 / mapPublishRate_),
                          &GlobalMappingNode::publishMap, this, false, false);
}

void GlobalMappingNode::setupROSInterface() {
  // Subscribers
  subLocalMap_ = nh_.subscribe("/height_mapping/map/pointcloud", 1,
                               &GlobalMappingNode::localMapCallback, this);

  // Publishers
  pubGlobalMap_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/height_mapping/globalmap/pointcloud", 1);
  pubMapRegion_ = nh_.advertise<visualization_msgs::Marker>(
      "/height_mapping/globalmap/region", 1);

  // Services
  if (debugMode_) {
    srvClearMap_ =
        nh_.advertiseService("/height_mapping/global/clear_map",
                             &GlobalMappingNode::clearMapCallback, this);
    srvSaveMap_ =
        nh_.advertiseService("/height_mapping/global/save_to_image",
                             &GlobalMappingNode::saveMapCallback, this);
  }
}

GlobalMapping::Parameters GlobalMappingNode::getGlobalMappingParameters() {

  GlobalMapping::Parameters params;
  params.mapFrame = mapFrame_;
  params.heightEstimatorType =
      nhMap_.param<std::string>("heightEstimatorType", "StatMean");
  params.gridResolution = nhMap_.param<double>("gridResolution", 0.1);
  params.mapLengthX = nhMap_.param<double>("mapLengthX", 400.0);
  params.mapLengthY = nhMap_.param<double>("mapLengthY", 400.0);
  params.mapSaveDir = nhMap_.param<std::string>("mapSaveDir", "");
  return params;
}

void GlobalMappingNode::localMapCallback(
    const sensor_msgs::PointCloud2Ptr &msg) {

  globalMapping_->updateFromLocalMap(msg);
}

void GlobalMappingNode::publishMap(const ros::TimerEvent &) {
  sensor_msgs::PointCloud2 cloud_msg;

  // Define which layers to include in the point cloud
  std::vector<std::string> layers = {
      "elevation"}; // or whatever layers you want to publish

  globalMapping_->toPointCloud2(globalMapping_->getHeightMap(),
                                layers, // Add this parameter
                                globalMapping_->getMeasuredIndices(),
                                cloud_msg);

  pubGlobalMap_.publish(cloud_msg);

  // Visualize map region
  visualization_msgs::Marker msg_map_region;
  HeightMapMsgs::toMapRegion(globalMapping_->getHeightMap(), msg_map_region);
  pubMapRegion_.publish(msg_map_region);
}

bool GlobalMappingNode::clearMapCallback(std_srvs::Empty::Request &req,
                                         std_srvs::Empty::Response &res) {
  return globalMapping_->clearMap();
}

bool GlobalMappingNode::saveMapCallback(
    height_map_msgs::SaveLayerToImage::Request &req,
    height_map_msgs::SaveLayerToImage::Response &res) {
  return globalMapping_->saveLayerToImage(req, res);
}
