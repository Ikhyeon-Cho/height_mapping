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

  std::cout << "\033[1;32m[HeightMapping::GlobalMapping]: Waiting for Height "
               "map... \033[0m\n";
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

  if (!localMapReceived_) {
    localMapReceived_ = true;
    mapPublishTimer_.start();
    std::cout
        << "\033[1;32m[GlobalMapping::GlobalMapping]: Local map received! "
        << "Use local map for global mapping... \033[0m\n";
  }

  // TODO: Define point cloud type: local map or cloud?
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::moveFromROSMsg(*msg, cloud);
}

void GlobalMappingNode::publishMap(const ros::TimerEvent &) {

  sensor_msgs::PointCloud2 cloud_msg;
  std::vector<std::string> layers = {"elevation"};

  toPointCloud2(globalMapping_->getHeightMap(), layers,
                globalMapping_->getMeasuredIndices(), cloud_msg);

  pubGlobalMap_.publish(cloud_msg);

  // Visualize map region
  visualization_msgs::Marker msg_map_region;
  HeightMapMsgs::toMapRegion(globalMapping_->getHeightMap(), msg_map_region);
  pubMapRegion_.publish(msg_map_region);
}

void GlobalMappingNode::toPointCloud2(
    const grid_map::HeightMap &map, const std::vector<std::string> &layers,
    const std::unordered_set<grid_map::Index> &measured_indices,
    sensor_msgs::PointCloud2 &cloud) {

  // Setup cloud header
  cloud.header.frame_id = map.getFrameId();
  cloud.header.stamp.fromNSec(map.getTimestamp());
  cloud.is_dense = false;

  // Setup field names and cloud structure
  std::vector<std::string> fieldNames;
  fieldNames.reserve(layers.size());

  for (const auto &layer : layers) {
    if (layer == map.getHeightLayer()) {
      fieldNames.insert(fieldNames.end(), {"x", "y", "z"});
    } else if (layer == "color") {
      fieldNames.push_back("rgb");
    } else {
      fieldNames.push_back(layer);
    }
  }

  // Setup point field structure
  cloud.fields.clear();
  cloud.fields.reserve(fieldNames.size());
  int offset = 0;

  for (const auto &name : fieldNames) {
    sensor_msgs::PointField field;
    field.name = name;
    field.count = 1;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = offset;
    cloud.fields.push_back(field);
    offset += sizeof(float);
  }

  // Initialize cloud size
  const size_t num_points = measured_indices.size();
  cloud.height = 1;
  cloud.width = num_points;
  cloud.point_step = offset;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);

  // Setup point field iterators
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>>
      iterators;
  for (const auto &name : fieldNames) {
    iterators.emplace(name,
                      sensor_msgs::PointCloud2Iterator<float>(cloud, name));
  }

  // Fill point cloud data
  size_t valid_points = 0;
  for (const auto &index : measured_indices) {
    grid_map::Position3 position;
    if (!map.getPosition3(map.getHeightLayer(), index, position)) {
      continue;
    }

    // Update each field
    for (auto &[field_name, iterator] : iterators) {
      if (field_name == "x")
        *iterator = static_cast<float>(position.x());
      else if (field_name == "y")
        *iterator = static_cast<float>(position.y());
      else if (field_name == "z")
        *iterator = static_cast<float>(position.z());
      else if (field_name == "rgb")
        *iterator = static_cast<float>(map.at("color", index));
      else
        *iterator = static_cast<float>(map.at(field_name, index));
      ++iterator;
    }
    ++valid_points;
  }

  // Adjust final cloud size
  cloud.width = valid_points;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);
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
