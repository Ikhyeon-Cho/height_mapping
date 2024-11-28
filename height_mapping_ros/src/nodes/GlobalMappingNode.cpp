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
  mapPublishRate_ = nhPriv_.param<double>("mapPublishRate", 10.0);
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
  subLaserCloud_ = nh_.subscribe("/height_mapping/mapping/lasercloud", 1,
                                 &GlobalMappingNode::laserCloudCallback, this);
  subRGBCloud_ = nh_.subscribe("/height_mapping/mapping/rgbdcloud", 1,
                               &GlobalMappingNode::rgbCloudCallback, this);

  // Publishers
  pubGlobalMap_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/height_mapping/globalmap/pointcloud", 1);
  pubMapRegion_ = nh_.advertise<visualization_msgs::Marker>(
      "/height_mapping/globalmap/region", 1);

  // Services
  srvSaveMapToBag_ =
      nh_.advertiseService("/height_mapping/global_mapping/save_map",
                           &GlobalMappingNode::saveMapCallback, this);
  srvClearMap_ =
      nh_.advertiseService("/height_mapping/global_mapping/clear_map",
                           &GlobalMappingNode::clearMapCallback, this);
}

GlobalMapping::Parameters GlobalMappingNode::getGlobalMappingParameters() {

  GlobalMapping::Parameters params;
  params.mapFrame = mapFrame_;
  params.heightEstimatorType =
      nhMap_.param<std::string>("heightEstimatorType", "StatMean");
  params.gridResolution = nhGlobalMap_.param<double>("gridResolution", 0.1);
  params.mapLengthX = nhGlobalMap_.param<double>("mapLengthX", 400.0);
  params.mapLengthY = nhGlobalMap_.param<double>("mapLengthY", 400.0);

  bagSavePath_ = nhGlobalMap_.param<std::string>(
      "bagSavePath",
      std::string("/home/") + std::getenv("USER") + "/Downloads");
  return params;
}

void GlobalMappingNode::laserCloudCallback(
    const sensor_msgs::PointCloud2Ptr &msg) {

  if (!laserReceived_) {
    laserReceived_ = true;
    mapPublishTimer_.start();
    std::cout
        << "\033[1;32m[HeightMapping::GlobalMapping]: Laser cloud received! "
        << "Start global mapping... \033[0m\n";
  }
  pcl::PointCloud<Laser> cloud;
  pcl::moveFromROSMsg(*msg, cloud);
  globalMapping_->mapping(cloud);

  auto [get, laser2Map] = tf_.getTransform("velodyne", mapFrame_); // TODO
  if (!get)
    return;
  Eigen::Vector3f laserPosition3D(laser2Map.transform.translation.x,
                                  laser2Map.transform.translation.y,
                                  laser2Map.transform.translation.z);
  globalMapping_->raycasting(laserPosition3D, cloud);
}

void GlobalMappingNode::rgbCloudCallback(
    const sensor_msgs::PointCloud2Ptr &msg) {

  if (!rgbReceived_) {
    rgbReceived_ = true;
    mapPublishTimer_.start();
    std::cout
        << "\033[1;32m[HeightMapping::GlobalMapping]: Colored cloud received! "
        << "Start global mapping... \033[0m\n";
  }
  pcl::PointCloud<Color> cloud;
  pcl::moveFromROSMsg(*msg, cloud);
  globalMapping_->mapping(cloud);
}

void GlobalMappingNode::publishMap(const ros::TimerEvent &) {

  // Visualize global map
  sensor_msgs::PointCloud2 cloud_msg;
  std::vector<std::string> layers = globalMapping_->getHeightMap().getLayers();
  toPointCloud2(globalMapping_->getHeightMap(), layers,
                globalMapping_->getMeasuredGridIndices(), cloud_msg);
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
    if (layer == grid_map::HeightMap::CoreLayers::ELEVATION) {
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
    if (!map.getPosition3(grid_map::HeightMap::CoreLayers::ELEVATION, index,
                          position)) {
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
  globalMapping_->clearMap();
  return true;
}

bool GlobalMappingNode::saveMapCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res) {
  try {
    // Folder check and creation
    std::filesystem::path save_path(bagSavePath_);
    std::filesystem::path save_dir =
        save_path.has_extension() ? save_path.parent_path() : save_path;

    if (!std::filesystem::exists(save_dir)) {
      std::filesystem::create_directories(save_dir);
    }

    // Save GridMap to bag
    mapWriter_.writeToBag(globalMapping_->getHeightMap(), bagSavePath_,
                          "/height_mapping/globalmap/gridmap");

    // Save GridMap to PCD
    mapWriter_.writeToPCD(globalMapping_->getHeightMap(),
                          bagSavePath_.substr(0, bagSavePath_.rfind('.')) +
                              ".pcd");

    std::cout << "\033[1;33m[HeightMapping::GlobalMapping]: Successfully saved "
              << "map to " << bagSavePath_ << "\033[0m\n";

  } catch (const std::exception &e) {
    std::cout
        << "\033[1;31m[HeightMapping::GlobalMapping]: Failed to save map: "
        << std::string(e.what()) << "\033[0m\n";
  }

  return true;
}
