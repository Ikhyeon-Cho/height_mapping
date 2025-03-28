/*
 * gt_generation_node.cpp
 *
 *  Created on: Mar 25, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "data_generation/ros/gt_generation_node.h"
#include "data_generation/ros/config.h"

#include <pcl_conversions/pcl_conversions.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <filesystem>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace data_generation_ros {

GTGenerationNode::GTGenerationNode() : nh_("~") {

  // Configs from data_generation_node.yaml
  ros::NodeHandle cfg_node(nh_, "node");
  ros::NodeHandle cfg_frame_id(nh_, "frame_id");

  // ROS node
  GTGenerationNode::loadConfig(cfg_node);
  initializePubSubs();
  initializeTimers();
  initializeServices();

  // TF frame IDs
  frame_ids::loadFromConfig(cfg_frame_id);

  // Load reference map for data generation
  if (!loadGlobalMap(cfg_.reference_map_path)) {
    std::cerr << "\033[1;31m[data_generation_ros::GTGenerationNode]: "
              << "Failed to load reference map! Quitting...\033[0m\n";
    return;
  }
  // Print pre-loaded map info
  const auto &map_length_x = global_map_.getLength().x();
  const auto &map_length_y = global_map_.getLength().y();
  const auto &map_resolution = global_map_.getResolution();
  std::cout << "\033[1;32m[data_generation_ros::GTGenerationNode]: "
               "Reference map loaded successfully with size "
            << map_length_x << " x " << map_length_y << " m "
            << "and resolution " << map_resolution << " m\033[0m\n";

  std::cout << "\033[1;32m[data_generation_ros::GTGenerationNode]: "
               "Data generation node initialized. Waiting for scan inputs...\033[0m\n";
}

void GTGenerationNode::loadConfig(const ros::NodeHandle &nh) {

  cfg_.inputscan_topic = nh.param<std::string>("input_scan_topic", "/velodyne_points");
  cfg_.reference_map_path = nh.param<std::string>("reference_map_path", "");
  cfg_.data_collection_period = nh.param<double>("dataset/collection_period", 1.0);
  cfg_.scan_z_min = nh.param<double>("preprocessing/scan_z_min", -0.2);
  cfg_.scan_z_max = nh.param<double>("preprocessing/scan_z_max", 2.0);
  cfg_.scan_range_max = nh.param<double>("dataset/scan_range_max", 10.0);
  cfg_.map_size_x = nh.param<double>("dataset/map_size_x", 15.0);
  cfg_.map_size_y = nh.param<double>("dataset/map_size_y", 15.0);
  cfg_.dataset_output_path = nh.param<std::string>("dataset/output_path", "");
  cfg_.debug_mode = nh.param<bool>("debug_mode", false);
}

void GTGenerationNode::initializePubSubs() {

  const auto &input_topic = cfg_.inputscan_topic;
  sub_scan_ = nh_.subscribe(input_topic, 1, &GTGenerationNode::lidarScanCallback, this);
  pub_reference_map_ =
      nh_.advertise<grid_map_msgs::GridMap>("/data_generation/input/map_ref",
                                            1,
                                            true /* latched topic */);
  pub_scan_ = nh_.advertise<sensor_msgs::PointCloud2>("/data_generation/scan", 1);
  pub_map_ = nh_.advertise<grid_map_msgs::GridMap>("/data_generation/map", 1);
  if (cfg_.debug_mode) {
    debug_pub_map_ =
        nh_.advertise<grid_map_msgs::GridMap>("/data_generation/debug/map", 1);
  }
}

void GTGenerationNode::initializeTimers() {

  auto collection_duration = ros::Duration(cfg_.data_collection_period);
  auto publish_duration = ros::Duration(1.0); // 1Hz for visualization

  collection_timer_ = nh_.createTimer(collection_duration,
                                      &GTGenerationNode::collectionTimerCallback,
                                      this,
                                      false,
                                      false);
  publish_timer_ = nh_.createTimer(publish_duration,
                                   &GTGenerationNode::publishTimerCallback,
                                   this,
                                   false,
                                   true); // Start immediately
  if (cfg_.debug_mode) {
    debug_pub_timer_ = nh_.createTimer(10, &GTGenerationNode::debugTimerCallback, this);
  }
}

void GTGenerationNode::initializeServices() {

  srv_start_collection_ = nh_.advertiseService("/data_generation/start_collection",
                                               &GTGenerationNode::startCollection,
                                               this);
  srv_stop_collection_ = nh_.advertiseService("/data_generation/stop_collection",
                                              &GTGenerationNode::stopCollection,
                                              this);
}

bool GTGenerationNode::loadGlobalMap(const std::string &path) {

  // Check the input path validity
  if (path.empty()) {
    std::cerr << "\033[31m[data_generation_ros::GTGenerationNode]: "
                 "The path to the reference map is empty!\033[0m\n";
    ros::shutdown();
    return false;
  } else {
    std::cout << "\033[1;33m[data_generation_ros::GTGenerationNode]: "
              << "Reference map path --> " << path << "\033[0m\n";
  }

  std::cout << "\033[1;33m[data_generation_ros::GTGenerationNode]: "
               "Loading reference map...\033[0m\n";

  // Load reference height map msg from bag file
  grid_map_msgs::GridMap gridMapMsg;
  try {
    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Read);

    std::vector<std::string> topics{"/height_map_global"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    if (view.size() == 0) {
      bag.close();
      std::cerr << "\033[31m[data_generation_ros::GTGenerationNode]: "
                   "No height map found in the bag file!\033[0m\n";
      ros::shutdown();
      return false;
    }

    // Get the most recent message
    for (const rosbag::MessageInstance &m : view) {
      gridMapMsg = *(m.instantiate<grid_map_msgs::GridMap>());
    }
    bag.close();
    // Publish global map (latched topic)
    pub_reference_map_.publish(gridMapMsg);
  } catch (const std::exception &e) {
    ros::shutdown();
    return false;
  }

  // Load global map from the msg
  return grid_map::GridMapRosConverter::fromMessage(gridMapMsg, global_map_);
}

void GTGenerationNode::lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  /* Update height map, which is a learning target
  / - Extract ground/non-ground points from scan
  / - Update local heights using non-ground points. Required due to dynamic obstacles
  / - Apply raycasting correction to the ground points.
  / - Layers with interest:
  /   - elevation
  /   - scan/raycasting
  */

  frame_ids::sensor::LIDAR = msg->header.frame_id;
  if (!lidarscan_received_) {
    lidarscan_received_ = true;
    std::cout << "\033[1;32m[data_generation_ros::GTGenerationNode]: "
              << "Pointcloud Received! Use LiDAR scans for data generation... \033[0m\n";
  }

  // 1. Get transform matrix from tf tree
  geometry_msgs::TransformStamped lidar2base, base2map, lidar2map;
  if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, frame_ids::sensor::LIDAR, lidar2base) ||
      !tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map))
    return;
  lidar2map = tf_.multiplyTransforms(lidar2base, base2map);

  // 2. Convert msg to pcl pointcloud
  pcl::moveFromROSMsg(*msg, *input_scan_);

  // 3. Extract ground/non-ground points from the scan
  auto scan_R = PointCloudOps::applyTransform<VelodynePoint>(input_scan_, lidar2base);
  auto points_ground_R = extractGroundPoints(scan_R);
  auto points_nonground_R = extractNonGroundPoints(scan_R);
  if (!points_ground_R || !points_nonground_R)
    return;

  // 4. Update height map using non-ground points
  auto points_nonground_M =
      PointCloudOps::applyTransform<Laser>(points_nonground_R, base2map);
  grid_map::Position robot_position(base2map.transform.translation.x,
                                    base2map.transform.translation.y);
  auto map = getLatestLocalHeightMapFromScan(points_nonground_M, robot_position);
  if (!map)
    return;

  // 5. Raycasting correction using ground points
  Eigen::Vector3f sensorOrigin3D(lidar2map.transform.translation.x,
                                 lidar2map.transform.translation.y,
                                 lidar2map.transform.translation.z);
  auto points_ground_M = PointCloudOps::applyTransform<Laser>(points_ground_R, base2map);
  raycaster_.correctHeight(*map, *points_ground_M, sensorOrigin3D);

  // 6. Clamp height map
  clampHeightMap(map, 1.0);

  target_map_ = map;

  // 5. Publish local height map
  publishLocalMap(*map);
  publishScan(points_ground_R);
}

pcl::PointCloud<Laser>::Ptr GTGenerationNode::extractGroundPoints(
    const pcl::PointCloud<VelodynePoint>::Ptr &scan_base) {

  /* Extract ground points from the scan
  / - Filter scan by z-coordinate (z > cfg_.scan_z_min)
  / - Filter scan by ring info (ring >= 0 && ring <= 7)
  /   - Assume that velodyne is mounted horizontally
  */

  auto points_ground = boost::make_shared<pcl::PointCloud<Laser>>();
  points_ground->header = scan_base->header;

  for (const auto &point : scan_base->points) {

    // 1. Filter scan by z - remove noisy underground points
    if (point.z < cfg_.scan_z_min)
      continue;

    // 2. Filter scan by ring info
    if (point.ring >= 8 && point.ring <= 16)
      continue;

    Laser point_laser;
    point_laser.x = point.x;
    point_laser.y = point.y;
    point_laser.z = point.z;
    point_laser.intensity = point.intensity;
    points_ground->push_back(point_laser);
  }
  if (points_ground->empty())
    return nullptr;

  return points_ground;
}

pcl::PointCloud<Laser>::Ptr GTGenerationNode::extractNonGroundPoints(
    const pcl::PointCloud<VelodynePoint>::Ptr &scan_base) {

  /* Extract non-ground points from the scan
  / - Filter scan by z-coordinate
  / - Filter scan by ring info
  /   - Assume that velodyne is mounted horizontally
  */

  auto points_non_ground = boost::make_shared<pcl::PointCloud<Laser>>();
  points_non_ground->header = scan_base->header;
  for (const auto &point : scan_base->points) {

    // 1. Filter scan by z - remove overhanging points
    if (point.z > cfg_.scan_z_max)
      continue;

    // 2. Filter scan by ring info
    if (point.ring >= 0 && point.ring <= 7)
      continue;

    Laser point_laser;
    point_laser.x = point.x;
    point_laser.y = point.y;
    point_laser.z = point.z;
    point_laser.intensity = point.intensity;
    points_non_ground->push_back(point_laser);
  }

  if (points_non_ground->empty())
    return nullptr;

  return points_non_ground;
}

HeightMap::Ptr GTGenerationNode::getLatestLocalHeightMapFromScan(
    const pcl::PointCloud<Laser>::Ptr &scan,
    const grid_map::Position &robot_position) {

  auto map = getLocalHeightMap(robot_position);
  if (!map)
    return nullptr;

  updateHeightMap(map, scan);
  return map;
}

HeightMap::Ptr GTGenerationNode::getLocalHeightMap(const grid_map::Position &origin) {

  // Crop local map from global reference map
  grid_map::Length map_size(cfg_.map_size_x, cfg_.map_size_y);

  bool is_success{false};
  auto map =
      std::make_shared<HeightMap>(global_map_.getSubmap(origin, map_size, is_success));
  if (!is_success)
    return nullptr;
  return map;
}

void GTGenerationNode::updateHeightMap(const HeightMap::Ptr &map,
                                       const pcl::PointCloud<Laser>::Ptr &cloud) {
  grid_map::Index measuredIndex;
  grid_map::Position measuredPosition;

  for (const auto &point : cloud->points) {
    measuredPosition << point.x, point.y;

    if (!map->getIndex(measuredPosition, measuredIndex))
      continue;

    if (map->isEmptyAt(measuredIndex)) {
      map->at(layers::Height::ELEVATION, measuredIndex) = point.z;
      map->at(layers::Sensor::Lidar::INTENSITY, measuredIndex) = point.intensity;
      continue;
    }

    auto &height = map->at(layers::Height::ELEVATION, measuredIndex);
    height = std::max(height, point.z);
  }
}

void GTGenerationNode::clampHeightMap(const HeightMap::Ptr &map, const float max_height) {

  geometry_msgs::TransformStamped base2map;
  if (!tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map))
    return;
  auto robot_position_z = base2map.transform.translation.z;

  // Get reference to the elevation layer matrix
  grid_map::Matrix &elevation = map->get(layers::Height::ELEVATION);

  // Calculate maximum allowed height (robot_z + max_height)
  float max_allowed_height = robot_position_z + max_height;

  // Use Eigen array operations to clamp all values in one operation
  // This preserves NaN values and only clamps finite values that exceed
  // max_allowed_height
  elevation = (elevation.array() > max_allowed_height && elevation.array().isFinite())
                  .select(max_allowed_height, elevation);
}

bool GTGenerationNode::startCollection(std_srvs::Empty::Request &req,
                                       std_srvs::Empty::Response &res) {
  if (is_collecting_) {
    std::cout << "\033[1;33m[data_generation_ros::GTGenerationNode]: "
              << "Dataset collection already in progress\033[0m\n";
    return true;
  }

  // Create dataset directories
  // TODO: Check if the directory exists
  if (!cfg_.dataset_output_path.empty()) {
    std::filesystem::create_directories(cfg_.dataset_output_path + "/scans");
    std::filesystem::create_directories(cfg_.dataset_output_path + "/maps");
  } else {
    std::cerr << "\033[1;31m[data_generation_ros::GTGenerationNode]: "
              << "Dataset path is empty!\033[0m\n";
    return false;
  }

  // Start collection
  is_collecting_ = true;
  collection_timer_.start();

  std::cout << "\033[1;32m[data_generation_ros::GTGenerationNode]: "
            << "Dataset collection started. Saving to: " << cfg_.dataset_output_path
            << "\033[0m\n";
  return true;
}

void GTGenerationNode::collectionTimerCallback(const ros::TimerEvent &event) {
  // Write down the scan and map
}

bool GTGenerationNode::stopCollection(std_srvs::Empty::Request &req,
                                      std_srvs::Empty::Response &res) {
  if (!is_collecting_) {
    std::cout << "\033[1;33m[data_generation_ros::GTGenerationNode]: "
              << "Dataset collection not in progress\033[0m\n";
    return true;
  }

  // Stop collection
  is_collecting_ = false;
  collection_timer_.stop();

  std::cout << "\033[1;32m[data_generation_ros::GTGenerationNode]: "
            << "Dataset collection stopped. Collected " << sample_count_
            << " samples\033[0m\n";
  return true;
}

void GTGenerationNode::publishTimerCallback(const ros::TimerEvent &event) {
  //
}

void GTGenerationNode::debugTimerCallback(const ros::TimerEvent &event) {
  //
  // Lookup transform from tf tree
  geometry_msgs::TransformStamped base2map;
  if (!tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map))
    return;

  // Get local height map
  grid_map::Position robot_position(base2map.transform.translation.x,
                                    base2map.transform.translation.y);
  auto local_map = getLocalHeightMap(robot_position);
  if (!local_map) {
    std::cerr << "\033[1;31m[data_generation_ros::GTGenerationNode]: "
              << "Failed to get local height map!\033[0m\n";
    return;
  }

  // Publish local height map
  publishDebugMap(*local_map);
}

void GTGenerationNode::publishScan(const pcl::PointCloud<Laser>::Ptr &cloud) {
  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*cloud, msg_cloud);
  pub_scan_.publish(msg_cloud);
}

void GTGenerationNode::publishLocalMap(const grid_map::GridMap &map) {
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(map, msg);
  pub_map_.publish(msg);
}

void GTGenerationNode::publishDebugMap(const grid_map::GridMap &map) {
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(map, msg);
  debug_pub_map_.publish(msg);
}

} // namespace data_generation_ros

int main(int argc, char **argv) {

  ros::init(argc, argv, "data_generation_node");
  data_generation_ros::GTGenerationNode node;
  ros::spin();

  return 0;
}
