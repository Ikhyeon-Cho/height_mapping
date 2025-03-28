/*
 * data_generation_node.cpp
 *
 *  Created on: Mar 25, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "data_generation/ros/data_generation_node.h"
#include "data_generation/ros/config.h"

#include <pcl_conversions/pcl_conversions.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <filesystem>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace data_generation_ros {

DataGenerationNode::DataGenerationNode() : nh_("~") {

  // Configs from data_generation_node.yaml
  ros::NodeHandle cfg_node(nh_, "node");
  ros::NodeHandle cfg_frame_id(nh_, "frame_id");

  // ROS node
  DataGenerationNode::loadConfig(cfg_node);
  initializePubSubs();
  initializeTimers();
  initializeServices();

  frame_id_ = FrameID::loadFromConfig(cfg_frame_id);

  // Load reference map for data generation
  if (!loadGlobalMap(cfg_.reference_map_path)) {
    std::cerr << "\033[1;31m[data_generation_ros::DataGenerationNode]: "
              << "Failed to load reference map! Quitting...\033[0m\n";
    return;
  }
  std::cout << "\033[1;32m[data_generation_ros::DataGenerationNode]: "
               "Data generation node initialized. Waiting for scan inputs...\033[0m\n";
}

void DataGenerationNode::loadConfig(const ros::NodeHandle &nh) {

  cfg_.input_scan_topic = nh.param<std::string>("input_scan_topic", "/velodyne_points");
  cfg_.reference_map_path = nh.param<std::string>("reference_map_path", "");
  cfg_.data_collection_period = nh.param<double>("data_collection_period", 1.0);

  cfg_.dataset_output_path = nh.param<std::string>("dataset/output_path", "");
  cfg_.scan_range = nh.param<double>("dataset/scan_range", 7.5);
  cfg_.height_map_size_x = nh.param<double>("dataset/height_map_size_x", 15.0);
  cfg_.height_map_size_y = nh.param<double>("dataset/height_map_size_y", 15.0);
}

void DataGenerationNode::initializePubSubs() {

  sub_lidarscan_ = nh_.subscribe(cfg_.input_scan_topic,
                                 1,
                                 &DataGenerationNode::lidarScanCallback,
                                 this);
  pub_ref_map_ =
      nh_.advertise<grid_map_msgs::GridMap>("/height_mapping/data_generation/ref_map",
                                            1,
                                            true /* latched topic */);
  pub_scan_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/data_generation/scan", 1);
  pub_local_map_ =
      nh_.advertise<grid_map_msgs::GridMap>("/height_mapping/data_generation/map", 1);
}

void DataGenerationNode::initializeTimers() {

  auto collection_duration = ros::Duration(cfg_.data_collection_period);
  auto publish_duration = ros::Duration(1.0); // 1Hz for visualization

  collection_timer_ = nh_.createTimer(collection_duration,
                                      &DataGenerationNode::collectionTimerCallback,
                                      this,
                                      false,
                                      false);
  publish_timer_ = nh_.createTimer(publish_duration,
                                   &DataGenerationNode::publishTimerCallback,
                                   this,
                                   false,
                                   true); // Start immediately
}

void DataGenerationNode::initializeServices() {

  srv_start_collection_ =
      nh_.advertiseService("/data_generation/start_collection",
                           &DataGenerationNode::startCollectionCallback,
                           this);
  srv_stop_collection_ = nh_.advertiseService("/data_generation/stop_collection",
                                              &DataGenerationNode::stopCollectionCallback,
                                              this);
}

bool DataGenerationNode::loadGlobalMap(const std::string &path) {

  // Check the input path validity
  if (path.empty()) {
    std::cerr << "\033[31m[data_generation_ros::DataGenerationNode]: "
                 "The path to the reference map is empty!\033[0m\n";
    ros::shutdown();
    return false;
  } else {
    std::cout << "\033[1;33m[data_generation_ros::DataGenerationNode]: "
              << "Reference map path --> " << path << "\033[0m\n";
  }

  std::cout << "\033[1;33m[data_generation_ros::DataGenerationNode]: "
               "Loading reference map...\033[0m\n";

  // Load reference height map from bag file
  try {
    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Read);

    std::vector<std::string> topics{"/height_map_global"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    if (view.size() == 0) {
      bag.close();
      std::cerr << "\033[31m[data_generation_ros::DataGenerationNode]: "
                   "No height map found in the bag file!\033[0m\n";
      ros::shutdown();
      return false;
    }

    // Get the most recent message
    grid_map_msgs::GridMap gridMapMsg;
    for (const rosbag::MessageInstance &m : view) {
      gridMapMsg = *(m.instantiate<grid_map_msgs::GridMap>());
    }
    bag.close();

    // Publish reference map
    pub_ref_map_.publish(gridMapMsg);

    // Convert message to height map
    HeightMap ref_map;
    grid_map::GridMapRosConverter::fromMessage(gridMapMsg, ref_map);

    // Initialize height mapper from reference map
    auto cfg_mapper = global_mapper::loadConfig(ros::NodeHandle(nh_, "height_mapper"));
    height_mapper_ = std::make_unique<HeightMapper>(cfg_mapper, ref_map);

    const auto &map_length_x = ref_map.getLength().x();
    const auto &map_length_y = ref_map.getLength().y();
    const auto &map_resolution = ref_map.getResolution();
    std::cout << "\033[1;32m[data_generation_ros::DataGenerationNode]: "
                 "Reference map loaded successfully with size "
              << map_length_x << " x " << map_length_y << " m "
              << "and resolution " << map_resolution << " m\033[0m\n";

    return true;
  } catch (const std::exception &e) {
    ros::shutdown();
    return false;
  }
}

void DataGenerationNode::lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  if (!lidarscan_received_) {
    lidarscan_received_ = true;
    frame_id_.sensor = msg->header.frame_id;
    std::cout << "\033[1;32m[data_generation_ros::DataGenerationNode]: "
              << "Pointcloud Received! Use LiDAR scans for data generation... \033[0m\n";
  }

  // 1. Get transform matrix using tf tree
  geometry_msgs::TransformStamped sensor2base, base2map;
  if (!tf_.lookupTransform(frame_id_.robot, frame_id_.sensor, sensor2base) ||
      !tf_.lookupTransform(frame_id_.map, frame_id_.robot, base2map))
    return;

  // 2. Process scan
  auto scan_raw = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);
  auto scan_processed = processLidarScan(scan_raw, sensor2base, base2map);

  // 3. Height mapping
  auto scan_rasterized = height_mapper_->heightMapping(scan_processed);

  // 4. Raycasting correction: remove dynamic objects
  auto sensor2map = tf_.multiplyTransforms(sensor2base, base2map);
  Eigen::Vector3f sensorOrigin3D(sensor2map.transform.translation.x,
                                 sensor2map.transform.translation.y,
                                 sensor2map.transform.translation.z);
  height_mapper_->raycasting(sensorOrigin3D, scan_processed);

  // 5. Publish processed scan
  publishScan(scan_processed);

  // 6. Publish local map
  // get local map from global map in height mapper
  auto global_map = height_mapper_->getHeightMap();
  grid_map::Position robot_position(base2map.transform.translation.x,
                                    base2map.transform.translation.y);
  bool is_success;
  auto local_map =
      global_map.getSubmap(robot_position, grid_map::Length(15.0, 15.0), is_success);

  publishLocalMap(local_map);
}

pcl::PointCloud<Laser>::Ptr
DataGenerationNode::processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                                     const geometry_msgs::TransformStamped &lidar2base,
                                     const geometry_msgs::TransformStamped &base2map) {
  // 1. Filter scan
  auto cloud_base = PointCloudOps::applyTransform<Laser>(cloud, lidar2base);
  auto cloud_processed = boost::make_shared<pcl::PointCloud<Laser>>();
  height_mapper_->fastHeightFilter(cloud_base, cloud_processed);
  cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed,
                                                      "x",
                                                      -cfg_.scan_range,
                                                      cfg_.scan_range);
  cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed,
                                                      "y",
                                                      -cfg_.scan_range,
                                                      cfg_.scan_range);

  // 2. Transform scan to map frame
  cloud_processed = PointCloudOps::applyTransform<Laser>(cloud_processed, base2map);

  if (cloud_processed->empty())
    return nullptr;
  return cloud_processed;
}

bool DataGenerationNode::startCollectionCallback(std_srvs::Empty::Request &req,
                                                 std_srvs::Empty::Response &res) {
  if (is_collecting_) {
    std::cout << "\033[1;33m[data_generation_ros::DataGenerationNode]: "
              << "Dataset collection already in progress\033[0m\n";
    return true;
  }

  // Create dataset directories
  // TODO: Check if the directory exists
  if (!cfg_.dataset_output_path.empty()) {
    std::filesystem::create_directories(cfg_.dataset_output_path + "/scans");
    std::filesystem::create_directories(cfg_.dataset_output_path + "/gt_maps");
  } else {
    std::cerr << "\033[1;31m[data_generation_ros::DataGenerationNode]: "
              << "Dataset path is empty!\033[0m\n";
    return false;
  }

  // Start collection
  is_collecting_ = true;
  collection_timer_.start();

  std::cout << "\033[1;32m[data_generation_ros::DataGenerationNode]: "
            << "Dataset collection started. Saving to: " << cfg_.dataset_output_path
            << "\033[0m\n";
  return true;
}

void DataGenerationNode::collectionTimerCallback(const ros::TimerEvent &event) {
  //
}

bool DataGenerationNode::stopCollectionCallback(std_srvs::Empty::Request &req,
                                                std_srvs::Empty::Response &res) {
  if (!is_collecting_) {
    std::cout << "\033[1;33m[data_generation_ros::DataGenerationNode]: "
              << "Dataset collection not in progress\033[0m\n";
    return true;
  }

  // Stop collection
  is_collecting_ = false;
  collection_timer_.stop();

  std::cout << "\033[1;32m[data_generation_ros::DataGenerationNode]: "
            << "Dataset collection stopped. Collected " << sample_count_
            << " samples\033[0m\n";
  return true;
}

void DataGenerationNode::publishTimerCallback(const ros::TimerEvent &event) {
  //
}

void DataGenerationNode::publishScan(const pcl::PointCloud<Laser>::Ptr &cloud) {
  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*cloud, msg_cloud);
  pub_scan_.publish(msg_cloud);
}

void DataGenerationNode::publishLocalMap(const grid_map::GridMap &local_map) {
  grid_map_msgs::GridMap msg_map;
  grid_map::GridMapRosConverter::toMessage(local_map, msg_map);
  pub_local_map_.publish(msg_map);
}

bool DataGenerationNode::extractLocalMap(
    const geometry_msgs::TransformStamped &robot_pose,
    grid_map::GridMap &local_map) {
  // Implementation for local map extraction will be added later
  return false;
}

} // namespace data_generation_ros

int main(int argc, char **argv) {

  ros::init(argc, argv, "data_generation_node");
  data_generation_ros::DataGenerationNode node;
  ros::spin();

  return 0;
}
