/*
 * global_mapping_node.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/ros/global_mapping_node.h"
#include "height_mapping/ros/config.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>

namespace {
std::string determineSaveDirectory(const std::string &requested_directory);
std::string createFilename(const std::string &directory,
                           const std::string &requested_filename);
} // namespace

namespace height_mapping_ros {

GlobalMappingNode::GlobalMappingNode() : nh_("~") {

  // Configs from global_mapping_node.yaml
  ros::NodeHandle cfg_node(nh_, "node");
  ros::NodeHandle cfg_frame_id(nh_, "frame_id");
  ros::NodeHandle cfg_mapper(nh_, "global_mapper");

  // ROS node
  GlobalMappingNode::loadConfig(cfg_node);
  initializeTimers();
  initializePubSubs();
  initializeServices();

  frame_id_ = FrameID::loadFromConfig(cfg_frame_id);

  // Global Mapper
  mapper_ = std::make_unique<height_mapping::GlobalMapper>(
      global_mapper::loadConfig(cfg_mapper));

  std::cout << "\033[1;33m[height_mapping_ros::GlobalMappingNode]: "
               "Global mapping node initialized. Waiting for scan inputs...\033[0m\n";
}

void GlobalMappingNode::loadConfig(const ros::NodeHandle &nh) {

  cfg.lidarcloud_topic = nh.param<std::string>("lidar_topic", "/velodyne_points");
  cfg.rgbdcloud_topic = nh.param<std::string>("rgbd_topic", "/camera/pointcloud/points");
  cfg.map_publish_rate = nh.param<double>("map_publish_rate", 10.0);
  cfg.remove_backward_points = nh.param<bool>("remove_backward_points", false);
  cfg.debug_mode = nh.param<bool>("debug_mode", false);
}

void GlobalMappingNode::initializeTimers() {

  auto map_publish_duration = ros::Duration(1.0 / cfg.map_publish_rate);
  map_publish_timer_ = nh_.createTimer(map_publish_duration,
                                       &GlobalMappingNode::publishPointCloudMap,
                                       this,
                                       false,
                                       false);
}

void GlobalMappingNode::initializePubSubs() {

  sub_lidarscan_ =
      nh_.subscribe(cfg.lidarcloud_topic, 1, &GlobalMappingNode::lidarScanCallback, this);
  sub_rgbdscan_ =
      nh_.subscribe(cfg.rgbdcloud_topic, 1, &GlobalMappingNode::rgbdScanCallback, this);
  pub_scan_rasterized_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/global/scan_rasterized",
                                              1);
  pub_map_cloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/global/map_cloud", 1);
  pub_map_region_ =
      nh_.advertise<visualization_msgs::Marker>("/height_mapping/global/mapping_region",
                                                1);
}

void GlobalMappingNode::initializeServices() {

  srv_save_map_ = nh_.advertiseService("/height_mapping/global/save_map",
                                       &GlobalMappingNode::saveMap,
                                       this);
  srv_clear_map_ = nh_.advertiseService("/height_mapping/global/clear_map",
                                        &GlobalMappingNode::clearMap,
                                        this);
}

void GlobalMappingNode::lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  if (!lidarscan_received_) {
    lidarscan_received_ = true;
    frame_id_.sensor = msg->header.frame_id;
    map_publish_timer_.start();
    std::cout
        << "\033[1;32m[height_mapping_ros::GlobalMappingNode]: Pointcloud Received! "
        << "Use LiDAR scans for global mapping... \033[0m\n";
  }

  // 1. Get transform matrix using tf tree
  geometry_msgs::TransformStamped sensor2base, base2map;
  if (!tf_.lookupTransform(frame_id_.robot, frame_id_.sensor, sensor2base) ||
      !tf_.lookupTransform(frame_id_.map, frame_id_.robot, base2map))
    return;

  // 2. Convert ROS msg to PCL data
  auto scan_raw = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  auto scan_processed = processLidarScan(scan_raw, sensor2base, base2map);
  if (!scan_processed)
    return;

  auto scan_rasterized = mapper_->heightMapping(scan_processed);

  auto sensor2map = tf_.multiplyTransforms(sensor2base, base2map);
  Eigen::Vector3f sensorOrigin3D(sensor2map.transform.translation.x,
                                 sensor2map.transform.translation.y,
                                 sensor2map.transform.translation.z);
  mapper_->raycasting(sensorOrigin3D, scan_processed);

  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*scan_rasterized, msg_cloud);
  pub_scan_rasterized_.publish(msg_cloud);
}

void GlobalMappingNode::rgbdScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  if (!rgbdscan_received_) {
    rgbdscan_received_ = true;
    frame_id_.sensor = msg->header.frame_id;
    map_publish_timer_.start();
    std::cout
        << "\033[1;32m[height_mapping_ros::GlobalMappingNode]: Colored cloud received! "
        << "Start global mapping... \033[0m\n";
  }

  // 1. Get transform matrix using tf tree
  geometry_msgs::TransformStamped sensor2base, base2map;
  if (!tf_.lookupTransform(frame_id_.robot, frame_id_.sensor, sensor2base) ||
      !tf_.lookupTransform(frame_id_.map, frame_id_.robot, base2map))
    return;

  // 2. Convert ROS msg to PCL data
  auto scan_raw = boost::make_shared<pcl::PointCloud<Color>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  auto scan_processed = processRGBDScan(scan_raw, sensor2base, base2map);
  if (!scan_processed)
    return;

  auto scan_rasterized = mapper_->heightMapping(scan_processed);

  auto sensor2map = tf_.multiplyTransforms(sensor2base, base2map);
  Eigen::Vector3f sensorOrigin3D(sensor2map.transform.translation.x,
                                 sensor2map.transform.translation.y,
                                 sensor2map.transform.translation.z);
  mapper_->raycasting(sensorOrigin3D, scan_processed);

  sensor_msgs::PointCloud2 msg_cloud;
  pcl::toROSMsg(*scan_rasterized, msg_cloud);
  pub_scan_rasterized_.publish(msg_cloud);
}

pcl::PointCloud<Laser>::Ptr
GlobalMappingNode::processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                                    const geometry_msgs::TransformStamped &lidar2base,
                                    const geometry_msgs::TransformStamped &base2map) {
  // 1. Filter local pointcloud
  auto cloud_base = PointCloudOps::applyTransform<Laser>(cloud, lidar2base);
  auto cloud_processed = boost::make_shared<pcl::PointCloud<Laser>>();
  mapper_->fastHeightFilter(cloud_base, cloud_processed);
  cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed, "x", -10.0, 10.0);
  cloud_processed = PointCloudOps::passThrough<Laser>(cloud_processed, "y", -10.0, 10.0);

  // (Optional) Remove remoter points
  if (cfg.remove_backward_points)
    cloud_processed = PointCloudOps::filterAngle2D<Laser>(cloud_processed, -135.0, 135.0);

  // 2. Transform pointcloud to map frame
  cloud_processed = PointCloudOps::applyTransform<Laser>(cloud_processed, base2map);

  if (cloud_processed->empty())
    return nullptr;
  return cloud_processed;
}

pcl::PointCloud<Color>::Ptr
GlobalMappingNode::processRGBDScan(const pcl::PointCloud<Color>::Ptr &cloud,
                                   const geometry_msgs::TransformStamped &camera2base,
                                   const geometry_msgs::TransformStamped &base2map) {

  auto cloud_base = PointCloudOps::applyTransform<Color>(cloud, camera2base);
  auto cloud_processed = boost::make_shared<pcl::PointCloud<Color>>();
  mapper_->fastHeightFilter(cloud_base, cloud_processed);
  cloud_processed = PointCloudOps::passThrough<Color>(cloud_processed, "x", -10.0, 10.0);
  cloud_processed = PointCloudOps::passThrough<Color>(cloud_processed, "y", -10.0, 10.0);

  // (Optional) Remove remoter points
  if (cfg.remove_backward_points)
    cloud_processed = PointCloudOps::filterAngle2D<Color>(cloud_processed, -135.0, 135.0);

  cloud_processed = PointCloudOps::applyTransform<Color>(cloud_processed, base2map);

  if (cloud_processed->empty())
    return nullptr;
  return cloud_processed;
}

void GlobalMappingNode::publishPointCloudMap(const ros::TimerEvent &) {

  std::vector<std::string> layers = {
      height_mapping::layers::Height::ELEVATION,
      height_mapping::layers::Height::ELEVATION_MAX,
      height_mapping::layers::Height::ELEVATION_MIN,
      height_mapping::layers::Height::ELEVATION_VARIANCE,
      height_mapping::layers::Height::N_MEASUREMENTS,

  };
  // Visualize global map
  sensor_msgs::PointCloud2 msg_map_cloud;
  toPointCloud2(mapper_->getHeightMap(),
                layers,
                mapper_->getMeasuredGridIndices(),
                msg_map_cloud);
  pub_map_cloud_.publish(msg_map_cloud);

  // Visualize map region
  visualization_msgs::Marker msg_map_region;
  toMapRegion(mapper_->getHeightMap(), msg_map_region);
  pub_map_region_.publish(msg_map_region);
}

void GlobalMappingNode::toPointCloud2(
    const HeightMap &map,
    const std::vector<std::string> &layers,
    const std::unordered_set<grid_map::Index> &measuredIndices,
    sensor_msgs::PointCloud2 &cloud) {

  // Setup cloud header
  cloud.header.frame_id = map.getFrameId();
  cloud.header.stamp.fromNSec(map.getTimestamp());
  cloud.is_dense = false;

  // Setup field names and cloud structure
  std::vector<std::string> fieldNames;
  fieldNames.reserve(layers.size());

  // Setup field names
  fieldNames.insert(fieldNames.end(), {"x", "y", "z"});
  for (const auto &layer : layers) {
    if (layer == "color") {
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
  const size_t num_points = measuredIndices.size();
  cloud.height = 1;
  cloud.width = num_points;
  cloud.point_step = offset;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);

  // Setup point field iterators
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>> iterators;
  for (const auto &name : fieldNames) {
    iterators.emplace(name, sensor_msgs::PointCloud2Iterator<float>(cloud, name));
  }

  // Fill point cloud data
  size_t validPoints = 0;
  for (const auto &index : measuredIndices) {
    grid_map::Position3 position;
    if (!map.getPosition3(height_mapping::layers::Height::ELEVATION, index, position)) {
      continue;
    }

    // Update each field
    for (auto &[fieldName, iterator] : iterators) {
      if (fieldName == "x")
        *iterator = static_cast<float>(position.x());
      else if (fieldName == "y")
        *iterator = static_cast<float>(position.y());
      else if (fieldName == "z")
        *iterator = static_cast<float>(position.z());
      else if (fieldName == "rgb")
        *iterator = static_cast<float>(map.at("color", index));
      else
        *iterator = static_cast<float>(map.at(fieldName, index));
      ++iterator;
    }
    ++validPoints;
  }

  // Adjust final cloud size
  cloud.width = validPoints;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);
}

void GlobalMappingNode::toMapRegion(const height_mapping::HeightMap &map,
                                    visualization_msgs::Marker &marker) {

  marker.ns = "height_map";
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.header.frame_id = map.getFrameId();
  marker.header.stamp = ros::Time::now();

  float length_x_half = (map.getLength().x() - 0.5 * map.getResolution()) / 2.0;
  float length_y_half = (map.getLength().y() - 0.5 * map.getResolution()) / 2.0;

  marker.points.resize(5);
  marker.points[0].x = map.getPosition().x() + length_x_half;
  marker.points[0].y = map.getPosition().y() + length_x_half;
  marker.points[0].z = 0;

  marker.points[1].x = map.getPosition().x() + length_x_half;
  marker.points[1].y = map.getPosition().y() - length_x_half;
  marker.points[1].z = 0;

  marker.points[2].x = map.getPosition().x() - length_x_half;
  marker.points[2].y = map.getPosition().y() - length_x_half;
  marker.points[2].z = 0;

  marker.points[3].x = map.getPosition().x() - length_x_half;
  marker.points[3].y = map.getPosition().y() + length_x_half;
  marker.points[3].z = 0;

  marker.points[4] = marker.points[0];
}

bool GlobalMappingNode::saveMap(height_mapping::save_map::Request &req,
                                height_mapping::save_map::Response &res) {

  std::cout
      << "\033[1;32m[height_mapping_ros::GlobalMappingNode]: Saving map...\033[0m\n";

  std::string directory = determineSaveDirectory(req.directory);
  std::string filename = createFilename(directory, req.filename);

  pcl::PointCloud<height_mapping_types::ElevationPoint> cloud;
  auto &map = mapper_->getHeightMap();
  auto &indices = mapper_->getMeasuredGridIndices();
  if (!toPclCloud(map, indices, cloud)) {
    res.success = false;
    return false;
  }

  bool success = savePointCloud(cloud, filename);
  res.success = success;
  return success;
}

bool GlobalMappingNode::toPclCloud(
    const HeightMap &map,
    const std::unordered_set<grid_map::Index> &grid_indices,
    pcl::PointCloud<height_mapping_types::ElevationPoint> &cloud) {

  cloud.header.frame_id = map.getFrameId();
  cloud.header.stamp = pcl_conversions::toPCL(ros::Time::now());
  cloud.points.reserve(grid_indices.size());

  try {
    for (const auto &index : grid_indices) {
      grid_map::Position3 position;
      map.getPosition3(height_mapping::layers::Height::ELEVATION, index, position);

      height_mapping_types::ElevationPoint point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.elevation = map.at(height_mapping::layers::Height::ELEVATION, index);
      point.elevation_min = map.at(height_mapping::layers::Height::ELEVATION_MIN, index);
      point.elevation_max = map.at(height_mapping::layers::Height::ELEVATION_MAX, index);
      point.elevation_variance =
          map.at(height_mapping::layers::Height::ELEVATION_VARIANCE, index);
      point.n_measurements =
          map.at(height_mapping::layers::Height::N_MEASUREMENTS, index);

      cloud.push_back(point);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;
  } catch (const std::exception &e) {
    std::cerr << "Error converting height map to point cloud: " << e.what() << std::endl;
    return false;
  }

  return true;
}

bool GlobalMappingNode::savePointCloud(
    const pcl::PointCloud<height_mapping_types::ElevationPoint> &cloud,
    const std::string &filename) {
  if (pcl::io::savePCDFileASCII(filename, cloud) == -1) {
    std::cerr << "\033[33m[height_mapping_ros::GlobalMappingNode]: Failed to save "
                 "elevation cloud to "
              << filename << "\033[0m\n";
    return false;
  }

  std::cout
      << "\033[1;32m[height_mapping_ros::GlobalMappingNode]: elevation cloud saved to "
      << filename << "\033[0m\n";
  return true;
}

bool GlobalMappingNode::clearMap(std_srvs::Empty::Request &req,
                                 std_srvs::Empty::Response &res) {
  mapper_->clearMap();
  return true;
}

} // namespace height_mapping_ros

namespace {
std::string determineSaveDirectory(const std::string &requested_directory) {
  if (requested_directory.empty()) {
    // return package path
    std::string package_path = ros::package::getPath("height_mapping");
    size_t last_slash_pos = package_path.find_last_of('/');
    return (last_slash_pos != std::string::npos)
               ? package_path.substr(0, last_slash_pos + 1)
               : package_path + "/";
  }

  // Preprocess requested directory ("~", "*/")
  std::string directory = requested_directory;
  if (!directory.empty() && directory[0] == '~') {
    if (const char *home = std::getenv("HOME")) {
      directory.replace(0, 1, home);
    }
  }
  if (!directory.empty() && directory.back() != '/') {
    directory += '/';
  }

  return directory;
}

std::string createFilename(const std::string &directory,
                           const std::string &requested_filename) {
  if (requested_filename.empty()) {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");
    return directory + "elevation_" + ss.str() + ".pcd";
  }

  std::string filename = directory + requested_filename;
  if (filename.length() < 4 || filename.substr(filename.length() - 4) != ".pcd") {
    filename += ".pcd";
  }

  return filename;
}
} // namespace

int main(int argc, char **argv) {

  ros::init(argc, argv, "global_mapping_node");
  height_mapping_ros::GlobalMappingNode node;
  ros::spin();

  return 0;
}