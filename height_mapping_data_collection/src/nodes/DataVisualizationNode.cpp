#include "height_mapping_data_collection/DataVisualizationNode.h"
#include <algorithm>
#include <pcl/io/pcd_io.h>

DataVisualizationNode::DataVisualizationNode() {
  getParameters();
  setupROSInterface();

  // Get all .pcd files in the directory
  for (const auto &entry :
       std::filesystem::directory_iterator(dataDirectory_)) {
    if (entry.path().extension() == ".pcd") {
      cloudFiles_.push_back(entry.path().string());
    }
  }

  // Sort files by name to ensure sequential playback
  std::sort(cloudFiles_.begin(), cloudFiles_.end());

  if (cloudFiles_.empty()) {
    ROS_ERROR("No .pcd files found in directory: %s", dataDirectory_.c_str());
    return;
  }

  ROS_INFO("Found %zu point cloud files", cloudFiles_.size());
  publishTimer_.start();
}

void DataVisualizationNode::getParameters() {
  dataDirectory_ = nhPriv_.param<std::string>(
      "data_directory", std::string("/home/") + std::getenv("USER") +
                            "/data_collection/pointclouds");
  publishRate_ = nhPriv_.param<double>("publish_rate", 10.0); // Hz
  frameId_ = nhPriv_.param<std::string>("frame_id", "map");
}

void DataVisualizationNode::setupROSInterface() {
  pubCloud_ =
      nh_.advertise<sensor_msgs::PointCloud2>("/visualization/pointcloud", 1);

  publishTimer_ =
      nhPriv_.createWallTimer(ros::WallDuration(1.0 / publishRate_),
                              &DataVisualizationNode::publishNextCloud, this);
}

pcl::PointCloud<Laser>::Ptr
DataVisualizationNode::readPCDFile(const std::string &filename) {
  auto cloud = boost::make_shared<pcl::PointCloud<Laser>>();
  if (pcl::io::loadPCDFile<Laser>(filename, *cloud) == -1) {
    throw std::runtime_error("Could not read file: " + filename);
  }
  return cloud;
}

void DataVisualizationNode::publishNextCloud(const ros::WallTimerEvent &event) {

  if (cloudFiles_.empty())
    return;

  try {
    // Read cloud
    auto cloud = readPCDFile(cloudFiles_[currentFileIdx_]);

    // Convert to ROS message
    sensor_msgs::PointCloud2 cloudMsg;
    pcl::toROSMsg(*cloud, cloudMsg);
    cloudMsg.header.frame_id = frameId_;
    cloudMsg.header.stamp = ros::Time::now();

    // Publish
    pubCloud_.publish(cloudMsg);

    std::cout << "Publishing cloud " << currentFileIdx_ + 1 << "/"
              << cloudFiles_.size() << ": "
              << cloudFiles_[currentFileIdx_].c_str() << std::endl;

    // Move to next file
    currentFileIdx_ = (currentFileIdx_ + 1) % cloudFiles_.size();
  } catch (const std::exception &e) {
    ROS_ERROR("Error publishing cloud: %s", e.what());
  }
}