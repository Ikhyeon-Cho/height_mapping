#include "height_mapping_data_collection/DataCollectionNode.h"
#include <chrono>
#include <fstream>
#include <tf2/utils.h>

DataCollectionNode::DataCollectionNode() {

  getNodeParameters();
  getFrameIDs();
  setupROSInterface();

  getDataCollectionParameters();
  initialize();

  std::cout
      << "\033[1;32m[HeightMapping::DataCollection]: Data collection node "
         "initialized. Waiting for scan input... \033[0m\n";
}

void DataCollectionNode::getNodeParameters() {
  publishRate_ = nhDataCollection_.param<double>("publishRate", 10.0); // Hz
  dataCollectionPeriod_ =
      nhDataCollection_.param<double>("dataCollectionPeriod", 1.0); // [s]
}

void DataCollectionNode::getFrameIDs() {
  mapFrame_ = nhFrameID_.param<std::string>("map", "map");
  baselinkFrame_ = nhFrameID_.param<std::string>("base_link", "base_link");
}

void DataCollectionNode::setupROSInterface() {

  subLidarTopic_ = nhDataCollection_.param<std::string>("lidarCloudTopic",
                                                        "/velodyne/points");
  subLidarScan_ = nh_.subscribe(subLidarTopic_, 1,
                                &DataCollectionNode::laserCloudCallback, this);
  pubHeightMap_ = nh_.advertise<grid_map_msgs::GridMap>(
      "/height_mapping/data_collection/map", 1);
  pubScan_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/height_mapping/data_collection/scan", 1);
  startCollectionServer_ = nhPriv_.advertiseService(
      "save_map", &DataCollectionNode::startMapSaverCallback, this);
  dataCollectionTimer_ = nh_.createTimer(
      ros::Duration(dataCollectionPeriod_),
      &DataCollectionNode::dataCollectionTimerCallback, this, false, false);
  publishTimer_ = nh_.createTimer(ros::Duration(1.0 / publishRate_),
                                  &DataCollectionNode::publishTimerCallback,
                                  this, false, false);
}

void DataCollectionNode::getDataCollectionParameters() {

  nhDataCollection_.getParam("globalMapPath", globalMapPath_);
  nhDataCollection_.getParam("dataCollectionPath", dataCollectionPath_);

  // For height map
  minHeightThreshold_ = nhMap_.param<double>("minHeightThreshold", -0.2); // [m]
  maxHeightThreshold_ = nhMap_.param<double>("maxHeightThreshold", 1.5);  // [m]
  mapLength_.x() = nhMap_.param<double>("mapLengthX", 15.0);              // [m]
  mapLength_.y() = nhMap_.param<double>("mapLengthY", 15.0);              // [m]
}

void DataCollectionNode::initialize() {
  globalMap_ = mapReader_.openBagFile(globalMapPath_,
                                      "/height_mapping/globalmap/gridmap");
  heightFilter_ = std::make_shared<height_mapping::FastHeightFilter>(
      minHeightThreshold_, maxHeightThreshold_);
  scanWriter_.setDataPath(dataCollectionPath_ + "/velodyne/");
  mapWriter_.setDataPath(dataCollectionPath_ + "/heightmap/");
}

void DataCollectionNode::laserCloudCallback(
    const sensor_msgs::PointCloud2Ptr &msg) {

  // Get Transform matrix: lidar -> baselink -> map
  const auto &lidarFrame = msg->header.frame_id;
  auto [get1, lidar2Baselink] = tf_.getTransform(lidarFrame, baselinkFrame_);
  auto [get2, baselink2Map] = tf_.getTransform(baselinkFrame_, mapFrame_);
  if (!get1 || !get2)
    return;

  // Prepare pointcloud
  auto inputCloud = boost::make_shared<pcl::PointCloud<Laser>>();
  processedCloud_ = boost::make_shared<pcl::PointCloud<Laser>>();

  // Get pointcloud in baselink frame
  pcl::moveFromROSMsg(*msg, *inputCloud);
  auto baselinkCloud =
      utils::pcl::transformPointcloud<Laser>(inputCloud, lidar2Baselink);

  // Apply height filter
  auto baselinkCloudFiltered = boost::make_shared<pcl::PointCloud<Laser>>();
  heightFilter_->filter<Laser>(baselinkCloud, baselinkCloudFiltered);

  // Filter pointcloud by x-y range
  auto range = mapLength_ / 2.0;
  processedCloud_ = utils::pcl::filterPointcloudByField<Laser>(
      baselinkCloudFiltered, "x", -range.x(), range.x());
  processedCloud_ = utils::pcl::filterPointcloudByField<Laser>(
      processedCloud_, "y", -range.y(), range.y());

  // transform pointcloud to map frame
  processedCloud_ =
      utils::pcl::transformPointcloud<Laser>(processedCloud_, baselink2Map);

  // Get robot-centric heightmap
  grid_map::Position robotPosition(baselink2Map.transform.translation.x,
                                   baselink2Map.transform.translation.y);
  if (!getSubMap(robotPosition, heightMap_))
    return;

  // height update
  updateCurrentHeightMap(heightMap_, processedCloud_);

  // Need this for robot-centric coordinates
  heightMap_.setPosition(heightMap_.getPosition() - robotPosition);
  heightMap_.getHeightMatrix().array() -= baselink2Map.transform.translation.z;
  for (auto &point : processedCloud_->points) {
    point.x -= baselink2Map.transform.translation.x;
    point.y -= baselink2Map.transform.translation.y;
    point.z -= baselink2Map.transform.translation.z;
  }

  // If not, start timer
  dataCollectionTimer_.start();
  publishTimer_.start();
}

bool DataCollectionNode::getSubMap(const grid_map::Position &position,
                                   grid_map::GridMap &map) {
  bool success{false};
  map = globalMap_.getSubmap(position, mapLength_, success);
  return success;
}

void DataCollectionNode::updateCurrentHeightMap(
    grid_map::HeightMap &map, const pcl::PointCloud<Laser>::ConstPtr &cloud) {

  grid_map::Index measuredIndex;
  grid_map::Position measuredPosition;

  auto &heightMatrix = map.getHeightMatrix();
  auto &minHeightMatrix = map.getMinHeightMatrix();
  auto &maxHeightMatrix = map.getMaxHeightMatrix();

  for (auto &point : cloud->points) {
    measuredPosition << point.x, point.y;
    if (!map.getIndex(measuredPosition, measuredIndex))
      continue;

    auto &height = heightMatrix(measuredIndex(0), measuredIndex(1));
    auto &minHeight = minHeightMatrix(measuredIndex(0), measuredIndex(1));
    auto &maxHeight = maxHeightMatrix(measuredIndex(0), measuredIndex(1));

    if (!map.isValid(measuredIndex)) {
      height = point.z;
      minHeight = point.z;
      maxHeight = point.z;
      continue;
    }

    auto alpha = 0.6;
    height = alpha * height + (1 - alpha) * point.z;
    minHeight = alpha * minHeight + (1 - alpha) * point.z;
    maxHeight = alpha * maxHeight + (1 - alpha) * point.z;
  }
}

void DataCollectionNode::dataCollectionTimerCallback(
    const ros::TimerEvent &event) {
  // Process and save like semantic KITTI format
  scanWriter_.writeScan(processedCloud_);

  // Save height map
  try {
    if (!mapWriter_.writeMap(heightMap_)) {
      ROS_WARN("[DataCollectionNode] Failed to write height map");
    }
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM(
        "[DataCollectionNode] Error writing height map: " << e.what());
  }
}

void DataCollectionNode::publishTimerCallback(const ros::TimerEvent &event) {

  // Publish pointcloud
  sensor_msgs::PointCloud2 cloudMsg;
  pcl::toROSMsg(*processedCloud_, cloudMsg);
  pubScan_.publish(cloudMsg);

  // Publish height map
  grid_map_msgs::GridMap heightmapMsg;
  grid_map::GridMapRosConverter::toMessage(heightMap_, heightmapMsg);
  pubHeightMap_.publish(heightmapMsg);
}

bool DataCollectionNode::startMapSaverCallback(std_srvs::Empty::Request &req,
                                               std_srvs::Empty::Response &res) {
  // std::cout << "\033[1;32m[HeightMapping::DataCollection]: Starting height
  // map "
  //              "collection... \033[0m\n";

  // // Read pose data
  // auto poses = scanReader.readPoses();

  // // Data loop
  // for (size_t i = 0; i < poses.size(); ++i) {

  //   // Set map position
  //   heightMapping_->clearMap();
  //   grid_map::Position position(poses[i].translation().x(),
  //                               poses[i].translation().y());
  //   heightMapping_->setMapPosition(position);

  //   // Mapping loop
  //   for (size_t i = 0; i < poses.size(); ++i) {
  //     // Create KITTI-style filename (6 digits with leading zeros)
  //     std::stringstream ss;
  //     ss << std::setw(6) << std::setfill('0') << i;
  //     std::string scanFilename = dataCollectionPath_ + "/" + ss.str() +
  //     ".pcd";

  //     // Read point cloud
  //     auto cloud = boost::make_shared<pcl::PointCloud<Laser>>();
  //     if (pcl::io::loadPCDFile<Laser>(scanFilename, *cloud) == -1) {
  //       ROS_ERROR("Failed to load point cloud: %s", scanFilename.c_str());
  //       return false;
  //     }
  //     cloud->header.frame_id = baselinkFrame_;

  //     if (cloud->empty()) {
  //       std::cout << "\033[1;33m[HeightMapping::DataCollection]: Empty cloud!
  //       "
  //                 << "Skipping... \033[0m\n";
  //       return false;
  //     }

  //     if (cloud->header.frame_id != baselinkFrame_) {
  //       std::cout
  //           << "\033[1;33m[HeightMapping::DataCollection]: Wrong frame ID! "
  //           << "Skipping... \033[0m\n";
  //       std::cout << "Expected: " << baselinkFrame_
  //                 << ", but got: " << cloud->header.frame_id << std::endl;
  //       return false;
  //     }

  //     // Height filtering
  //     auto filteredCloud = boost::make_shared<pcl::PointCloud<Laser>>();
  //     heightMapping_->fastHeightFilter<Laser>(cloud, filteredCloud);

  //     auto mapLength = heightMapping_->getHeightMap().getLength();
  //     filteredCloud = utils::pcl::filterPointcloudByField<Laser>(
  //         filteredCloud, "x", -mapLength.x(), mapLength.x());
  //     filteredCloud = utils::pcl::filterPointcloudByField<Laser>(
  //         filteredCloud, "y", -mapLength.y(), mapLength.y());
  //     if (removeRemoterPoints_) {
  //       filteredCloud = utils::pcl::filterPointcloudByAngle<Laser>(
  //           filteredCloud, -135.0, 135.0);
  //     }
  //     // Transform point cloud to map frame using the corresponding pose
  //     const auto &pose = poses[i];

  //     auto transformedCloud =
  //         utils::pcl::transformPointcloud<Laser>(filteredCloud, pose);

  //     if (transformedCloud->empty()) {
  //       std::cout << "\033[1;33m[HeightMapping::DataCollection]: Empty cloud!
  //       "
  //                 << "Skipping... \033[0m\n";
  //       return false;
  //     }

  //     // Map the filtered cloud
  //     auto mappedCloud = heightMapping_->mapping<Laser>(transformedCloud);

  //     // Optional: Perform raycasting correction
  //     Eigen::Vector3f sensorOrigin(pose.transform.translation.x,
  //                                  pose.transform.translation.y,
  //                                  pose.transform.translation.z);
  //     heightMapping_->raycasting<Laser>(sensorOrigin, mappedCloud);

  //     auto map = heightMapping_->getHeightMap();
  //     // calculate valid cell percentage
  //     int validCellCount = 0;
  //     for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
  //          ++iterator) {
  //       if (map.isValid(*iterator,
  //                       grid_map::HeightMap::CoreLayers::ELEVATION)) {
  //         validCellCount++;
  //       }
  //     }
  //     double validCellPercentage =
  //         static_cast<double>(validCellCount) / map.getSize().prod();
  //     std::cout << "Valid cell percentage: " << validCellPercentage
  //               << std::endl;
  //     if (validCellPercentage > 0.95) {
  //       break;
  //     }

  //     // Publish scan
  //     sensor_msgs::PointCloud2 cloudMsg;
  //     pcl::toROSMsg(*transformedCloud, cloudMsg);
  //     pubScan_.publish(cloudMsg);

  //     // Publish height map
  //     grid_map_msgs::GridMap msg;
  //     grid_map::GridMapRosConverter::toMessage(heightMapping_->getHeightMap(),
  //                                              msg);
  //     pubHeightMap_.publish(msg);

  //     ros::WallDuration(0.01).sleep();

  //   } // Mapping loop end

  //   // sleep
  //   std::cout
  //       << "\033[1;32m[HeightMapping::DataCollection]: Published height map!
  //       "
  //       << "Sleeping for 0.1 seconds... \033[0m\n";
  // } // Data loop end

  // return true;
}
