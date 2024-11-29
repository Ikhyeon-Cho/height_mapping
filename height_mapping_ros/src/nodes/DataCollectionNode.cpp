#include "height_mapping_ros/DataCollectionNode.h"
#include <chrono>
#include <fstream>
#include <tf2/utils.h>

DataCollectionNode::DataCollectionNode() {

  getNodeParameters();
  getFrameIDs();
  setupROSInterface();
  getDataCollectionParameters();

  // Initialize height map geometry
  heightMapping_ =
      std::make_unique<HeightMapping>(getHeightMappingParameters());

  std::cout
      << "\033[1;32m[HeightMapping::DataCollection]: Data collection node "
         "initialized. Waiting for LiDAR scan input... \033[0m\n";
}

void DataCollectionNode::getNodeParameters() {
  // poseUpdateRate_ = nhPriv_.param<double>("poseUpdateRate", 10.0); // Hz
}

void DataCollectionNode::getFrameIDs() {
  mapFrame_ = nhFrameID_.param<std::string>("map", "map");
  baselinkFrame_ = nhFrameID_.param<std::string>("base_link", "base_link");
}

void DataCollectionNode::getDataCollectionParameters() {

  nhDataCollection_.getParam("dataCollectionPath", dataCollectionPath_);
}

HeightMapping::Parameters DataCollectionNode::getHeightMappingParameters() {
  HeightMapping::Parameters params;
  params.mapFrame = mapFrame_;
  params.mapLengthX = nhMap_.param<double>("mapLengthX", 15.0); // [m]
  params.mapLengthY = nhMap_.param<double>("mapLengthY", 15.0); // [m]
  params.gridResolution =
      nhMap_.param<double>("gridResolution", 0.1); // [m/grid]
  params.heightEstimatorType =
      nhMap_.param<std::string>("heightEstimatorType", "StatMean");
  params.minHeight = nhMap_.param<double>("minHeightThreshold", -0.2); // [m]
  params.maxHeight = nhMap_.param<double>("maxHeightThreshold", 1.5);  // [m]

  return params;
}

void DataCollectionNode::setupROSInterface() {
  subLidarTopic_ = nhDataCollection_.param<std::string>("lidarCloudTopic",
                                                        "/velodyne/points");
  // Subscriber
  subLidarScan_ = nh_.subscribe(subLidarTopic_, 1,
                                &DataCollectionNode::laserCallback, this);

  pubHeightMap_ = nh_.advertise<grid_map_msgs::GridMap>(
      "/height_mapping/data_collection/map", 1);
  pubScan_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/height_mapping/data_collection/scan", 1);

  // Service server
  startCollectionServer_ = nhPriv_.advertiseService(
      "save_map", &DataCollectionNode::startMapSaverCallback, this);
}

void DataCollectionNode::laserCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  // Get Transform matrix from tf buffer
  const auto &laserFrame = msg->header.frame_id;
  auto [get1, laser2Baselink] = tf_.getTransform(
      laserFrame, baselinkFrame_, msg->header.stamp, ros::Duration(0.01));
  auto [get2, baselink2Map] = tf_.getTransform(
      baselinkFrame_, mapFrame_, msg->header.stamp, ros::Duration(0.01));
  if (!get1 || !get2)
    return;

  // Transform pointcloud to baselink frame
  auto cloud = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *cloud);
  auto baselinkCloud =
      utils::pcl::transformPointcloud<Laser>(cloud, laser2Baselink);

  // Create KITTI-style filename (6 digits with leading zeros)
  std::stringstream ss;
  ss << std::setw(6) << std::setfill('0') << scanCount_++;
  std::string filename = dataCollectionPath_ + "/" + ss.str() + ".pcd";

  // TODO: Folder creation

  // Save pointcloud: takes about 2ms per scan in VLP-16
  if (pcl::io::savePCDFileBinary(filename, *baselinkCloud) == -1) {
    ROS_ERROR_STREAM("Failed to save pointcloud to " << filename);
    return;
  }

  // Save pose data
  // Save as: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
  // T = | r11 r12 r13 tx |
  //     | r21 r22 r23 ty |
  //     | r31 r32 r33 tz |
  //     |  0   0   0   1 |

  double tx = baselink2Map.transform.translation.x;
  double ty = baselink2Map.transform.translation.y;
  double tz = baselink2Map.transform.translation.z;

  tf2::Quaternion q(
      baselink2Map.transform.rotation.x, baselink2Map.transform.rotation.y,
      baselink2Map.transform.rotation.z, baselink2Map.transform.rotation.w);
  tf2::Matrix3x3 rotationMatrix(q);

  // Open file for appending
  auto poseFilename = dataCollectionPath_ + "/poses.txt";
  std::ofstream file(poseFilename, std::ios::app);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << poseFilename << std::endl;
    return;
  }

  // Write transformation in SemanticKITTI format(row - major order)
  file << std::fixed << std::setprecision(6); // Set floating-point precision
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      file << rotationMatrix[i][j] << " ";
    }
    file << (i == 0 ? tx : (i == 1 ? ty : tz))
         << " "; // Add translation component
  }
  file << std::endl;
  file.close();
}

bool DataCollectionNode::startMapSaverCallback(std_srvs::Empty::Request &req,
                                               std_srvs::Empty::Response &res) {
  std::cout << "\033[1;32m[HeightMapping::DataCollection]: Starting height map "
               "collection... \033[0m\n";

  // Read pose data
  auto poseFilename = dataCollectionPath_ + "/poses.txt";
  auto poses = readPosesFromFile(poseFilename);

  // Data loop
  for (size_t i = 0; i < poses.size(); ++i) {

    // Set map position
    heightMapping_->clearMap();
    grid_map::Position position(poses[i].transform.translation.x,
                                poses[i].transform.translation.y);
    heightMapping_->setMapPosition(position);

    // Mapping loop
    for (size_t i = 0; i < poses.size(); ++i) {
      // Create KITTI-style filename (6 digits with leading zeros)
      std::stringstream ss;
      ss << std::setw(6) << std::setfill('0') << i;
      std::string scanFilename = dataCollectionPath_ + "/" + ss.str() + ".pcd";

      // Read point cloud
      auto cloud = boost::make_shared<pcl::PointCloud<Laser>>();
      if (pcl::io::loadPCDFile<Laser>(scanFilename, *cloud) == -1) {
        ROS_ERROR("Failed to load point cloud: %s", scanFilename.c_str());
        return false;
      }
      cloud->header.frame_id = baselinkFrame_;

      if (cloud->empty()) {
        std::cout << "\033[1;33m[HeightMapping::DataCollection]: Empty cloud! "
                  << "Skipping... \033[0m\n";
        return false;
      }

      if (cloud->header.frame_id != baselinkFrame_) {
        std::cout
            << "\033[1;33m[HeightMapping::DataCollection]: Wrong frame ID! "
            << "Skipping... \033[0m\n";
        std::cout << "Expected: " << baselinkFrame_
                  << ", but got: " << cloud->header.frame_id << std::endl;
        return false;
      }

      // Height filtering
      auto filteredCloud = boost::make_shared<pcl::PointCloud<Laser>>();
      heightMapping_->fastHeightFilter<Laser>(cloud, filteredCloud);

      auto mapLength = heightMapping_->getHeightMap().getLength();
      filteredCloud = utils::pcl::filterPointcloudByField<Laser>(
          filteredCloud, "x", -mapLength.x(), mapLength.x());
      filteredCloud = utils::pcl::filterPointcloudByField<Laser>(
          filteredCloud, "y", -mapLength.y(), mapLength.y());
      if (removeRemoterPoints_) {
        filteredCloud = utils::pcl::filterPointcloudByAngle<Laser>(
            filteredCloud, -135.0, 135.0);
      }
      // Transform point cloud to map frame using the corresponding pose
      const auto &pose = poses[i];
      auto transformedCloud =
          utils::pcl::transformPointcloud<Laser>(filteredCloud, pose);

      if (transformedCloud->empty()) {
        std::cout << "\033[1;33m[HeightMapping::DataCollection]: Empty cloud! "
                  << "Skipping... \033[0m\n";
        return false;
      }

      // Map the filtered cloud
      auto mappedCloud = heightMapping_->mapping<Laser>(transformedCloud);

      // Optional: Perform raycasting correction
      Eigen::Vector3f sensorOrigin(pose.transform.translation.x,
                                   pose.transform.translation.y,
                                   pose.transform.translation.z);
      heightMapping_->raycasting<Laser>(sensorOrigin, mappedCloud);

      auto map = heightMapping_->getHeightMap();
      // calculate valid cell percentage
      int validCellCount = 0;
      for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
           ++iterator) {
        if (map.isValid(*iterator,
                        grid_map::HeightMap::CoreLayers::ELEVATION)) {
          validCellCount++;
        }
      }
      double validCellPercentage =
          static_cast<double>(validCellCount) / map.getSize().prod();
      std::cout << "Valid cell percentage: " << validCellPercentage
                << std::endl;
      if (validCellPercentage > 0.95) {
        break;
      }

      // Publish scan
      sensor_msgs::PointCloud2 cloudMsg;
      pcl::toROSMsg(*transformedCloud, cloudMsg);
      pubScan_.publish(cloudMsg);

      // Publish height map
      grid_map_msgs::GridMap msg;
      grid_map::GridMapRosConverter::toMessage(heightMapping_->getHeightMap(),
                                               msg);
      pubHeightMap_.publish(msg);

      ros::WallDuration(0.01).sleep();

    } // Mapping loop end

    // sleep
    std::cout
        << "\033[1;32m[HeightMapping::DataCollection]: Published height map! "
        << "Sleeping for 0.1 seconds... \033[0m\n";
  } // Data loop end

  return true;
}

std::vector<geometry_msgs::TransformStamped>
DataCollectionNode::readPosesFromFile(const std::string &pose_file) {
  std::vector<geometry_msgs::TransformStamped> transforms;
  std::ifstream file(pose_file);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << pose_file << std::endl;
    return transforms;
  }

  std::string line;
  int count = 0;
  while (std::getline(file, line)) {
    std::istringstream stream(line);
    double r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz;

    // Read the 12 values from the file
    if (!(stream >> r11 >> r12 >> r13 >> tx >> r21 >> r22 >> r23 >> ty >> r31 >>
          r32 >> r33 >> tz)) {
      std::cerr << "Invalid pose format on line " << count + 1 << std::endl;
      continue;
    }

    // Convert rotation matrix to quaternion
    tf2::Matrix3x3 rotation_matrix(r11, r12, r13, r21, r22, r23, r31, r32, r33);
    tf2::Quaternion quaternion;
    rotation_matrix.getRotation(quaternion);

    // Populate TransformStamped
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now(); // Ignore this: no use
    transform.header.frame_id = mapFrame_;
    transform.child_frame_id = baselinkFrame_;
    transform.transform.translation.x = tx;
    transform.transform.translation.y = ty;
    transform.transform.translation.z = tz;
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();

    transforms.push_back(transform);
    ++count;
  }

  file.close();
  return transforms;
}
