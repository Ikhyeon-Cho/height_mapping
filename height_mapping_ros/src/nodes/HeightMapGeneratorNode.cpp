#include "height_mapping_ros/HeightMapGeneratorNode.h"
#include <pcl/io/pcd_io.h>

HeightMapGeneratorNode::HeightMapGeneratorNode() {
  getParameters();
  setupROSInterface();

  // Initialize grid map
  heightMap_.setFrameId(frameId_);
  heightMap_.add("elevation");
  heightMap_.add("variance");
  heightMap_.add("num_points");

  // Get all .pcd files
  for (const auto &entry :
       std::filesystem::directory_iterator(dataDirectory_)) {
    if (entry.path().extension() == ".pcd") {
      cloudFiles_.push_back(entry.path().string());
    }
  }
  std::sort(cloudFiles_.begin(), cloudFiles_.end());

  if (cloudFiles_.empty()) {
    ROS_ERROR("No .pcd files found in directory: %s", dataDirectory_.c_str());
    return;
  }

  ROS_INFO("Found %zu point cloud files", cloudFiles_.size());
  generateHeightMap();
  publishTimer_.start();
}

void HeightMapGeneratorNode::getParameters() {
  dataDirectory_ = nhPriv_.param<std::string>(
      "data_directory", std::string("/home/") + std::getenv("USER") +
                            "/data_collection/pointclouds");
  publishRate_ = nhPriv_.param<double>("publish_rate", 1.0);     // Hz
  mapResolution_ = nhPriv_.param<double>("map_resolution", 0.1); // meters
  minHeight_ = nhPriv_.param<double>("min_height", -1.0);        // meters
  maxHeight_ = nhPriv_.param<double>("max_height", 2.0);         // meters
  numScansToAggregate_ = nhPriv_.param<int>("num_scans_to_aggregate", 10);
  frameId_ = nhPriv_.param<std::string>("frame_id", "map");
}

void HeightMapGeneratorNode::setupROSInterface() {
  pubHeightMap_ = nh_.advertise<grid_map_msgs::GridMap>(
      "/height_map_generator/height_map", 1);

  publishTimer_ =
      nhPriv_.createWallTimer(ros::WallDuration(1.0 / publishRate_),
                              &HeightMapGeneratorNode::publishHeightMap, this);
}

pcl::PointCloud<Laser>::Ptr
HeightMapGeneratorNode::readPCDFile(const std::string &filename) {
  auto cloud = boost::make_shared<pcl::PointCloud<Laser>>();
  if (pcl::io::loadPCDFile<Laser>(filename, *cloud) == -1) {
    throw std::runtime_error("Could not read file: " + filename);
  }
  return cloud;
}

void HeightMapGeneratorNode::processPointCloud(
    const pcl::PointCloud<Laser>::Ptr &cloud) {
  for (const auto &point : cloud->points) {
    grid_map::Position position(point.x, point.y);
    grid_map::Index index;
    if (!heightMap_.getIndex(position, index)) {
      continue;
    }

    // Skip points outside height range
    if (point.z < minHeight_ || point.z > maxHeight_) {
      continue;
    }

    // Update elevation (mean height)
    const auto &numPoints = heightMap_.at("num_points", index);
    const auto &currentElevation = heightMap_.at("elevation", index);

    if (numPoints == 0) {
      heightMap_.at("elevation", index) = point.z;
      heightMap_.at("variance", index) = 0.0;
    } else {
      // Incremental mean and variance updates
      const double delta = point.z - currentElevation;
      heightMap_.at("elevation", index) += delta / (numPoints + 1);
      heightMap_.at("variance", index) +=
          delta * (point.z - heightMap_.at("elevation", index));
    }
    heightMap_.at("num_points", index) += 1;
  }
}

pcl::PointCloud<Laser>::Ptr HeightMapGeneratorNode::griddedFilterWithMaxHeight(
    const pcl::PointCloud<Laser>::Ptr& cloud, float gridSize) {
  if (cloud->empty()) {
    return cloud;
  }

  // grid cell: first, second -> (x_index, y_index), point
  std::unordered_map<std::pair<int, int>, Laser, pair_hash> grid_map;
  for (const auto& point : *cloud) {
    int x_index = std::floor(point.x / gridSize);
    int y_index = std::floor(point.y / gridSize);

    auto grid_key = std::make_pair(x_index, y_index);
    auto [iter, inserted] = grid_map.emplace(grid_key, point);

    if (!inserted && point.z > iter->second.z) {
      iter->second = point;
    }
  }

  auto cloud_downsampled = boost::make_shared<pcl::PointCloud<Laser>>();
  cloud_downsampled->reserve(grid_map.size());
  for (const auto& grid_cell : grid_map) {
    cloud_downsampled->points.emplace_back(grid_cell.second);
  }

  cloud_downsampled->header = cloud->header;
  return cloud_downsampled;
}

pcl::PointCloud<Laser>::Ptr HeightMapGeneratorNode::raycastFiltering(
    const pcl::PointCloud<Laser>::Ptr& cloud) {
  if (cloud->empty()) return cloud;

  // 1. Create a 2D grid for height map and visibility
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();

  for (const auto& point : cloud->points) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }

  const float cell_size = mapResolution_;
  const int grid_size_x = std::ceil((max_x - min_x) / cell_size) + 1;
  const int grid_size_y = std::ceil((max_y - min_y) / cell_size) + 1;

  // Initialize height map with minimum height
  std::vector<std::vector<float>> height_map(
      grid_size_x, std::vector<float>(grid_size_y, minHeight_));
  std::vector<std::vector<bool>> visibility_map(
      grid_size_x, std::vector<bool>(grid_size_y, false));

  // 2. Project points to 2D grid and keep maximum height
  for (const auto& point : cloud->points) {
    int idx_x = static_cast<int>((point.x - min_x) / cell_size);
    int idx_y = static_cast<int>((point.y - min_y) / cell_size);
    
    if (idx_x >= 0 && idx_x < grid_size_x && 
        idx_y >= 0 && idx_y < grid_size_y) {
      height_map[idx_x][idx_y] = std::max(height_map[idx_x][idx_y], point.z);
    }
  }

  // 3. Perform raycasting from sensor origin (assumed to be 0,0,0 if not available)
  const Eigen::Vector3f sensor_origin(0.0f, 0.0f, 0.0f);  // Modify if sensor origin is known
  
  // Mark visible cells using raycasting
  for (int x = 0; x < grid_size_x; ++x) {
    for (int y = 0; y < grid_size_y; ++y) {
      if (height_map[x][y] > minHeight_) {
        float world_x = min_x + x * cell_size;
        float world_y = min_y + y * cell_size;
        float world_z = height_map[x][y];

        // Ray from sensor to point
        Eigen::Vector3f ray_dir(world_x - sensor_origin.x(),
                              world_y - sensor_origin.y(),
                              world_z - sensor_origin.z());
        float ray_length = ray_dir.norm();
        ray_dir.normalize();

        // Check for occlusions along the ray
        bool is_visible = true;
        float step_size = cell_size * 0.5f;
        for (float t = 0; t < ray_length - step_size; t += step_size) {
          Eigen::Vector3f check_point = sensor_origin + ray_dir * t;
          int check_x = static_cast<int>((check_point.x() - min_x) / cell_size);
          int check_y = static_cast<int>((check_point.y() - min_y) / cell_size);

          if (check_x >= 0 && check_x < grid_size_x && 
              check_y >= 0 && check_y < grid_size_y) {
            if (height_map[check_x][check_y] > check_point.z() + 0.1f) {  // 10cm threshold
              is_visible = false;
              break;
            }
          }
        }
        visibility_map[x][y] = is_visible;
      }
    }
  }

  // 4. Create filtered cloud with only visible points
  auto filtered_cloud = boost::make_shared<pcl::PointCloud<Laser>>();
  for (const auto& point : cloud->points) {
    int idx_x = static_cast<int>((point.x - min_x) / cell_size);
    int idx_y = static_cast<int>((point.y - min_y) / cell_size);
    
    if (idx_x >= 0 && idx_x < grid_size_x && 
        idx_y >= 0 && idx_y < grid_size_y) {
      if (visibility_map[idx_x][idx_y] && 
          std::abs(point.z - height_map[idx_x][idx_y]) < 0.1f) {  // Only keep points near maximum height
        filtered_cloud->points.push_back(point);
      }
    }
  }

  filtered_cloud->width = filtered_cloud->points.size();
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = true;
  filtered_cloud->header = cloud->header;

  return filtered_cloud;
}

void HeightMapGeneratorNode::generateHeightMap() {
  ROS_INFO("Generating height map from %d scans...", numScansToAggregate_);

  // Find map bounds from first scan
  auto firstCloud = readPCDFile(cloudFiles_[0]);
  grid_map::Position minBound(std::numeric_limits<double>::max(),
                              std::numeric_limits<double>::max());
  grid_map::Position maxBound(std::numeric_limits<double>::lowest(),
                              std::numeric_limits<double>::lowest());

  // Fix type mismatch by casting point coordinates to double
  for (const auto &point : firstCloud->points) {
    minBound.x() = std::min(minBound.x(), static_cast<double>(point.x));
    minBound.y() = std::min(minBound.y(), static_cast<double>(point.y));
    minBound << -15.0 / 2, -15.0 / 2;
    maxBound.x() = std::max(maxBound.x(), static_cast<double>(point.x));
    maxBound.y() = std::max(maxBound.y(), static_cast<double>(point.y));
    maxBound << 15.0 / 2, 15.0 / 2;
  }

  // Add margin
  const double margin = 1.0; // meters
  minBound -= grid_map::Position(margin, margin);
  maxBound += grid_map::Position(margin, margin);

  // Initialize grid map
  grid_map::Length mapLength = maxBound - minBound;
  heightMap_.setGeometry(mapLength, mapResolution_,
                         (minBound + maxBound) / 2.0);
  heightMap_.clearAll();
  heightMap_.get("num_points").setConstant(0.0);

  // Process specified number of scans
  const int numScans = std::min(numScansToAggregate_, 
                               static_cast<int>(cloudFiles_.size()));
  for (int i = 0; i < numScans; ++i) {
    auto cloud = readPCDFile(cloudFiles_[i]);

    // 1. Remove points outside height range
    pcl::PointCloud<Laser>::Ptr heightFilteredCloud(new pcl::PointCloud<Laser>);
    for (const auto& point : cloud->points) {
      if (point.z >= minHeight_ && point.z <= maxHeight_) {
        heightFilteredCloud->points.push_back(point);
      }
    }
    heightFilteredCloud->width = heightFilteredCloud->points.size();
    heightFilteredCloud->height = 1;
    heightFilteredCloud->is_dense = true;

    // 2. Apply raycasting-based filtering
    auto raycastedCloud = raycastFiltering(heightFilteredCloud);

    // 3. Sample pointcloud with max height in each grid cell
    auto griddedCloud = griddedFilterWithMaxHeight(raycastedCloud, mapResolution_);

    // 4. Process filtered cloud
    processPointCloud(griddedCloud);
    ROS_INFO_THROTTLE(1.0, "Processing scan %d/%d", i + 1, numScans);
  }

  // Compute final variance
  for (grid_map::GridMapIterator it(heightMap_); !it.isPastEnd(); ++it) {
    const auto numPoints = heightMap_.at("num_points", *it);
    if (numPoints > 1) {
      heightMap_.at("variance", *it) /= (numPoints - 1);
    }
  }

  ROS_INFO("Height map generation complete!");
}

void HeightMapGeneratorNode::publishHeightMap(
    const ros::WallTimerEvent &event) {
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(heightMap_, msg);
  pubHeightMap_.publish(msg);
}