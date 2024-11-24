/*
 * GlobalMapping.cpp
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/GlobalMapping.h"

GlobalMapping::GlobalMapping(const Parameters &params) {

  globalmap_.setFrameId(params.mapFrame);
  globalmap_.setPosition(grid_map::Position(0.0, 0.0));
  // TODO: Decide if this is needed
  // globalmap_.setPosition(grid_map::Position(globalmap_.getLength().x() / 2,
  // globalmap_.getLength().y() / 2));
  globalmap_.setGeometry(grid_map::Length(params.mapLengthX, params.mapLengthY),
                         params.gridResolution);

  measured_indices_.reserve(globalmap_.getSize().prod());

  if (params.heightEstimatorType == "KalmanFilter") {
    height_estimator_ = std::make_unique<height_map::KalmanEstimator>();
  } else if (params.heightEstimatorType == "MovingAverage") {
    height_estimator_ = std::make_unique<height_map::MovingAverageEstimator>();
  } else if (params.heightEstimatorType == "StatMean") {
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  } else {
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  }
  std::cout << "\033[1;32m[HeightMapping::GlobalMapping]: Waiting for Height "
               "map... \033[0m\n";
}

// Save measured indices for efficiency
template <typename PointT>
void GlobalMapping::addMeasuredGridIndices(
    const grid_map::HeightMap &map, const pcl::PointCloud<PointT> &cloud) {

  grid_map::Index cell_index;
  grid_map::Position cell_position;

  for (const auto &point : cloud.points) {
    cell_position = grid_map::Position(point.x, point.y);

    // Skip if the point is out of the map
    if (!map.getIndex(cell_position, cell_index))
      continue;
    if (!map.isEmptyAt(cell_index))
      continue;

    measured_indices_.insert(cell_index);
  }
}

bool GlobalMapping::clearMap() {
  globalmap_.clearAll();
  return true;
}

bool GlobalMapping::saveLayerToImage(
    height_map_msgs::SaveLayerToImage::Request &req,
    height_map_msgs::SaveLayerToImage::Response &res) {
  // Srv input
  auto img_path = std::filesystem::path(params_.mapSaveDir) / req.map_name;
  const auto &layer = req.layer_name;

  // Check if the directory exists and create if not
  std::filesystem::path output_path(img_path);
  if (output_path.has_parent_path())
    std::filesystem::create_directories(output_path.parent_path());

  if (layer == "") // Save all layers to image
  {
    for (const auto &layer : globalmap_.getLayers()) {
      if (!saveMapToImage(layer, img_path)) {
        res.success = false;
        return false;
      }
    }
    res.success = true;
    res.img_name = (img_path / "**.png").string();
    return true;
  }

  else // Save a single layer to image
  {
    if (!saveMapToImage(layer, img_path)) {
      res.success = false;
      return false;
    }
    res.success = true;
    res.img_name = (img_path / (layer + ".png")).string();
    return true;
  }
}

bool GlobalMapping::saveMapToImage(const std::string &layer,
                                   const std::string &img_path) {
  const auto &data_matrix = globalmap_[layer];

  if (data_matrix.rows() == 0 || data_matrix.cols() == 0) {
    ROS_ERROR("Map has zero size. Skip map saver service");
    return false;
  }

  // Convert to cv image
  cv::Mat image;
  auto min_val = HeightMapMath::getMinVal(globalmap_, layer);
  auto max_val = HeightMapMath::getMaxVal(globalmap_, layer);
  if (!HeightMapConverter::toGrayImage(globalmap_, layer, image, min_val,
                                       max_val)) {
    ROS_ERROR("Failed to convert %s data to image.", layer.c_str());
    return false;
  }

  // Write cv image to a png file
  if (!cv::imwrite(img_path + layer + ".png", image)) {
    ROS_ERROR("Failed to write image to %s.", img_path.c_str());
    return false;
  }

  // Write metadata to a YAML file
  // std::string yaml_path = img_path + layer + ".yaml";
  // YAML::Emitter out;
  // out << YAML::BeginMap;
  // out << YAML::Key << "Layer" << YAML::Value << layer;
  // out << YAML::Key << "Grid Resolution" << YAML::Value
  //     << globalmap_.getResolution();
  // out << YAML::Key << "Min" << YAML::Value << min_val;
  // out << YAML::Key << "Max" << YAML::Value << max_val;
  // out << YAML::EndMap;

  // std::ofstream fout(yaml_path);
  // fout << out.c_str();
  // fout.close();

  // // Write a single metadata.txt: layer, grid resolution, min, max values
  // std::ofstream metadata;
  // metadata.open(img_path + +"_metadata.txt", std::ios_base::app);
  // metadata << "Layer: " << layer << std::endl;
  // metadata << "Grid Resolution: " << map_.getResolution() << std::endl;
  // metadata << "Min: " << min_val << std::endl;
  // metadata << "Max: " << max_val << std::endl << std::endl;
  // metadata.close();

  return true;
}

void GlobalMapping::toPointCloud2(
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
    offset += sizeof(float); // 4 bytes for FLOAT32
  }

  // Initialize cloud size based on measured points
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

  // Adjust final cloud size to actual valid points
  cloud.width = valid_points;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);
}

//////////////////////////////////////////////////
// Explicit instantiation of template functions //
//////////////////////////////////////////////////
// Laser
template void
GlobalMapping::addMeasuredGridIndices(const grid_map::HeightMap &map,
                                      const pcl::PointCloud<Laser> &cloud);

// Color
template void
GlobalMapping::addMeasuredGridIndices(const grid_map::HeightMap &map,
                                      const pcl::PointCloud<Color> &cloud);
