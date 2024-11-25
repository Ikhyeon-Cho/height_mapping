/*
 * GlobalMapping.cpp
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/GlobalMapping.h"

GlobalMapping::GlobalMapping(const Parameters &params) : params_(params) {

  initGlobalMap();

  initHeightEstimator();

  measured_indices_.reserve(globalmap_.getSize().prod());
}

void GlobalMapping::initGlobalMap() {

  globalmap_.setFrameId(params_.mapFrame);
  globalmap_.setPosition(grid_map::Position(0.0, 0.0));
  // TODO: Decide if this is needed
  // globalmap_.setPosition(grid_map::Position(globalmap_.getLength().x() / 2,
  // globalmap_.getLength().y() / 2));
  globalmap_.setGeometry(
      grid_map::Length(params_.mapLengthX, params_.mapLengthY),
      params_.gridResolution);
}

void GlobalMapping::initHeightEstimator() {

  if (params_.heightEstimatorType == "KalmanFilter") {
    height_estimator_ = std::make_unique<height_map::KalmanEstimator>();
  } else if (params_.heightEstimatorType == "MovingAverage") {
    height_estimator_ = std::make_unique<height_map::MovingAverageEstimator>();
  } else if (params_.heightEstimatorType == "StatMean") {
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  } else {
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  }
}
template <typename PointT>
void GlobalMapping::mapping(const pcl::PointCloud<PointT> &cloud) {
  updateMeasuredGridIndices<PointT>(globalmap_, cloud);
  height_estimator_->estimate(globalmap_, cloud);
}

// Save measured indices for efficiency
template <typename PointT>
void GlobalMapping::updateMeasuredGridIndices(
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

//////////////////////////////////////////////////
// Explicit instantiation of template functions //
//////////////////////////////////////////////////
// Laser
template void
GlobalMapping::mapping<Laser>(const pcl::PointCloud<Laser> &cloud);
template void
GlobalMapping::updateMeasuredGridIndices(const grid_map::HeightMap &map,
                                         const pcl::PointCloud<Laser> &cloud);

// Color
template void
GlobalMapping::mapping<Color>(const pcl::PointCloud<Color> &cloud);
template void
GlobalMapping::updateMeasuredGridIndices(const grid_map::HeightMap &map,
                                         const pcl::PointCloud<Color> &cloud);
