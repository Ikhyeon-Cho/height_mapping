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
  globalmap_.setGeometry(
      grid_map::Length(params_.mapLengthX, params_.mapLengthY),
      params_.gridResolution);
}

void GlobalMapping::initHeightEstimator() {

  if (params_.heightEstimatorType == "KalmanFilter") {
    height_estimator_ = std::make_unique<height_mapping::KalmanEstimator>();
  } else if (params_.heightEstimatorType == "MovingAverage") {
    height_estimator_ =
        std::make_unique<height_mapping::MovingAverageEstimator>();
  } else if (params_.heightEstimatorType == "StatMean") {
    height_estimator_ = std::make_unique<height_mapping::StatMeanEstimator>();
  } else {
    height_estimator_ = std::make_unique<height_mapping::StatMeanEstimator>();
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

void GlobalMapping::addBasicLayer(const std::string &layer) {
  auto basic_layers = globalmap_.getBasicLayers();
  basic_layers.insert(basic_layers.end(), {layer});
  globalmap_.setBasicLayers(basic_layers);
}

void GlobalMapping::clearMap() { globalmap_.clearAll(); }

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
