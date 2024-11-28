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

void GlobalMapping::raycastCorrection(const pcl::PointCloud<Laser> &cloud,
                                      const Eigen::Affine3d &sensor_transform) {
  auto &height_matrix = globalmap_.getHeightMatrix();
  const float cell_size = globalmap_.getResolution();
  const Eigen::Vector3f sensor_origin =
      sensor_transform.translation().cast<float>();

  // For each measured point (we know these were visible)
  for (const auto &point : cloud.points) {
    grid_map::Position point_pos(point.x, point.y);
    grid_map::Index point_index;
    if (!globalmap_.getIndex(point_pos, point_index))
      continue;

    // Cast ray from sensor to point
    Eigen::Vector3f ray_dir(point.x - sensor_origin.x(),
                            point.y - sensor_origin.y(),
                            point.z - sensor_origin.z());
    float ray_length = ray_dir.norm();
    ray_dir.normalize();

    // Check cells along ray path
    float step = cell_size * 0.5f;
    for (float t = 0; t < ray_length - step; t += step) {
      Eigen::Vector3f check_point = sensor_origin + ray_dir * t;
      grid_map::Position check_pos(check_point.x(), check_point.y());
      grid_map::Index check_idx;

      if (globalmap_.getIndex(check_pos, check_idx)) {
        float &height_at_check = height_matrix(check_idx(0), check_idx(1));

        // If map height would block our measured point
        if (height_at_check > check_point.z() + 0.15) {
          height_at_check = check_point.z();
          globalmap_.at("variance", check_idx) =
              std::max(globalmap_.at("variance", check_idx),
                       1.0f); // Increase uncertainty for corrected cells
        }
      }
    }
  }
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
