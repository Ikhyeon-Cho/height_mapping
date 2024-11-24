/*
 * HeightMapping.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_ros/HeightMapping.h"

HeightMapping::HeightMapping(const Parameters &params)
    : params_{params}, heightFilter_{params.minHeight, params.maxHeight} {

  paramValidityCheck();

  initHeightMap();

  initHeightEstimator();
}

void HeightMapping::paramValidityCheck() {

  if (params_.gridResolution <= 0) {
    throw std::invalid_argument(
        "[HeightMapping::HeightMapping]: Grid resolution must be positive");
  }
  if (params_.mapLengthX <= 0 || params_.mapLengthY <= 0) {
    throw std::invalid_argument(
        "[HeightMapping::HeightMapping]: Map dimensions must be positive");
  }
}

void HeightMapping::initHeightMap() {
  map_.setFrameId(params_.mapFrame);
  map_.setGeometry(grid_map::Length(params_.mapLengthX, params_.mapLengthY),
                   params_.gridResolution);
}

void HeightMapping::initHeightEstimator() {

  // Set height estimator
  // - Kalman Filter
  // - Moving Average
  // - StatMean (by default)

  if (params_.heightEstimatorType == "KalmanFilter") {
    height_estimator_ = std::make_unique<height_map::KalmanEstimator>();

    std::cout << "\033[1;33m[HeightMapping::HeightMapping]: Height estimator "
                 "type --> KalmanFilter \033[0m\n";

  } else if (params_.heightEstimatorType == "MovingAverage") {
    height_estimator_ = std::make_unique<height_map::MovingAverageEstimator>();

    std::cout << "\033[1;33m[HeightMapping::HeightMapping]: Height estimator "
                 "type --> MovingAverage \033[0m\n";

  } else if (params_.heightEstimatorType == "StatMean") {
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();

    std::cout << "\033[1;33m[HeightMapping::HeightMapping]: Height estimator "
                 "type --> StatisticalMeanEstimator "
                 "\033[0m\n";

  } else {
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();

    std::cout << "\033[1;33m[HeightMapping::HeightMapping] Invalid height "
                 "estimator type. Set Default: StatMeanEstimator \033[0m\n";
  }
}

template <typename PointT>
void HeightMapping::fastHeightFilter(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    typename pcl::PointCloud<PointT>::Ptr &filtered_cloud) {

  heightFilter_.filter<PointT>(cloud, filtered_cloud);
}

template <typename PointT>
void HeightMapping::update(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    const Eigen::Affine3d &transform) {

  // 1. downsampling
  auto downsampledCloud = boost::make_shared<pcl::PointCloud<PointT>>();
  height_map::pclProcessor::gridDownsampling<PointT>(downsampledCloud,
                                                     params_.gridResolution);
  // 2. transform
  auto transformedCloud = boost::make_shared<pcl::PointCloud<PointT>>();
  pcl::transformPointCloud(*downsampledCloud, *transformedCloud, transform);

  // 3. estimate height
  height_estimator_->estimate(map_, *transformedCloud);
}

void HeightMapping::updateMapOrigin(const grid_map::Position &position) {
  map_.move(position);
}

const grid_map::HeightMap &HeightMapping::getHeightMap() const { return map_; }

//////////////////////////////////////////////////
// Explicit instantiation of template functions //
//////////////////////////////////////////////////
// Laser
template void HeightMapping::fastHeightFilter<Laser>(
    const typename pcl::PointCloud<Laser>::Ptr &cloud,
    typename pcl::PointCloud<Laser>::Ptr &filtered_cloud);
template void
HeightMapping::update<Laser>(const pcl::PointCloud<Laser>::Ptr &cloud,
                                    const Eigen::Affine3d &transform);
// Color
template void HeightMapping::fastHeightFilter<Color>(
    const typename pcl::PointCloud<Color>::Ptr &cloud,
    typename pcl::PointCloud<Color>::Ptr &filtered_cloud);
template void
HeightMapping::update<Color>(const pcl::PointCloud<Color>::Ptr &cloud,
                                    const Eigen::Affine3d &transform);
