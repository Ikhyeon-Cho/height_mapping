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

  // 0. Check validity of parameters
  if (params.gridResolution <= 0) {
    throw std::invalid_argument(
        "[HeightMapping::HeightMapping]: Grid resolution must be positive");
  }
  if (params.mapLengthX <= 0 || params.mapLengthY <= 0) {
    throw std::invalid_argument(
        "[HeightMapping::HeightMapping]: Map dimensions must be positive");
  }

  // 1. Set map frame and geometry
  map_.setFrameId(params.mapFrame);
  map_.setGeometry(grid_map::Length(params.mapLengthX, params.mapLengthY),
                   params.gridResolution);

  // 2. Set height estimator
  // - Kalman Filter
  // - Moving Average
  // - StatMean (by default)
  if (params.heightEstimatorType == "KalmanFilter") {
    std::cout << "\033[1;33m[HeightMapping::HeightMapping]: Height estimator "
                 "type --> KalmanFilter \033[0m\n";
    height_estimator_ = std::make_unique<height_map::KalmanEstimator>();
  } else if (params.heightEstimatorType == "MovingAverage") {
    std::cout << "\033[1;33m[HeightMapping::HeightMapping]: Height estimator "
                 "type --> MovingAverage \033[0m\n";
    height_estimator_ = std::make_unique<height_map::MovingAverageEstimator>();
  } else if (params.heightEstimatorType == "StatMean") {
    std::cout << "\033[1;33m[HeightMapping::HeightMapping]: Height estimator "
                 "type --> StatisticalMeanEstimator "
                 "\033[0m\n";
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  } else {
    ROS_WARN("[HeightMapping::HeightMapping] Invalid height estimator type. "
             "Set Default: StatMeanEstimator");
    height_estimator_ = std::make_unique<height_map::StatMeanEstimator>();
  }
} // End of constructor

template <typename PointT>
void HeightMapping::fastHeightFilter(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    typename pcl::PointCloud<PointT>::Ptr &filtered_cloud) {

  heightFilter_.filter<PointT>(cloud, filtered_cloud);
}

template <typename PointT>
void HeightMapping::updateHeights(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    const Eigen::Affine3d &transform) {

  // downsampling
  auto downsampledCloud = boost::make_shared<pcl::PointCloud<PointT>>();
  height_map::pclProcessor::gridDownsampling<PointT>(downsampledCloud,
                                                     params_.gridResolution);

  // transform
  auto transformedCloud = boost::make_shared<pcl::PointCloud<PointT>>();
  pcl::transformPointCloud(*downsampledCloud, *transformedCloud, transform);

  // estimate height
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
HeightMapping::updateHeights<Laser>(const pcl::PointCloud<Laser>::Ptr &cloud,
                                    const Eigen::Affine3d &transform);
// Color
template void HeightMapping::fastHeightFilter<Color>(
    const typename pcl::PointCloud<Color>::Ptr &cloud,
    typename pcl::PointCloud<Color>::Ptr &filtered_cloud);
template void
HeightMapping::updateHeights<Color>(const pcl::PointCloud<Color>::Ptr &cloud,
                                    const Eigen::Affine3d &transform);
