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
typename pcl::PointCloud<PointT>::Ptr HeightMapping::griddedFilterWithMaxHeight(
    const typename pcl::PointCloud<PointT>::Ptr &cloud, float gridSize) {

  if (cloud->empty()) {
    return cloud;
  }

  // grid cell: first, second -> (x_index, y_index), point
  std::unordered_map<std::pair<int, int>, PointT, pair_hash> grid_map;
  for (const auto &point : *cloud) {
    int x_index = std::floor(point.x / gridSize);
    int y_index = std::floor(point.y / gridSize);

    auto grid_key = std::make_pair(x_index, y_index);
    auto [iter, inserted] = grid_map.emplace(grid_key, point);

    if (!inserted && point.z > iter->second.z) {
      iter->second = point;
    }
  }

  auto cloud_downsampled = boost::make_shared<pcl::PointCloud<PointT>>();
  cloud_downsampled->reserve(grid_map.size());
  for (const auto &grid_cell : grid_map) {
    cloud_downsampled->points.emplace_back(grid_cell.second);
  }

  cloud_downsampled->header = cloud->header;
  return cloud_downsampled;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
HeightMapping::mapping(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                       const Eigen::Affine3d &transform) {

  // 0. Range filter
  auto rangeFilteredCloud = utils::pcl::filterPointcloudByRange2D<PointT>(
      cloud, -params_.mapLengthX * 0.5, params_.mapLengthX * 0.5);

  // 1. Sample pointcloud with max height in each grid cell
  auto griddedCloud = griddedFilterWithMaxHeight<PointT>(
      rangeFilteredCloud, params_.gridResolution);

  // 2. transform
  auto transformedCloud = boost::make_shared<pcl::PointCloud<PointT>>();
  pcl::transformPointCloud(*griddedCloud, *transformedCloud, transform);
  transformedCloud->header.frame_id = params_.mapFrame;

  // 3. estimate height
  height_estimator_->estimate(map_, *transformedCloud);

  return transformedCloud;
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

template pcl::PointCloud<Laser>::Ptr
HeightMapping::griddedFilterWithMaxHeight<Laser>(
    const pcl::PointCloud<Laser>::Ptr &cloud, float gridSize);

template typename pcl::PointCloud<Laser>::Ptr
HeightMapping::mapping<Laser>(const pcl::PointCloud<Laser>::Ptr &cloud,
                              const Eigen::Affine3d &transform);
// Color
template pcl::PointCloud<Color>::Ptr
HeightMapping::griddedFilterWithMaxHeight<Color>(
    const pcl::PointCloud<Color>::Ptr &cloud, float gridSize);

template void HeightMapping::fastHeightFilter<Color>(
    const typename pcl::PointCloud<Color>::Ptr &cloud,
    typename pcl::PointCloud<Color>::Ptr &filtered_cloud);

template typename pcl::PointCloud<Color>::Ptr
HeightMapping::mapping<Color>(const pcl::PointCloud<Color>::Ptr &cloud,
                              const Eigen::Affine3d &transform);