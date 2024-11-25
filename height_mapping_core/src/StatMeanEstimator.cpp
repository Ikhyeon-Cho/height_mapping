/*
 * StatMeanEstimator.cpp
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/height_estimators/StatMeanEstimator.h"

namespace height_mapping {
void StatMeanEstimator::estimate(grid_map::HeightMap &map,
                                 const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a "
                 "different frame! \n";
    return;
  }

  map.addLayer("n_measured", 0.0);

  auto &height_matrix = map.getHeightMatrix();
  auto &variance_matrix = map.getVarianceMatrix();
  auto &n_measured_matrix = map["n_measured"];

  grid_map::Index cell_index;
  grid_map::Position cell_position;
  for (const auto &point : cloud) {
    // Skip if the point is out of the map
    cell_position << point.x, point.y;
    if (!map.getIndex(cell_position, cell_index))
      continue;

    auto &height = height_matrix(cell_index(0), cell_index(1));
    auto &variance = variance_matrix(cell_index(0), cell_index(1));
    auto &n_measured = n_measured_matrix(cell_index(0), cell_index(1));

    // Initialize the height and variance if it is NaN
    if (map.isEmptyAt(cell_index)) {
      height = point.z;
      variance = 0.0f;
      n_measured = 1.0f;
      continue;
    }

    updateStats(height, variance, n_measured, point.z);
  }
}

void StatMeanEstimator::estimate(grid_map::HeightMap &map,
                                 const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a "
                 "different frame! \n";
    return;
  }

  map.addLayer("intensity");
  map.addLayer("n_measured", 0.0);

  auto &height_matrix = map.getHeightMatrix();
  auto &variance_matrix = map.getVarianceMatrix();
  auto &n_measured_matrix = map["n_measured"];

  auto &intensity_matrix = map["intensity"];

  grid_map::Index cell_index;
  grid_map::Position cell_position;
  for (const auto &point : cloud) {
    // Skip if the point is out of the map
    cell_position << point.x, point.y;
    if (!map.getIndex(cell_position, cell_index))
      continue;

    auto &height = height_matrix(cell_index(0), cell_index(1));
    auto &variance = variance_matrix(cell_index(0), cell_index(1));
    auto &n_measured = n_measured_matrix(cell_index(0), cell_index(1));
    auto &intensity = intensity_matrix(cell_index(0), cell_index(1));

    // Initialize the height and variance if it is NaN
    if (map.isEmptyAt(cell_index)) {
      height = point.z;
      variance = 0.0f;
      n_measured = 1.0f;
      intensity = point.intensity;
      continue;
    }

    updateStats(height, variance, n_measured, point.z);
    updateStats(intensity, variance, n_measured, point.intensity);
  }
}

void StatMeanEstimator::estimate(
    grid_map::HeightMap &map, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a "
                 "different frame! \n";
    return;
  }

  map.addLayer("r");
  map.addLayer("g");
  map.addLayer("b");
  map.addLayer("color");
  map.addLayer("n_measured", 0.0);

  auto &height_matrix = map.getHeightMatrix();
  auto &variance_matrix = map.getVarianceMatrix();
  auto &n_measured_matrix = map["n_measured"];

  auto &r_matrix = map["r"];
  auto &g_matrix = map["g"];
  auto &b_matrix = map["b"];
  auto &color_matrix = map["color"];

  grid_map::Index cell_index;
  grid_map::Position cell_position;
  for (const auto &point : cloud) {
    // Skip if the point is out of the map
    cell_position << point.x, point.y;
    if (!map.getIndex(cell_position, cell_index))
      continue;

    auto &height = height_matrix(cell_index(0), cell_index(1));
    auto &variance = variance_matrix(cell_index(0), cell_index(1));
    auto &n_measured = n_measured_matrix(cell_index(0), cell_index(1));

    auto &r = r_matrix(cell_index(0), cell_index(1));
    auto &g = g_matrix(cell_index(0), cell_index(1));
    auto &b = b_matrix(cell_index(0), cell_index(1));
    auto &color = color_matrix(cell_index(0), cell_index(1));

    // Initialize the height and variance if it is NaN
    if (map.isEmptyAt(cell_index)) {
      height = point.z;
      variance = 0.0f;
      n_measured = 1.0f;

      r = point.r;
      g = point.g;
      b = point.b;
      grid_map::colorVectorToValue(point.getRGBVector3i(), color);

      continue;
    }

    if (!std::isfinite(r) || !std::isfinite(g) || !std::isfinite(b)) {
      r = point.r;
      g = point.g;
      b = point.b;
    }

    ++n_measured;
    updateStats(height, variance, n_measured, point.z);
    updateMean(r, n_measured, point.r);
    updateMean(g, n_measured, point.g);
    updateMean(b, n_measured, point.b);
    grid_map::colorVectorToValue(Eigen::Vector3i(r, g, b), color);
  }
}

} // namespace height_mapping