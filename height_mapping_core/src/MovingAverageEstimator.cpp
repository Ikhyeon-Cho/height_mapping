/*
 * MovingAverageEstimator.cpp
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/height_estimators/MovingAverageEstimator.h"

namespace height_mapping {
void MovingAverageEstimator::estimate(
    grid_map::HeightMap &map, const pcl::PointCloud<pcl::PointXYZ> &cloud) {

  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a "
                 "different frame! \n";
    return;
  }

  // Prepare matrices
  auto &heightMatrix = map.getHeightMatrix();
  auto &minHeightMatrix = map.getMinHeightMatrix();
  auto &maxHeightMatrix = map.getMaxHeightMatrix();

  map.addLayer("n_measured", 0.0);
  auto &numMeasuredMatrix = map["n_measured"];

  grid_map::Index measuredIndex;
  grid_map::Position measuredPosition;

  for (const auto &newPoint : cloud) {
    // Skip if the point is out of the map
    measuredPosition << newPoint.x, newPoint.y;
    if (!map.getIndex(measuredPosition, measuredIndex))
      continue;

    auto &height = heightMatrix(measuredIndex(0), measuredIndex(1));
    auto &minHeight = minHeightMatrix(measuredIndex(0), measuredIndex(1));
    auto &maxHeight = maxHeightMatrix(measuredIndex(0), measuredIndex(1));
    auto &nPoints = numMeasuredMatrix(measuredIndex(0), measuredIndex(1));

    // Initialize the height and variance if it is NaN
    if (map.isEmptyAt(measuredIndex)) {
      height = newPoint.z;
      minHeight = newPoint.z;
      maxHeight = newPoint.z;
      nPoints = 1;
      continue;
    }

    ++nPoints;
    movingAveageUpdate(height, newPoint.z, params_.alpha);
    minHeight = std::min(minHeight, newPoint.z);
    maxHeight = std::max(maxHeight, newPoint.z);
  }
}

void MovingAverageEstimator::estimate(
    grid_map::HeightMap &map, const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a "
                 "different frame! \n";
    return;
  }

  auto &heightMatrix = map.getHeightMatrix();
  auto &minHeightMatrix = map.getMinHeightMatrix();
  auto &maxHeightMatrix = map.getMaxHeightMatrix();

  map.addLayer("intensity");
  auto &intensityMatrix = map["intensity"];

  grid_map::Index measuredIndex;
  grid_map::Position measuredPosition;

  for (const auto &newPoint : cloud) {
    // Skip if the point is out of the map
    measuredPosition << newPoint.x, newPoint.y;
    if (!map.getIndex(measuredPosition, measuredIndex))
      continue;

    auto &height = heightMatrix(measuredIndex(0), measuredIndex(1));
    auto &intensity = intensityMatrix(measuredIndex(0), measuredIndex(1));
    auto &min_height = minHeightMatrix(measuredIndex(0), measuredIndex(1));
    auto &max_height = maxHeightMatrix(measuredIndex(0), measuredIndex(1));

    // Initialize the height and variance if it is NaN
    if (map.isEmptyAt(measuredIndex)) {
      height = min_height = max_height = newPoint.z;
      intensity = newPoint.intensity;
      continue;
    }
    movingAveageUpdate(height, newPoint.z, params_.alpha);
    movingAveageUpdate(intensity, newPoint.intensity, params_.alpha);
    min_height = std::min(min_height, newPoint.z);
    max_height = std::max(max_height, newPoint.z);
  }
}

void MovingAverageEstimator::estimate(
    grid_map::HeightMap &map, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {

  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a "
                 "different frame! \n";
    return;
  }

  auto &heightMatrix = map.getHeightMatrix();
  auto &minHeightMatrix = map.getMinHeightMatrix();
  auto &maxHeightMatrix = map.getMaxHeightMatrix();

  map.addLayer("r");
  map.addLayer("g");
  map.addLayer("b");
  map.addLayer("color");
  auto &red_matrix = map["r"];
  auto &green_matrix = map["g"];
  auto &blue_matrix = map["b"];
  auto &color_matrix = map["color"];

  grid_map::Index measuredIndex;
  grid_map::Position measuredPosition;

  for (const auto &newPoint : cloud) {
    // Skip if the point is out of the map
    measuredPosition << newPoint.x, newPoint.y;
    if (!map.getIndex(measuredPosition, measuredIndex))
      continue;

    auto &height = heightMatrix(measuredIndex(0), measuredIndex(1));
    auto &min_height = minHeightMatrix(measuredIndex(0), measuredIndex(1));
    auto &max_height = maxHeightMatrix(measuredIndex(0), measuredIndex(1));
    auto &red = red_matrix(measuredIndex(0), measuredIndex(1));
    auto &green = green_matrix(measuredIndex(0), measuredIndex(1));
    auto &blue = blue_matrix(measuredIndex(0), measuredIndex(1));
    auto &color = color_matrix(measuredIndex(0), measuredIndex(1));

    // Initialize the height and variance if it is NaN
    if (map.isEmptyAt(measuredIndex)) {
      height = min_height = max_height = newPoint.z;
      red = newPoint.r;
      green = newPoint.g;
      blue = newPoint.b;
      grid_map::colorVectorToValue(newPoint.getRGBVector3i(), color);

      continue;
    }

    // Height estimates
    movingAveageUpdate(height, newPoint.z, params_.alpha);
    min_height = std::min(min_height, newPoint.z);
    max_height = std::max(max_height, newPoint.z);

    // Color estimates
    movingAveageUpdate(red, newPoint.r, params_.alpha);
    movingAveageUpdate(green, newPoint.g, params_.alpha);
    movingAveageUpdate(blue, newPoint.b, params_.alpha);
    grid_map::colorVectorToValue(Eigen::Vector3i(red, green, blue), color);
  }
}

} // namespace height_mapping