/*
 * StatMeanEstimator.cpp
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/height_estimators/StatMeanEstimator.h"

/*
StatMeanEstimator tracks:
- elevation
- elevation_min
- elevation_max
- variance
- n_measured
- intensity (if available)
- color (if available)
*/
namespace height_mapping {

void StatMeanEstimator::estimate(grid_map::HeightMap &map,
                                 const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "\033[0;31m[HeightEstimator]: Frame ID mismatch - pointcloud "
                 "is in a different frame! \033[0m\n";
    return;
  }

  auto &heightMatrix = map.getHeightMatrix();
  auto &minHeightMatrix = map.getMinHeightMatrix();
  auto &maxHeightMatrix = map.getMaxHeightMatrix();
  auto &varianceMatrix = map.getVarianceMatrix();

  map.addLayer("n_measured", 0.0);
  auto &numMeasuredMatrix = map["n_measured"];

  map.addLayer("standard_error");
  auto &standardErrorMatrix = map["standard_error"];

  map.addLayer("confidence_interval");
  auto &confidenceIntervalMatrix = map["confidence_interval"];

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
    auto &variance = varianceMatrix(measuredIndex(0), measuredIndex(1));
    auto &nPoints = numMeasuredMatrix(measuredIndex(0), measuredIndex(1));
    auto &standardError =
        standardErrorMatrix(measuredIndex(0), measuredIndex(1));
    auto &confidenceInterval =
        confidenceIntervalMatrix(measuredIndex(0), measuredIndex(1));

    // Initialize the height and variance if empty (NaN values)
    if (map.isEmptyAt(measuredIndex)) {
      height = newPoint.z;
      minHeight = newPoint.z;
      maxHeight = newPoint.z;
      variance = 0.0f; // Single measurement -> no variance
      nPoints = 1;
      standardError = 2.0f; // Assume max error
      confidenceInterval = 2.0f;
      continue;
    }

    ++nPoints;

    // Height estimates
    updateHeightStats(height, variance, nPoints, newPoint.z);
    minHeight = std::min(minHeight, newPoint.z);
    maxHeight = std::max(maxHeight, newPoint.z);

    // Statistics
    standardError = getStandardError(nPoints, variance);
    confidenceInterval = getConfidenceInterval(nPoints, variance);
  }
}

void StatMeanEstimator::estimate(grid_map::HeightMap &map,
                                 const pcl::PointCloud<pcl::PointXYZI> &cloud) {
  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "\033[0;31m[HeightEstimator]: Frame ID mismatch - pointcloud "
                 "is in a different frame! \033[0m\n";
    return;
  }

  auto &heightMatrix = map.getHeightMatrix();
  auto &minHeightMatrix = map.getMinHeightMatrix();
  auto &maxHeightMatrix = map.getMaxHeightMatrix();
  auto &varianceMatrix = map.getVarianceMatrix();

  map.addLayer("n_measured", 0.0);
  auto &numMeasuredMatrix = map["n_measured"];

  map.addLayer("intensity");
  auto &intensityMatrix = map["intensity"];

  map.addLayer("standard_error");
  auto &standardErrorMatrix = map["standard_error"];

  map.addLayer("confidence_interval");
  auto &confidenceIntervalMatrix = map["confidence_interval"];

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
    auto &variance = varianceMatrix(measuredIndex(0), measuredIndex(1));
    auto &nPoints = numMeasuredMatrix(measuredIndex(0), measuredIndex(1));
    auto &intensity = intensityMatrix(measuredIndex(0), measuredIndex(1));
    auto &standardError =
        standardErrorMatrix(measuredIndex(0), measuredIndex(1));
    auto &confidenceInterval =
        confidenceIntervalMatrix(measuredIndex(0), measuredIndex(1));

    // Initialize the height and variance if empty (NaN values)
    if (map.isEmptyAt(measuredIndex)) {
      height = newPoint.z;
      minHeight = newPoint.z;
      maxHeight = newPoint.z;
      variance = 0.0f; // Single measurement -> no variance
      nPoints = 1;
      intensity = newPoint.intensity;
      standardError = 2.0f; // Assume max error
      confidenceInterval = 2.0f;
      continue;
    }

    ++nPoints;

    // Height estimates
    updateHeightStats(height, variance, nPoints, newPoint.z);
    minHeight = std::min(minHeight, newPoint.z);
    maxHeight = std::max(maxHeight, newPoint.z);

    // Statistics
    standardError = getStandardError(nPoints, variance);
    confidenceInterval = getConfidenceInterval(nPoints, variance);

    // Intensity
    meanFilter(intensity, nPoints, newPoint.intensity);
  }
}

void StatMeanEstimator::estimate(
    grid_map::HeightMap &map, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "\033[0;31m[HeightEstimator]: Frame ID mismatch - pointcloud "
                 "is in a different frame! \033[0m\n";
    return;
  }

  auto &heightMatrix = map.getHeightMatrix();
  auto &minHeightMatrix = map.getMinHeightMatrix();
  auto &maxHeightMatrix = map.getMaxHeightMatrix();
  auto &varianceMatrix = map.getVarianceMatrix();

  map.addLayer("n_measured", 0.0);
  auto &numMeasuredMatrix = map["n_measured"];

  map.addLayer("standard_error");
  auto &standardErrorMatrix = map["standard_error"];

  map.addLayer("confidence_interval");
  auto &confidenceIntervalMatrix = map["confidence_interval"];

  map.addLayer("r");
  map.addLayer("g");
  map.addLayer("b");
  map.addLayer("color");
  auto &r_matrix = map["r"];
  auto &g_matrix = map["g"];
  auto &b_matrix = map["b"];
  auto &color_matrix = map["color"];

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
    auto &variance = varianceMatrix(measuredIndex(0), measuredIndex(1));
    auto &nPoints = numMeasuredMatrix(measuredIndex(0), measuredIndex(1));
    auto &standardError =
        standardErrorMatrix(measuredIndex(0), measuredIndex(1));
    auto &confidenceInterval =
        confidenceIntervalMatrix(measuredIndex(0), measuredIndex(1));

    auto &r = r_matrix(measuredIndex(0), measuredIndex(1));
    auto &g = g_matrix(measuredIndex(0), measuredIndex(1));
    auto &b = b_matrix(measuredIndex(0), measuredIndex(1));
    auto &color = color_matrix(measuredIndex(0), measuredIndex(1));

    // Initialize the height and variance if empty (NaN values)
    if (map.isEmptyAt(measuredIndex)) {
      height = newPoint.z;
      minHeight = newPoint.z;
      maxHeight = newPoint.z;
      variance = 0.0f; // Single measurement -> no variance
      nPoints = 1;
      r = newPoint.r;
      g = newPoint.g;
      b = newPoint.b;
      grid_map::colorVectorToValue(newPoint.getRGBVector3i(), color);
      standardError = 2.0f; // Assume max error
      confidenceInterval = 2.0f;
      continue;
    }

    // Update the color if it is NaN
    if (!std::isfinite(r) || !std::isfinite(g) || !std::isfinite(b)) {
      r = newPoint.r;
      g = newPoint.g;
      b = newPoint.b;
    }

    ++nPoints;
    // Height estimates
    updateHeightStats(height, variance, nPoints, newPoint.z);
    minHeight = std::min(minHeight, newPoint.z);
    maxHeight = std::max(maxHeight, newPoint.z);

    // Statistics
    standardError = getStandardError(nPoints, variance);
    confidenceInterval = getConfidenceInterval(nPoints, variance);

    // Color update
    meanFilter(r, nPoints, newPoint.r);
    meanFilter(g, nPoints, newPoint.g);
    meanFilter(b, nPoints, newPoint.b);
    grid_map::colorVectorToValue(Eigen::Vector3i(r, g, b), color);
  }
}

} // namespace height_mapping