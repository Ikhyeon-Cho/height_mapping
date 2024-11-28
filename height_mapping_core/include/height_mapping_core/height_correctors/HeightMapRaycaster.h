/*
 * HeightMapRaycaster.h
 *
 *  Created on: Nov 28, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_mapping_core/map/HeightMap.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace height_mapping {

class HeightMapRaycaster {
public:
  HeightMapRaycaster() = default;

  template <typename PointT>
  void correctHeight(grid_map::HeightMap &map,
                     const pcl::PointCloud<PointT> &cloud,
                     const Eigen::Vector3f &sensorOrigin) {

    auto &heightMatrix = map.getHeightMatrix();
    auto &varianceMatrix = map.getVarianceMatrix();

    map.addLayer("raycasting");
    auto &raycastingMatrix = map.get("raycasting");
    const float sensorHeight = sensorOrigin.z();

    grid_map::Index measuredIndex;
    grid_map::Position measuredPosition;

    for (const auto &point : cloud.points) {
      // Skip if measured point is out of the map
      measuredPosition << point.x, point.y;
      if (!map.getIndex(measuredPosition, measuredIndex))
        continue;

      // Create ray from two points: sensor to measured point
      Eigen::Vector3f rayDir(point.x - sensorOrigin.x(),
                             point.y - sensorOrigin.y(),
                             point.z - sensorOrigin.z());
      float rayLength = rayDir.norm();
      rayDir.normalize();

      // Visibility check through ray
      float samplingStep = map.getResolution() * 1.0f; // 0.5
      for (float t = 0; t < rayLength - samplingStep; t += samplingStep) {

        // Get ray point
        Eigen::Vector3f pointOnRay = sensorOrigin + rayDir * t;

        // Get ray point index
        grid_map::Position checkPosition(pointOnRay.x(), pointOnRay.y());
        grid_map::Index checkIndex;
        // Skip if the ray point is out of the map
        if (!map.getIndex(checkPosition, checkIndex))
          continue;

        if (isStaticAt(map, checkIndex))
          continue;

        // Get map height and variance at the ray point
        auto &mapHeight = heightMatrix(checkIndex(0), checkIndex(1));
        auto &mapHeightVariance = varianceMatrix(checkIndex(0), checkIndex(1));
        auto &rayHeight = raycastingMatrix(checkIndex(0), checkIndex(1));
        rayHeight = pointOnRay.z();

        // Update height if current height is higher than the ray point
        if (mapHeight > pointOnRay.z() + correctionThreshold_) {
          mapHeight = pointOnRay.z();
        }
      }
    }
  }

  // Avoid raycasting on static terrain
  bool isStaticAt(const grid_map::HeightMap &map, const grid_map::Index &index);

private:
  float correctionThreshold_{0.0f};
  float heightDiffThreshold_{0.55f};
}; // namespace height_mapping

} // namespace height_mapping
