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
    {

      auto &heightMatrix = map.getHeightMatrix();
      auto &varianceMatrix = map.getVarianceMatrix();
      const float gridResolution = map.getResolution();
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
        rayDir.normalize();
        float rayLength = rayDir.norm();

        // Visibility check through ray sampling
        float samplingStep = gridResolution * 0.5f;
        for (float t = 0; t < rayLength - samplingStep; t += samplingStep) {

          // Get ray point and corresponding map height
          Eigen::Vector3f pointOnRay = sensorOrigin + rayDir * t;
          grid_map::Position checkPosition(pointOnRay.x(), pointOnRay.y());
          grid_map::Index checkIndex;

          // Skip if the ray point is out of the map
          if (!map.getIndex(checkPosition, checkIndex))
            continue;

          // Skip if the ray point is on static terrain
          // if (isStaticAt(map, checkIndex))
            // continue;

          auto &checkPositionHeight =
              heightMatrix(checkIndex(0), checkIndex(1));
          auto &checkPositionVariance =
              varianceMatrix(checkIndex(0), checkIndex(1));

          if (checkPositionHeight > pointOnRay.z() + correctionThreshold_) {
            checkPositionHeight = pointOnRay.z();
            checkPositionVariance =
                std::max(checkPositionVariance,
                         1.0f); // Increase uncertainty: TODO -> improve this
          }
        }
      }
    }
  }

  // Avoid raycasting on static terrain
  bool isStaticAt(const grid_map::HeightMap &map, const grid_map::Index &index);

private:
  float correctionThreshold_{0.0f};
  float heightDiffThreshold_{0.3f};
};

} // namespace height_mapping
