/*
 * MovingAverageEstimator.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_mapping_core/height_estimators/HeightEstimatorBase.h"

namespace height_mapping {
class MovingAverageEstimator : public HeightEstimatorBase {
public:
  MovingAverageEstimator() = default;
  ~MovingAverageEstimator() override = default;

  void setMovingAverageWeight(float alpha) { alpha_ = alpha; }

  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZ> &cloud) override;
  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZI> &cloud) override;
  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZRGB> &cloud) override;

private:
  static void updateMean(float &height, const float point_height,
                         const float alpha) {
    height = (1 - alpha) * height + alpha * point_height;
  }

  float alpha_{0.8}; // Moving average weight [0, 1]
};
} // namespace height_mapping
