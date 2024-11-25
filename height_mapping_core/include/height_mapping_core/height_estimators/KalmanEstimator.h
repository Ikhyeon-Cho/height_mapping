/*
 * KalmanEstimator.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_mapping_core/height_estimators/HeightEstimatorBase.h"

namespace height_mapping {
class KalmanEstimator : public HeightEstimatorBase {
public:
  KalmanEstimator() = default;
  ~KalmanEstimator() override = default;

  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZ> &cloud) override;
  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZI> &cloud) override;
  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZRGB> &cloud) override;

private:
  static void KalmanUpdate(float &height, float &variance,
                           const float point_height,
                           const float point_variance) {
    const float K = variance / (variance + point_variance);
    height = height + K * (point_height - height);
    variance = (1 - K) * variance;
  }

  template <typename PointT>
  static float getPointVariance(const PointT &point) {
    return std::sqrt(point.x * point.x + point.y * point.y);
  }
};
} // namespace height_mapping
