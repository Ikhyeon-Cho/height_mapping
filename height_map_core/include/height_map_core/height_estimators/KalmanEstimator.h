/*
 * KalmanEstimator.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef KALMAN_HEIGHT_ESTIMATOR_H
#define KALMAN_HEIGHT_ESTIMATOR_H

#include "height_map_core/height_estimators/HeightEstimatorBase.h"

namespace height_map
{
class KalmanEstimator : public HeightEstimatorBase
{
public:
  KalmanEstimator() = default;
  virtual ~KalmanEstimator() = default;

  void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZ>& cloud) override;
  void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZI>& cloud) override;
  void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZRGB>& cloud) override;

private:
  void KalmanUpdate(float& height, float& variance, float point_height, float point_variance)
  {
    float K = variance / (variance + point_variance);
    height = height + K * (point_height - height);
    variance = (1 - K) * variance;
  }

  float getPointVariance(const pcl::PointXYZ& point) const
  {
    return std::sqrt(point.x * point.x + point.y * point.y);
  }

  float getPointVariance(const pcl::PointXYZI& point) const
  {
    return std::sqrt(point.x * point.x + point.y * point.y);
  }

  float getPointVariance(const pcl::PointXYZRGB& point) const
  {
    return std::sqrt(point.x * point.x + point.y * point.y);
  }
};
}  // namespace height_map

#endif /* KALMAN_HEIGHT_ESTIMATOR_H */