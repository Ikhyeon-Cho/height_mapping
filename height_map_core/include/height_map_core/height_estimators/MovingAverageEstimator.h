/*
 * MovingAverageEstimator.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef EWMA_HEIGHT_ESTIMATOR_H
#define EWMA_HEIGHT_ESTIMATOR_H

#include "height_map_core/height_estimators/HeightEstimatorBase.h"

namespace height_map
{
class MovingAverageEstimator : public HeightEstimatorBase
{
public:
  MovingAverageEstimator() = default;
  virtual ~MovingAverageEstimator() = default;

  void setMovingAverageWeight(float alpha)
  {
    alpha_ = alpha;
  }

  void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZ>& cloud) override;
  void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZI>& cloud) override;
  void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZRGB>& cloud) override;

private:
  void ExponentialWeightedMovingAverageUpdate(float& height, float point_height)
  {
    height = (1 - alpha_) * height + alpha_ * point_height;
  }

  float alpha_{ 0.8 };
};
}  // namespace height_map

#endif /* EWMA_HEIGHT_ESTIMATOR_H */