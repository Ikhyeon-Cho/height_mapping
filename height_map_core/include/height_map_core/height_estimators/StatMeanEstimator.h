/*
 * StatMeanEstimator.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef SIMPLE_MEAN_ESTIMATOR_H
#define SIMPLE_MEAN_ESTIMATOR_H

#include "height_map_core/height_estimators/HeightEstimatorBase.h"

namespace height_map
{
class StatMeanEstimator : public HeightEstimatorBase
{
public:
  StatMeanEstimator() = default;
  virtual ~StatMeanEstimator() = default;

  void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZ>& cloud) override;
  void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZI>& cloud) override;
  void estimate(grid_map::HeightMap& map, const pcl::PointCloud<pcl::PointXYZRGB>& cloud) override;

private:

  // Update statistics: mu, sigma2, n
  // recursive update of mean and variance:
  // https://math.stackexchange.com/questions/374881/recursive-formula-for-variance
  void statisticalMeanUpdate(float& height, float& variance, float& n_measured, float point_height)
  {
    auto prev_height = height;
    n_measured += 1;
    height += (point_height - height) / n_measured;
    variance += std::pow(prev_height, 2) - std::pow(height, 2) +
                (std::pow(point_height, 2) - variance - std::pow(prev_height, 2)) / n_measured;
  }
};
}  // namespace height_map

#endif /* SIMPLE_MEAN_ESTIMATOR_H */