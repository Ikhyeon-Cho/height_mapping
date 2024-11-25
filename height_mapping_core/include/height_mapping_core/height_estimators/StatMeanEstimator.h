/*
 * StatMeanEstimator.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_mapping_core/height_estimators/HeightEstimatorBase.h"

namespace height_mapping {
class StatMeanEstimator : public HeightEstimatorBase {
public:
  StatMeanEstimator() = default;
  ~StatMeanEstimator() override = default;

  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZ> &cloud) override;
  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZI> &cloud) override;
  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZRGB> &cloud) override;

private:
  // Update statistics: mu, sigma2, n
  // recursive update of mean and variance:
  // https://math.stackexchange.com/questions/374881/recursive-formula-for-variance
  void updateStats(float &height, float &variance, float n_measured,
                   float point_height) {
    const float prev_height = height;
    height += (point_height - height) / n_measured;
    variance +=
        std::pow(prev_height, 2) - std::pow(height, 2) +
        (std::pow(point_height, 2) - variance - std::pow(prev_height, 2)) /
            n_measured;
  }
  void updateMean(float &attribute, float n_measured, float point_attribute) {
    attribute += (point_attribute - attribute) / n_measured;
  }
};
} // namespace height_mapping
