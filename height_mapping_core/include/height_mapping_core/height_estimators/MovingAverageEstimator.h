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
  struct Parameters {
    float alpha{0.8f};      // Moving average weight [0, 1]:
                            // closer to 1 means more weight to new measurement
    float min_weight{0.1f}; // Minimum weight for stability
    float max_weight{0.9f}; // Maximum weight to ensure adaptability
    bool adaptive_weight{
        false}; // Enable adaptive weight based on height difference
  };

  explicit MovingAverageEstimator() = default;
  ~MovingAverageEstimator() override = default;

  void setMovingAverageWeight(float alpha) { params_.alpha = alpha; }

  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZ> &cloud) override;
  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZI> &cloud) override;
  void estimate(grid_map::HeightMap &map,
                const pcl::PointCloud<pcl::PointXYZRGB> &cloud) override;

private:
  static float clamp(float value, float min, float max) {
    return std::min(std::max(value, min), max);
  }
  /**
   * Update mean with exponential moving average
   * @param current Current value
   * @param new_value New measurement
   * @param alpha Weight factor [0,1], higher means more weight to new
   * measurement
   */
  void movingAveageUpdate(float &current, float new_value, float alpha) const {
    alpha = clamp(alpha, params_.min_weight, params_.max_weight);
    current = ((1.0f - alpha) * current) + (alpha * new_value);
  }

  /**
   * Calculate adaptive weight based on measurement difference
   * Reduces weight for measurements that differ significantly
   */
  float calculateAdaptiveWeight(float current, float new_value) const {
    if (!params_.adaptive_weight)
      return params_.alpha;

    const float diff = std::abs(new_value - current);
    const float max_expected_diff = 0.5f; // Could be made configurable

    // Reduce weight for larger differences
    return params_.alpha * std::exp(-diff / max_expected_diff);
  }

  Parameters params_;
};
} // namespace height_mapping
