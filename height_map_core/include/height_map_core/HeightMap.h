/*
 * HeightMap.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_H
#define HEIGHT_MAP_H

#include <grid_map_core/grid_map_core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

namespace grid_map
{
class HeightMap : public GridMap
{
public:
  HeightMap(double length_x, double length_y, double grid_resolution);

  /// @brief
  /// @param pointcloud The pointcloud that will be used to update the height map, aligned with the map frame
  /// @param method 1. KalmanFilter 2. EwmaFilter 3. KalmanFilter+ConsistencyCheck
  void update(const pcl::PointCloud<pcl::PointXYZI>& pointcloud);

  const std::string& getHeightLayer() const;
  const GridMap::Matrix& getHeightMatrix() const;
  GridMap::Matrix& getHeightMatrix();

  bool isEmptyAt(const Index& index) const;

  bool isEmptyAt(const std::string& layer, const Index& index) const;

  void smoothing();

  float getMaxHeight() const;
  float getMinHeight() const;
  std::tuple<float, float> getMinMaxHeight() const;

private:
  // Helper Functions
  void doKF(float& mu, float& sigma2, float point_z, float point_sigma2);  // Kalman Filter
  void doEWMA(float& mu, float point_z, float alpha = 0.8);                // Exponential Weighted Moving Average Filter

  // Basic Layers
  std::string layer_height_{ "elevation" };
  std::string layer_variance_{ "variance" };

  // Consistency Check Layers
  std::string layer_min_height_{ "elevation_min" };
  std::string layer_max_height_{ "elevation_max" };
  std::string layer_height_diff_{ "elevation_diff" };

  // Statistics Layers
  std::string layer_measured_num_{ "measured_num" };

  // Intensity Layer: LiDAR intensity
  std::string layer_intensity_{ "intensity" };
};

}  // namespace grid_map

#endif  // HEIGHT_MAP_H