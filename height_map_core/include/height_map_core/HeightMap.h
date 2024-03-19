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
  HeightMap();

  HeightMap(double length_x, double length_y, double grid_resolution);

  pcl::PointCloud<pcl::PointXYZI>::Ptr getGridDownsampledCloud(const pcl::PointCloud<pcl::PointXYZI>& pointcloud);

  void update(const pcl::PointCloud<pcl::PointXYZI>& pointcloud, const std::string& method = "KalmanFilter");

  const GridMap::Matrix& getElevationLayer() const;

  GridMap::Matrix& getElevationLayer();

  const GridMap::Matrix& getUncertaintyLayer() const;

  GridMap::Matrix& getUncertaintyLayer();

  bool isEmptyAt(const Index& index) const;

  bool isEmptyAt(const std::string& layer, const Index& index) const;

  void smoothing();

private:
  // Helper Functions
  void doKF(float& mu, float& sigma2, float point_z, float point_sigma2);  // Kalman Filter
  void doEWMA(float& mu, float& sigma2, float point_z, float point_sigma2,
              float alpha = 0.95);  // Exponential Weighted Moving Average Filter

  // Basic Layers
  std::string layer_elevation_{ "elevation" };
  std::string layer_uncertainty_{ "uncertainty" };
  std::string layer_intensity_{ "intensity" };

  // Consistency Check Layers
  std::string layer_min_z_{ "min_z" };
  std::string layer_max_z_{ "max_z" };

  // Statistics Layers
  std::string layer_statistics_mu_{ "statistics_mu" };
  std::string layer_statistics_sigma2_{ "statistics_sigma2" };
  std::string layer_statistics_n_{ "statistics_n" };

  // Grid-based pointcloud downsampling
  std::string layer_downsampled_cloud_{ "downsampled_cloud" };
  std::string layer_downsampled_cloud_intensity_{ "downsampled_cloud_intensity" };
};
}  // namespace grid_map

#endif  // HEIGHT_MAP_H