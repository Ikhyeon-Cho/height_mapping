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
  using PointXYZR = pcl::PointXYZI;  // Intensity holds range R from the robot

  HeightMap();

  HeightMap(double length_x, double length_y, double grid_resolution);

  HeightMap(const std::vector<std::string>& layers);

  void update(const pcl::PointCloud<pcl::PointXYZI>& pointcloud);

  const GridMap::Matrix& getElevationLayer() const;

  GridMap::Matrix& getElevationLayer();

  const GridMap::Matrix& getVarianceLayer() const;

  GridMap::Matrix& getVarianceLayer();

  const GridMap::Matrix& getNumMeasuredPointsLayer() const;

  GridMap::Matrix& getNumMeasuredPointsLayer();

  bool isEmptyAt(const Index& index) const;

  bool isEmptyAt(const std::string& layer, const Index& index) const;

  void smoothing();
  // void rayCasting(const Position3 &robotPosition3);

private:  // Helper Functions
  pcl::PointCloud<PointXYZR>::Ptr getDownsampledCloudAtGrid(const pcl::PointCloud<PointXYZR>& pointcloud);

  void updateElevation(const pcl::PointCloud<PointXYZR>& pointcloud);

  void updateSampleVariance(const pcl::PointCloud<PointXYZR>& pointcloud);
};
}  // namespace grid_map

#endif  // HEIGHT_MAP_H